#include <stdio.h>
#include <string.h>
#include "report.h"

/* Format a sample index as time with a readable unit */
static void fmt_time(char *buf, int size, uint32_t sample, uint32_t rate)
{
    double t = (double)sample / rate;
    if (t < 1e-3)
        snprintf(buf, (size_t)size, "%.2f us", t * 1e6);
    else if (t < 1.0)
        snprintf(buf, (size_t)size, "%.3f ms", t * 1e3);
    else
        snprintf(buf, (size_t)size, "%.4f s", t);
}

/* ============================================================ terminal */

void print_events(FILE *f, const waveform_t *w, const event_list_t *ev)
{
    char t[24], lab[40], err[80];
    fprintf(f, "  %-12s %-5s %-7s %-12s %-16s %s\n",
            "time", "proto", "type", "value", "errors", "details");
    fprintf(f, "  ------------ ----- ------- ------------ ---------------- -------\n");
    for (int i = 0; i < ev->count; i++) {
        const event_t *e = &ev->items[i];
        fmt_time(t, sizeof t, e->start, w->sample_rate);
        if (e->type == EV_TIMING || e->type == EV_ERROR)
            lab[0] = '\0';
        else
            ev_label(e, lab, sizeof lab);
        err_to_string(e->errors, err, sizeof err);
        fprintf(f, "  %-12s %-5s %-7s %-12s %-16s %s\n", t, proto_name(e->proto),
                ev_type_name(e->type), lab, err[0] ? err : "-", e->note);
    }
}

void print_summary(FILE *f, const event_list_t *ev)
{
    int per_err[ERR_COUNT] = { 0 };
    for (int i = 0; i < ev->count; i++)
        for (int b = 0; b < ERR_COUNT; b++)
            if (ev->items[i].errors & (1u << b))
                per_err[b]++;
    fprintf(f, "  %d events, %d with errors", ev->count, ev_count_errors(ev));
    for (int b = 0; b < ERR_COUNT; b++)
        if (per_err[b])
            fprintf(f, " | %s: %d", err_name(b), per_err[b]);
    fprintf(f, "\n");
}

/* ================================================================ HTML */

/* Escape text for HTML (notes may contain ' < > &) */
static void html_text(FILE *f, const char *s)
{
    for (; *s; s++) {
        switch (*s) {
        case '<': fputs("&lt;", f); break;
        case '>': fputs("&gt;", f); break;
        case '&': fputs("&amp;", f); break;
        case '"': fputs("&quot;", f); break;
        default:  fputc(*s, f);
        }
    }
}

#define LABEL_W  70     /* left column for channel names   */
#define ROW_H    34     /* height of one digital channel   */
#define ANN_H    30     /* height of the annotation row    */
#define AXIS_H   22

static const char *ev_color(const event_t *e)
{
    if (e->errors & ~ERR_TIMING) return "#e5484d";       /* red: error  */
    if (e->errors)               return "#f5a524";       /* orange: timing */
    switch (e->type) {
    case EV_ADDR:  return "#3e63dd";
    case EV_START:
    case EV_STOP:  return "#8b8d98";
    case EV_BREAK: return "#8e4ec6";
    case EV_FRAME: return "#12a594";
    default:       return "#30a46c";
    }
}

static void svg_waveform(FILE *f, const waveform_t *w, const event_list_t *ev)
{
    if (w->len < 2)
        return;

    /* Make the drawing wider when there are many events so labels fit */
    int n_vis = 0;
    for (int i = 0; i < ev->count; i++)
        if (ev->items[i].type != EV_TIMING)
            n_vis++;
    int plot_w = 1000;
    if (n_vis * 46 > plot_w)
        plot_w = n_vis * 46;
    if (plot_w > 12000)
        plot_w = 12000;

    int width  = LABEL_W + plot_w + 10;
    int height = w->num_ch * ROW_H + ANN_H + AXIS_H + 8;
    double sx  = (double)plot_w / (w->len - 1);   /* pixels per sample */

    fprintf(f, "<div class='wave'><svg width='%d' height='%d' "
               "xmlns='http://www.w3.org/2000/svg'>\n", width, height);

    /* Time axis with 10 ticks (drawn first so it stays behind) */
    int yt = w->num_ch * ROW_H + ANN_H + 4;
    for (int k = 0; k <= 10; k++) {
        uint32_t s = (uint32_t)((double)(w->len - 1) * k / 10);
        char t[24];
        fmt_time(t, sizeof t, s, w->sample_rate);
        double x = LABEL_W + s * sx;
        fprintf(f, "<line x1='%.1f' y1='0' x2='%.1f' y2='%d' class='grid'/>"
                   "<text x='%.1f' y='%d' class='tick'>%s</text>\n",
                x, x, yt, x, yt + 14, t);
    }

    /* Digital channels: draw only the edges (fast even for long captures) */
    for (int c = 0; c < w->num_ch; c++) {
        int y_hi = c * ROW_H + 6, y_lo = c * ROW_H + ROW_H - 6;
        fprintf(f, "<text x='4' y='%d' class='ch'>%s</text>\n",
                c * ROW_H + ROW_H / 2 + 4, w->name[c]);
        fprintf(f, "<path class='sig' d='M%d %d", LABEL_W,
                w->data[c][0] ? y_hi : y_lo);
        uint32_t i = 0;
        while ((i = wf_next_edge(w, c, i)) < w->len) {
            fprintf(f, " H%.1f V%d", LABEL_W + i * sx, w->data[c][i] ? y_hi : y_lo);
        }
        fprintf(f, " H%d'/>\n", LABEL_W + plot_w);
    }

    /* Annotation row: one box per decoded item, hover for details */
    int ya = w->num_ch * ROW_H + 2;
    fprintf(f, "<text x='4' y='%d' class='ch'>decode</text>\n", ya + 17);
    for (int i = 0; i < ev->count; i++) {
        const event_t *e = &ev->items[i];
        char lab[40], err[80], tip[200];
        if (e->type == EV_TIMING && !e->errors)
            continue;                             /* info only */
        double x0 = LABEL_W + e->start * sx;
        double x1 = LABEL_W + e->end * sx;
        if (x1 - x0 < 3)
            x1 = x0 + 3;
        ev_label(e, lab, sizeof lab);
        err_to_string(e->errors, err, sizeof err);
        snprintf(tip, sizeof tip, "%s %s %s %s", ev_type_name(e->type), lab,
                 err[0] ? err : "", e->note);

        fprintf(f, "<g><title>");
        html_text(f, tip);
        if (e->type == EV_TIMING) {
            /* timing violation: thin orange bar under the decoded items */
            fprintf(f, "</title><rect x='%.1f' y='%d' width='%.1f' height='4' "
                       "fill='#f5a524'/></g>\n", x0, ya + ANN_H - 3,
                    (x1 - x0 < 6) ? 6.0 : x1 - x0);
            continue;
        }
        fprintf(f, "</title><rect x='%.1f' y='%d' width='%.1f' height='%d' rx='3' "
                   "fill='%s' fill-opacity='0.85'/>", x0, ya, x1 - x0, ANN_H - 6,
                ev_color(e));
        /* Text only if the box is wide enough to hold it */
        if ((x1 - x0) > 7.0 * strlen(lab)) {
            fprintf(f, "<text x='%.1f' y='%d' class='lab'>", (x0 + x1) / 2, ya + 15);
            html_text(f, lab);
            fprintf(f, "</text>");
        }
        fprintf(f, "</g>\n");
    }

    fprintf(f, "</svg></div>\n");
}

static void html_table(FILE *f, const waveform_t *w, const event_list_t *ev)
{
    fprintf(f, "<table><tr><th>#</th><th>time</th><th>type</th><th>value</th>"
               "<th>errors</th><th>details</th></tr>\n");
    for (int i = 0; i < ev->count; i++) {
        const event_t *e = &ev->items[i];
        char t[24], lab[40], err[80];
        fmt_time(t, sizeof t, e->start, w->sample_rate);
        if (e->type == EV_TIMING || e->type == EV_ERROR)
            lab[0] = '\0';
        else
            ev_label(e, lab, sizeof lab);
        err_to_string(e->errors, err, sizeof err);
        fprintf(f, "<tr%s><td>%d</td><td>%s</td><td>%s</td><td>", e->errors ? " class='bad'" : "",
                i, t, ev_type_name(e->type));
        html_text(f, lab);
        fprintf(f, "</td><td>%s</td><td>", err);
        html_text(f, e->note);
        fprintf(f, "</td></tr>\n");
    }
    fprintf(f, "</table>\n");
}

int report_write_html(const char *path, const char *title,
                      const report_section_t *sec, int n)
{
    FILE *f = fopen(path, "w");
    if (!f)
        return -1;

    fprintf(f, "<!DOCTYPE html><html><head><meta charset='utf-8'><title>");
    html_text(f, title);
    fprintf(f, "</title><style>\n"
        "body{font-family:system-ui,sans-serif;margin:24px;color:#1c2024;background:#fcfcfd}\n"
        "h1{font-size:22px;margin:0 0 4px} h2{font-size:17px;margin:28px 0 6px}\n"
        ".info{font:13px ui-monospace,monospace;background:#f0f0f3;padding:8px 10px;"
        "border-radius:6px;white-space:pre-wrap}\n"
        ".cards{display:flex;gap:10px;flex-wrap:wrap;margin:12px 0}\n"
        ".card{background:#fff;border:1px solid #e0e1e6;border-radius:8px;padding:8px 14px}\n"
        ".card b{display:block;font-size:20px}\n"
        ".wave{overflow-x:auto;border:1px solid #e0e1e6;border-radius:8px;background:#fff;"
        "padding:6px;margin:8px 0}\n"
        ".sig{fill:none;stroke:#0d74ce;stroke-width:1.5}\n"
        ".ch{font:12px ui-monospace,monospace;fill:#60646c}\n"
        ".lab{font:11px ui-monospace,monospace;fill:#fff;text-anchor:middle}\n"
        ".tick{font:10px ui-monospace,monospace;fill:#80838d;text-anchor:middle}\n"
        ".grid{stroke:#eceef0;stroke-width:1}\n"
        "table{border-collapse:collapse;font:12px ui-monospace,monospace;margin:6px 0}\n"
        "td,th{border-bottom:1px solid #e8e8ec;padding:3px 10px;text-align:left}\n"
        "tr.bad td{background:#feebec;color:#ce2c31}\n"
        "details{margin:6px 0} summary{cursor:pointer;color:#60646c}\n"
        "</style></head><body>\n<h1>");
    html_text(f, title);
    fprintf(f, "</h1><div style='color:#60646c'>Embedded Communication Protocol Analyzer</div>\n");

    /* Overall summary cards */
    int total = 0, bad = 0;
    for (int s = 0; s < n; s++) {
        total += sec[s].ev->count;
        bad   += ev_count_errors(sec[s].ev);
    }
    fprintf(f, "<div class='cards'><div class='card'><b>%d</b>captures</div>"
               "<div class='card'><b>%d</b>decoded events</div>"
               "<div class='card'><b style='color:%s'>%d</b>events with errors</div></div>\n",
            n, total, bad ? "#ce2c31" : "#18794e", bad);

    for (int s = 0; s < n; s++) {
        const report_section_t *r = &sec[s];
        int per_err[ERR_COUNT] = { 0 };
        for (int i = 0; i < r->ev->count; i++)
            for (int b = 0; b < ERR_COUNT; b++)
                if (r->ev->items[i].errors & (1u << b))
                    per_err[b]++;

        fprintf(f, "<h2>");
        html_text(f, r->title);
        fprintf(f, "</h2><div class='info'>");
        html_text(f, r->info);
        fprintf(f, "\nErrors:");
        int any = 0;
        for (int b = 0; b < ERR_COUNT; b++)
            if (per_err[b]) {
                fprintf(f, " %s=%d", err_name(b), per_err[b]);
                any = 1;
            }
        fprintf(f, "%s</div>\n", any ? "" : " none");
        svg_waveform(f, r->w, r->ev);
        fprintf(f, "<details><summary>Event table (%d)</summary>\n", r->ev->count);
        html_table(f, r->w, r->ev);
        fprintf(f, "</details>\n");
    }
    fprintf(f, "<p style='color:#80838d;font-size:12px'>Hover a box in the decode row "
               "for details. Red = protocol error, orange = timing violation.</p>"
               "</body></html>\n");
    fclose(f);
    return 0;
}
