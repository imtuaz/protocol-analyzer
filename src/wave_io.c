#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "wave_io.h"

/* ---------------------------------------------------------------- CSV */

int wf_save_csv(const waveform_t *w, const char *path)
{
    FILE *f = fopen(path, "w");
    if (!f)
        return -1;

    fprintf(f, "# sample_rate=%u\n", w->sample_rate);
    fprintf(f, "time_s");
    for (int c = 0; c < w->num_ch; c++)
        fprintf(f, ",%s", w->name[c]);
    fprintf(f, "\n");

    /* Write first sample, every sample where any channel changes, last sample */
    for (uint32_t i = 0; i < w->len; i++) {
        int changed = (i == 0) || (i == w->len - 1);
        for (int c = 0; c < w->num_ch && !changed && i > 0; c++)
            if (w->data[c][i] != w->data[c][i - 1])
                changed = 1;
        if (!changed)
            continue;
        fprintf(f, "%.9f", (double)i / w->sample_rate);
        for (int c = 0; c < w->num_ch; c++)
            fprintf(f, ",%d", w->data[c][i]);
        fprintf(f, "\n");
    }
    fclose(f);
    return 0;
}

typedef struct {
    double  t;
    uint8_t v[WF_MAX_CH];
} csv_row_t;

int wf_load_csv(waveform_t *w, const char *path, uint32_t sample_rate)
{
    FILE *f = fopen(path, "r");
    char  line[512];
    const char *names[WF_MAX_CH];
    char  name_buf[WF_MAX_CH][WF_NAME_LEN];
    int   num_ch = 0;

    if (!f)
        return -1;

    /* Optional comment lines ('#' or ';'). Our own files store the
     * original sample rate as "# sample_rate=N". */
    uint32_t file_rate = 0;
    for (;;) {
        if (!fgets(line, sizeof line, f)) {
            fclose(f);
            return -1;
        }
        if (line[0] != '#' && line[0] != ';')
            break;
        unsigned long r;
        if (sscanf(line, "# sample_rate=%lu", &r) == 1)
            file_rate = (uint32_t)r;
    }
    if (sample_rate == 0)
        sample_rate = file_rate;

    /* Header: time,name0,name1,... */
    char *tok = strtok(line, ",\r\n");          /* skip time column */
    while ((tok = strtok(NULL, ",\r\n")) != NULL && num_ch < WF_MAX_CH) {
        while (*tok == ' ')
            tok++;
        snprintf(name_buf[num_ch], WF_NAME_LEN, "%s", tok);
        names[num_ch] = name_buf[num_ch];
        num_ch++;
    }
    if (num_ch == 0) {
        fclose(f);
        return -1;
    }

    /* Read all rows into memory */
    csv_row_t *rows = NULL;
    int n = 0, cap = 0;
    while (fgets(line, sizeof line, f)) {
        csv_row_t r;
        char *p = line;
        char *end;
        r.t = strtod(p, &end);
        if (end == p)
            continue;                            /* not a number: skip */
        p = end;
        for (int c = 0; c < num_ch; c++) {
            while (*p == ',' || *p == ' ')
                p++;
            r.v[c] = (uint8_t)(strtol(p, &end, 10) ? 1 : 0);
            p = end;
        }
        if (n == cap) {
            cap = cap ? cap * 2 : 256;
            csv_row_t *tmp = realloc(rows, (size_t)cap * sizeof *rows);
            if (!tmp) {
                free(rows);
                fclose(f);
                return -1;
            }
            rows = tmp;
        }
        rows[n++] = r;
    }
    fclose(f);
    if (n == 0) {
        free(rows);
        return -1;
    }

    /* Guess: 10 samples in the shortest pulse, so edges keep enough
     * time resolution for decoding and timing checks */
    if (sample_rate == 0) {
        double min_dt = 1e9;
        for (int i = 1; i < n; i++) {
            double dt = rows[i].t - rows[i - 1].t;
            if (dt > 1e-12 && dt < min_dt)
                min_dt = dt;
        }
        sample_rate = (min_dt < 1e9) ? (uint32_t)(10.0 / min_dt + 0.5) : 1000000;
        if (sample_rate > 100000000u)
            sample_rate = 100000000u;
    }

    if (wf_init(w, sample_rate, num_ch, names) != 0) {
        free(rows);
        return -1;
    }

    /* Each row's level is held until the time of the next row */
    double t0 = rows[0].t;
    for (int i = 0; i < n; i++) {
        uint32_t until = (i + 1 < n)
            ? (uint32_t)llround((rows[i + 1].t - t0) * sample_rate)
            : w->len + 1;
        if (until > w->len)
            wf_hold(w, rows[i].v, until - w->len);
    }
    free(rows);
    return 0;
}

/* ---------------------------------------------------------------- VCD */

int wf_save_vcd(const waveform_t *w, const char *path)
{
    FILE *f = fopen(path, "w");
    if (!f)
        return -1;

    fprintf(f, "$timescale 1ns $end\n$scope module capture $end\n");
    for (int c = 0; c < w->num_ch; c++)
        fprintf(f, "$var wire 1 %c %s $end\n", '!' + c, w->name[c]);
    fprintf(f, "$upscope $end\n$enddefinitions $end\n");

    for (uint32_t i = 0; i < w->len; i++) {
        int first = 1;
        for (int c = 0; c < w->num_ch; c++) {
            if (i == 0 || w->data[c][i] != w->data[c][i - 1]) {
                if (first) {
                    unsigned long long t_ns =
                        (unsigned long long)i * 1000000000ull / w->sample_rate;
                    fprintf(f, "#%llu\n", t_ns);
                    first = 0;
                }
                fprintf(f, "%d%c\n", w->data[c][i], '!' + c);
            }
        }
    }
    fprintf(f, "#%llu\n",
            (unsigned long long)w->len * 1000000000ull / w->sample_rate);
    fclose(f);
    return 0;
}
