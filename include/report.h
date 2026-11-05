/*
 * report.h - show decoded results: terminal table and HTML report.
 *
 * The HTML report draws every waveform as SVG (no external library),
 * with coloured boxes for decoded items (hover a box for details) and
 * a table of all events. Open it in any web browser.
 */
#ifndef REPORT_H
#define REPORT_H

#include <stdio.h>
#include "waveform.h"
#include "events.h"

#define REPORT_INFO_LEN 512

typedef struct {
    char               title[64];
    const waveform_t  *w;
    const event_list_t *ev;
    char               info[REPORT_INFO_LEN];   /* config / measurements */
} report_section_t;

void print_events(FILE *f, const waveform_t *w, const event_list_t *ev);
void print_summary(FILE *f, const event_list_t *ev);

int  report_write_html(const char *path, const char *title,
                       const report_section_t *sec, int n);

#endif /* REPORT_H */
