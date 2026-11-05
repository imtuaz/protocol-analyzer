/*
 * waveform.h - a captured (or simulated) set of digital signals.
 *
 * Think of it as the memory of a logic analyzer:
 *   data[ch][i] = logic level (0 or 1) of channel `ch` at sample `i`.
 * Sample i happens at time  t = i / sample_rate  seconds.
 */
#ifndef WAVEFORM_H
#define WAVEFORM_H

#include <stdint.h>

#define WF_MAX_CH   4
#define WF_NAME_LEN 12

typedef struct {
    uint32_t sample_rate;               /* samples per second            */
    int      num_ch;                    /* number of channels used       */
    char     name[WF_MAX_CH][WF_NAME_LEN];
    uint8_t *data[WF_MAX_CH];           /* one array of levels per ch    */
    uint32_t len;                       /* samples stored                */
    uint32_t cap;                       /* samples allocated             */
    double   t_exact;                   /* exact time cursor (samples),
                                           used by generators to avoid
                                           rounding drift                */
} waveform_t;

/* Create an empty waveform. names may be NULL. Returns 0 on success. */
int  wf_init(waveform_t *w, uint32_t sample_rate, int num_ch, const char *names[]);
void wf_free(waveform_t *w);

/* Append `count` samples where channel c has level level[c]. */
int  wf_hold(waveform_t *w, const uint8_t level[], uint32_t count);

/* Append a duration given in (fractional) samples. The fraction is
 * carried over, so 1000 bits of 86.8 samples end at exactly 86800.     */
int  wf_emit(waveform_t *w, const uint8_t level[], double nsamples);

/* Level of channel ch at sample i (returns last level if i >= len). */
uint8_t wf_get(const waveform_t *w, int ch, uint32_t i);

/* First index > from where channel ch changes level, or w->len. */
uint32_t wf_next_edge(const waveform_t *w, int ch, uint32_t from);

/* Fault injection helpers (used by tests and the fuzzer) */
void wf_invert(waveform_t *w, int ch, uint32_t start, uint32_t count);
void wf_truncate(waveform_t *w, uint32_t new_len);

#endif /* WAVEFORM_H */
