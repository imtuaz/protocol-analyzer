#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "waveform.h"

int wf_init(waveform_t *w, uint32_t sample_rate, int num_ch, const char *names[])
{
    memset(w, 0, sizeof *w);
    if (num_ch < 1 || num_ch > WF_MAX_CH || sample_rate == 0)
        return -1;
    w->sample_rate = sample_rate;
    w->num_ch      = num_ch;
    for (int c = 0; c < num_ch; c++) {
        if (names && names[c])
            snprintf(w->name[c], WF_NAME_LEN, "%s", names[c]);
        else
            snprintf(w->name[c], WF_NAME_LEN, "CH%u", (unsigned)c & 0xFu);
    }
    return 0;
}

void wf_free(waveform_t *w)
{
    for (int c = 0; c < WF_MAX_CH; c++) {
        free(w->data[c]);
        w->data[c] = NULL;
    }
    w->len = w->cap = 0;
}

/* Make room for at least `need` samples (capacity doubles each time) */
static int wf_reserve(waveform_t *w, uint32_t need)
{
    if (need <= w->cap)
        return 0;
    uint32_t new_cap = w->cap ? w->cap : 1024;
    while (new_cap < need)
        new_cap *= 2;
    for (int c = 0; c < w->num_ch; c++) {
        uint8_t *p = realloc(w->data[c], new_cap);
        if (!p)
            return -1;
        w->data[c] = p;
    }
    w->cap = new_cap;
    return 0;
}

/* Append without touching t_exact */
static int wf_append(waveform_t *w, const uint8_t level[], uint32_t count)
{
    if (count == 0)
        return 0;
    if (wf_reserve(w, w->len + count) != 0)
        return -1;
    for (int c = 0; c < w->num_ch; c++)
        memset(w->data[c] + w->len, level[c] ? 1 : 0, count);
    w->len += count;
    return 0;
}

int wf_hold(waveform_t *w, const uint8_t level[], uint32_t count)
{
    int r = wf_append(w, level, count);
    w->t_exact = w->len;           /* integer hold: cursor = end */
    return r;
}

int wf_emit(waveform_t *w, const uint8_t level[], double nsamples)
{
    w->t_exact += nsamples;
    uint32_t target = (uint32_t)(w->t_exact + 0.5);   /* round */
    if (target > w->len)
        return wf_append(w, level, target - w->len);
    return 0;
}

uint8_t wf_get(const waveform_t *w, int ch, uint32_t i)
{
    if (w->len == 0)
        return 1;
    if (i >= w->len)
        i = w->len - 1;
    return w->data[ch][i];
}

uint32_t wf_next_edge(const waveform_t *w, int ch, uint32_t from)
{
    if (from >= w->len)
        return w->len;
    uint8_t v = w->data[ch][from];
    for (uint32_t i = from + 1; i < w->len; i++)
        if (w->data[ch][i] != v)
            return i;
    return w->len;
}

void wf_invert(waveform_t *w, int ch, uint32_t start, uint32_t count)
{
    for (uint32_t i = start; i < start + count && i < w->len; i++)
        w->data[ch][i] ^= 1;
}

void wf_truncate(waveform_t *w, uint32_t new_len)
{
    if (new_len < w->len)
        w->len = new_len;
    w->t_exact = w->len;
}
