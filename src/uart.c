#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "uart.h"

void uart_cfg_default(uart_cfg_t *cfg, uint32_t baud)
{
    cfg->baud      = baud;
    cfg->data_bits = 8;
    cfg->parity    = PARITY_NONE;
    cfg->stop_bits = 1;
}

/* ===================================================== generator ===== */

/* Drive channel ch to `level` for `bits` bit times; other channels keep
 * their current level. */
static int drive(waveform_t *w, int ch, int level, double bits, uint32_t baud)
{
    uint8_t lv[WF_MAX_CH];
    for (int c = 0; c < w->num_ch; c++)
        lv[c] = wf_get(w, c, w->len ? w->len - 1 : 0);
    if (w->len == 0)
        for (int c = 0; c < w->num_ch; c++)
            lv[c] = 1;
    lv[ch] = (uint8_t)level;
    return wf_emit(w, lv, bits * w->sample_rate / baud);
}

static int parity_bit(uint16_t value, int data_bits, parity_t p)
{
    int ones = 0;
    for (int k = 0; k < data_bits; k++)
        ones += (value >> k) & 1;
    return (p == PARITY_EVEN) ? (ones & 1) : !(ones & 1);
}

int uart_gen_idle(waveform_t *w, int ch, const uart_cfg_t *cfg, double bits)
{
    return drive(w, ch, 1, bits, cfg->baud);
}

int uart_gen_byte(waveform_t *w, int ch, const uart_cfg_t *cfg,
                  uint16_t value, int faults)
{
    int r = drive(w, ch, 0, 1, cfg->baud);                     /* start */
    for (int k = 0; k < cfg->data_bits; k++)
        r |= drive(w, ch, (value >> k) & 1, 1, cfg->baud);     /* LSB first */
    if (cfg->parity != PARITY_NONE) {
        int p = parity_bit(value, cfg->data_bits, cfg->parity);
        if (faults & UART_FAULT_PARITY)
            p = !p;
        r |= drive(w, ch, p, 1, cfg->baud);
    }
    for (int k = 0; k < cfg->stop_bits; k++)
        r |= drive(w, ch, (faults & UART_FAULT_STOP) ? 0 : 1, 1, cfg->baud);
    return r;
}

int uart_gen_break(waveform_t *w, int ch, const uart_cfg_t *cfg, double bits)
{
    int r = drive(w, ch, 0, bits, cfg->baud);
    r |= drive(w, ch, 1, 2, cfg->baud);          /* back to idle */
    return r;
}

/* ======================================================= decoder ===== */

/* Read the line at (fractional) sample position pos.
 * If a bit is wide enough, use a 3-sample majority vote around the centre
 * to reject single-sample noise (like real UART hardware does). */
static int sample_bit(const waveform_t *w, int ch, double pos, double spb)
{
    uint32_t c = (uint32_t)(pos + 0.5);
    if (spb >= 6.0 && c >= 1) {
        int sum = wf_get(w, ch, c - 1) + wf_get(w, ch, c) + wf_get(w, ch, c + 1);
        return sum >= 2;
    }
    return wf_get(w, ch, c);
}

void uart_decode(const waveform_t *w, int ch, const uart_cfg_t *cfg,
                 event_list_t *out)
{
    const uint8_t *d = w->data[ch];
    double spb = (double)w->sample_rate / cfg->baud;     /* samples per bit */
    int    has_par = (cfg->parity != PARITY_NONE);
    int    frame_bits = 1 + cfg->data_bits + has_par + cfg->stop_bits;
    uint32_t i = 1;

    if (w->len < 2)
        return;
    if (spb < 3.0) {                    /* cannot find bit centres reliably */
        event_t *e = ev_add(out, PROTO_UART, EV_ERROR, 0, w->len - 1);
        if (e) {
            e->errors = ERR_TIMING;
            snprintf(e->note, EV_NOTE_LEN, "sample rate too low: %.1f samples/bit (need >= 3)", spb);
        }
        return;
    }

    for (;;) {
        /* 1. Find a falling edge = possible start bit */
        while (i < w->len && !(d[i - 1] == 1 && d[i] == 0))
            i++;
        if (i >= w->len)
            break;
        double s = i;

        /* Frame does not fit in the capture -> incomplete */
        if (s + spb * (frame_bits - 0.5) >= w->len) {
            event_t *e = ev_add(out, PROTO_UART, EV_ERROR, i, w->len - 1);
            if (e) {
                e->errors = ERR_INCOMPLETE;
                snprintf(e->note, EV_NOTE_LEN, "frame cut at end of capture");
            }
            break;
        }

        /* 2. Start bit must still be low at its centre, else it was noise */
        if (sample_bit(w, ch, s + 0.5 * spb, spb) != 0) {
            event_t *e = ev_add(out, PROTO_UART, EV_ERROR, i,
                                (uint32_t)(s + 0.5 * spb));
            if (e) {
                e->errors = ERR_GLITCH;
                snprintf(e->note, EV_NOTE_LEN, "short low pulse, not a start bit");
            }
            i = (uint32_t)(s + 0.5 * spb) + 1;
            continue;
        }

        /* 3. Data bits, sampled at the centre of each bit, LSB first */
        uint32_t value = 0;
        for (int k = 0; k < cfg->data_bits; k++)
            value |= (uint32_t)sample_bit(w, ch, s + spb * (1.5 + k), spb) << k;

        uint32_t errors = 0;
        int par_level = 0;
        double pos = s + spb * (1.5 + cfg->data_bits);

        /* 4. Parity */
        if (has_par) {
            par_level = sample_bit(w, ch, pos, spb);
            if (par_level != parity_bit((uint16_t)value, cfg->data_bits, cfg->parity))
                errors |= ERR_PARITY;
            pos += spb;
        }

        /* 5. Stop bit(s) must be high */
        for (int k = 0; k < cfg->stop_bits; k++) {
            if (sample_bit(w, ch, pos, spb) != 1)
                errors |= ERR_FRAMING;
            pos += spb;
        }

        uint32_t end = (uint32_t)(s + spb * frame_bits);
        if (end >= w->len)
            end = w->len - 1;

        /* 6. All zeros incl. stop bit = BREAK condition, not a data byte */
        if (value == 0 && par_level == 0 && (errors & ERR_FRAMING)) {
            uint32_t rise = i;
            while (rise < w->len && d[rise] == 0)
                rise++;
            event_t *e = ev_add(out, PROTO_UART, EV_BREAK, i, rise ? rise - 1 : 0);
            if (e)
                snprintf(e->note, EV_NOTE_LEN, "line low for %.1f bit times",
                         (rise - s) / spb);
            i = rise + 1;
            continue;
        }

        event_t *e = ev_add(out, PROTO_UART, EV_DATA, i, end);
        if (e) {
            e->value  = value;
            e->errors = errors;
            if (value >= 0x20 && value < 0x7F)
                snprintf(e->note, EV_NOTE_LEN, "'%c'", (char)value);
        }

        /* 7. Continue searching after the centre of the last stop bit */
        i = (uint32_t)(s + spb * (frame_bits - 0.5)) + 1;
    }
}

/* ============================================ baud rate detection ===== */

static int cmp_u32(const void *a, const void *b)
{
    uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
    return (x > y) - (x < y);
}

static const uint32_t k_std_baud[] = {
    1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200,
    230400, 460800, 921600, 1000000, 2000000
};

int uart_detect_baud(const waveform_t *w, int ch, baud_result_t *res)
{
    res->ok = 0;
    res->measured_baud = 0;
    res->standard_baud = 0;
    res->error_pct = 0;

    /* 1. Collect the width of every pulse between two edges.
     *    First and last runs are idle time, not bits -> skipped. */
    uint32_t cap = 1024, n = 0;
    uint32_t *wid = malloc(cap * sizeof *wid);
    if (!wid)
        return -1;

    uint32_t e0 = wf_next_edge(w, ch, 0);
    while (e0 < w->len) {
        uint32_t e1 = wf_next_edge(w, ch, e0);
        if (e1 >= w->len)
            break;
        if (n == cap) {
            cap *= 2;
            uint32_t *t = realloc(wid, cap * sizeof *wid);
            if (!t) {
                free(wid);
                return -1;
            }
            wid = t;
        }
        wid[n++] = e1 - e0;
        e0 = e1;
    }
    if (n < 3) {
        free(wid);
        return -1;
    }
    qsort(wid, n, sizeof *wid, cmp_u32);

    /* 2. Bit time = the shortest width that appears repeatedly.
     *    A single short glitch is ignored because it is not repeated. */
    uint32_t need = (n / 50 > 3) ? n / 50 : 3;
    double t0 = 0;
    for (uint32_t a = 0; a < n && t0 == 0; a++) {
        if (wid[a] < 2)
            continue;
        uint32_t cnt = 0;
        for (uint32_t b = a; b < n && wid[b] <= wid[a] * 1.15; b++)
            cnt++;
        if (cnt >= need)
            t0 = wid[a];
    }
    if (t0 == 0) {
        free(wid);
        return -1;
    }

    /* 3. Refine: every pulse is (almost) a whole number of bits.
     *    Bit time = total width / total number of bits. */
    double sum_w = 0, sum_n = 0;
    for (uint32_t a = 0; a < n; a++) {
        double ratio = wid[a] / t0;
        double k = floor(ratio + 0.5);
        if (k >= 1 && k <= 12 && fabs(ratio - k) < 0.3) {
            sum_w += wid[a];
            sum_n += k;
        }
    }
    free(wid);
    if (sum_n == 0)
        return -1;

    double bit_time = sum_w / sum_n;
    res->measured_baud = w->sample_rate / bit_time;

    /* 4. Snap to nearest standard baud if within 5 % */
    double best = 1e9;
    for (size_t k = 0; k < sizeof k_std_baud / sizeof k_std_baud[0]; k++) {
        double err = fabs(res->measured_baud - k_std_baud[k]) / k_std_baud[k];
        if (err < best) {
            best = err;
            res->standard_baud = k_std_baud[k];
        }
    }
    if (best > 0.05)
        res->standard_baud = (uint32_t)(res->measured_baud + 0.5);

    res->error_pct = (res->measured_baud - res->standard_baud)
                     / res->standard_baud * 100.0;
    res->ok = 1;
    return 0;
}

void uart_check_timing(const waveform_t *w, int ch, const uart_cfg_t *cfg,
                       event_list_t *out)
{
    baud_result_t br;
    if (uart_detect_baud(w, ch, &br) != 0)
        return;
    double dev = (br.measured_baud - cfg->baud) / cfg->baud * 100.0;
    if (fabs(dev) > UART_BAUD_TOLERANCE_PCT) {
        event_t *e = ev_add(out, PROTO_UART, EV_TIMING, 0, w->len ? w->len - 1 : 0);
        if (e) {
            e->errors = ERR_TIMING;
            snprintf(e->note, EV_NOTE_LEN, "baud %.0f vs %u expected (%+.2f%%)",
                     br.measured_baud, cfg->baud, dev);
        }
    }
}
