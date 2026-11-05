#include <string.h>
#include "fuzz.h"
#include "rng.h"
#include "waveform.h"
#include "events.h"
#include "uart.h"
#include "spi.h"
#include "i2c.h"
#include "can.h"

/* ------------------------------------------------- fault injection --- */

static void inject_faults(waveform_t *w, rng_t *r, double unit)
{
    int nfaults = (int)rng_range(r, 1, 3);
    for (int k = 0; k < nfaults && w->len > 10; k++) {
        int ch = (int)rng_range(r, 0, (uint32_t)w->num_ch - 1);
        uint32_t pos = rng_range(r, 0, w->len - 1);
        switch (rng_range(r, 0, 3)) {
        case 0:                        /* short glitch: 1..3 samples */
            wf_invert(w, ch, pos, rng_range(r, 1, 3));
            break;
        case 1: {                      /* long inverted span: up to 2 bit times */
            uint32_t len = rng_range(r, 1, (uint32_t)(2 * unit) + 1);
            wf_invert(w, ch, pos, len);
            break;
        }
        case 2:                        /* random noise: sparse flips */
            for (int j = 0; j < 20; j++)
                wf_invert(w, ch, rng_range(r, 0, w->len - 1), 1);
            break;
        default:                       /* capture stops early */
            wf_truncate(w, pos + 1);
            break;
        }
    }
}

static int check_invariants(const waveform_t *w, const event_list_t *ev)
{
    for (int i = 0; i < ev->count; i++) {
        const event_t *e = &ev->items[i];
        if (e->start > e->end || (w->len > 0 && e->end >= w->len))
            return 0;
    }
    return 1;
}

static void classify(fuzz_stats_t *st, const event_list_t *clean,
                     const event_list_t *faulty)
{
    if (ev_count_errors(faulty) > 0)
        st->detected++;
    else if (!ev_same_payload(clean, faulty))
        st->silent++;
    else
        st->harmless++;
}

/* --------------------------------------------- one iteration each --- */

static void fuzz_uart(rng_t *r, fuzz_stats_t *st, FILE *log, uint32_t seed, int it)
{
    static const uint32_t bauds[] = { 9600, 57600, 115200, 460800, 1000000 };
    uart_cfg_t cfg;
    waveform_t w;
    event_list_t a, b;
    uint8_t msg[32];
    int n = (int)rng_range(r, 1, 32);

    uart_cfg_default(&cfg, bauds[rng_range(r, 0, 4)]);
    cfg.parity    = (parity_t)rng_range(r, 0, 2);
    cfg.stop_bits = (int)rng_range(r, 1, 2);
    cfg.data_bits = (int)rng_range(r, 7, 8);
    uint32_t rate = cfg.baud * rng_range(r, 8, 40);     /* 8..40 samples per bit */

    wf_init(&w, rate, 1, NULL);
    uart_gen_idle(&w, 0, &cfg, 2);
    for (int i = 0; i < n; i++) {
        msg[i] = (uint8_t)(rng_next(r) & ((1u << cfg.data_bits) - 1));
        uart_gen_byte(&w, 0, &cfg, msg[i], 0);
        uart_gen_idle(&w, 0, &cfg, rng_range(r, 0, 3));
    }
    uart_gen_idle(&w, 0, &cfg, 2);

    ev_init(&a);
    ev_init(&b);
    uart_decode(&w, 0, &cfg, &a);
    int ok = (a.count == n) && ev_count_errors(&a) == 0;
    for (int i = 0; ok && i < n; i++)
        ok = (a.items[i].value == msg[i]);
    if (ok) st->clean_ok++;
    else {
        st->clean_fail++;
        if (log) fprintf(log, "UART round-trip FAIL seed=%u it=%d\n", seed, it);
    }

    inject_faults(&w, r, (double)rate / cfg.baud);
    uart_decode(&w, 0, &cfg, &b);
    if (!check_invariants(&w, &b)) st->invariant_fail++;
    classify(st, &a, &b);
    ev_free(&a);
    ev_free(&b);
    wf_free(&w);
}

static void fuzz_spi(rng_t *r, fuzz_stats_t *st, FILE *log, uint32_t seed, int it)
{
    spi_cfg_t cfg;
    waveform_t w;
    event_list_t a, b;
    uint32_t mo[16], mi[16];
    int n = (int)rng_range(r, 1, 16);

    spi_cfg_mode(&cfg, (int)rng_range(r, 0, 3));
    cfg.lsb_first = (int)rng_range(r, 0, 1);
    cfg.bits      = (rng_range(r, 0, 3) == 0) ? 16 : 8;
    double sclk   = 1e6;
    uint32_t rate = (uint32_t)(sclk * rng_range(r, 4, 20));

    for (int i = 0; i < n; i++) {
        mo[i] = rng_next(r) & ((1u << cfg.bits) - 1);
        mi[i] = rng_next(r) & ((1u << cfg.bits) - 1);
    }
    spi_wf_init(&w, rate, &cfg);
    spi_gen_transfer(&w, &cfg, sclk, mo, mi, n);

    ev_init(&a);
    ev_init(&b);
    spi_decode(&w, &cfg, &a);
    int k = 0, ok = (ev_count_errors(&a) == 0);
    for (int i = 0; i < a.count; i++) {
        if (a.items[i].type != EV_DATA) continue;
        if (k >= n || a.items[i].value != mo[k] || a.items[i].value2 != mi[k]) ok = 0;
        k++;
    }
    ok = ok && (k == n);
    if (ok) st->clean_ok++;
    else {
        st->clean_fail++;
        if (log) fprintf(log, "SPI round-trip FAIL seed=%u it=%d\n", seed, it);
    }

    inject_faults(&w, r, (double)rate / sclk);
    spi_decode(&w, &cfg, &b);
    if (!check_invariants(&w, &b)) st->invariant_fail++;
    classify(st, &a, &b);
    ev_free(&a);
    ev_free(&b);
    wf_free(&w);
}

static void fuzz_i2c(rng_t *r, fuzz_stats_t *st, FILE *log, uint32_t seed, int it)
{
    i2c_timing_t t;
    waveform_t w;
    event_list_t a, b;
    uint8_t data[8];
    int n = (int)rng_range(r, 1, 8);
    int fast = (int)rng_range(r, 0, 1);
    double hz = fast ? 400e3 : 100e3;
    uint8_t addr = (uint8_t)rng_range(r, 0x08, 0x77);

    for (int i = 0; i < n; i++)
        data[i] = (uint8_t)rng_next(r);
    i2c_timing_for_freq(&t, hz);
    i2c_wf_init(&w, (uint32_t)(hz * rng_range(r, 20, 100)));
    i2c_gen_write(&w, &t, addr, data, n, -1);

    ev_init(&a);
    ev_init(&b);
    i2c_decode(&w, fast ? I2C_FAST : I2C_STANDARD, &a);
    int k = 0, ok = (ev_count_errors(&a) == 0);
    for (int i = 0; i < a.count; i++) {
        if (a.items[i].type == EV_ADDR && a.items[i].value != addr) ok = 0;
        if (a.items[i].type != EV_DATA) continue;
        if (k >= n || a.items[i].value != data[k]) ok = 0;
        k++;
    }
    ok = ok && (k == n);
    if (ok) st->clean_ok++;
    else {
        st->clean_fail++;
        if (log) fprintf(log, "I2C round-trip FAIL seed=%u it=%d\n", seed, it);
    }

    inject_faults(&w, r, (double)w.sample_rate / hz);
    i2c_decode(&w, fast ? I2C_FAST : I2C_STANDARD, &b);
    if (!check_invariants(&w, &b)) st->invariant_fail++;
    classify(st, &a, &b);
    ev_free(&a);
    ev_free(&b);
    wf_free(&w);
}

static void fuzz_can(rng_t *r, fuzz_stats_t *st, FILE *log, uint32_t seed, int it)
{
    static const uint32_t rates[] = { 125000, 250000, 500000, 1000000 };
    can_frame_t f[4];
    waveform_t w;
    event_list_t a, b;
    int nf = (int)rng_range(r, 1, 4);
    uint32_t br = rates[rng_range(r, 0, 3)];

    can_wf_init(&w, br * rng_range(r, 8, 40));
    can_gen_idle(&w, br, 11);
    for (int i = 0; i < nf; i++) {
        memset(&f[i], 0, sizeof f[i]);
        f[i].id  = (uint16_t)(rng_next(r) & 0x7FF);
        f[i].rtr = (rng_range(r, 0, 9) == 0);
        f[i].dlc = (int)rng_range(r, 0, 8);
        for (int k = 0; k < 8; k++)
            f[i].data[k] = (uint8_t)(rng_range(r, 0, 3) == 0 ? 0x00 : rng_next(r));
        can_gen_frame(&w, br, &f[i], 0);
    }
    can_gen_idle(&w, br, 3);

    ev_init(&a);
    ev_init(&b);
    can_decode(&w, 0, br, &a);
    int ok = (a.count == nf) && ev_count_errors(&a) == 0;
    for (int i = 0; ok && i < nf; i++) {
        ok = (a.items[i].value == f[i].id) && (a.items[i].dlc == f[i].dlc);
        for (int k = 0; ok && !f[i].rtr && k < f[i].dlc; k++)
            ok = (a.items[i].data[k] == f[i].data[k]);
    }
    if (ok) st->clean_ok++;
    else {
        st->clean_fail++;
        if (log) fprintf(log, "CAN round-trip FAIL seed=%u it=%d\n", seed, it);
    }

    inject_faults(&w, r, (double)w.sample_rate / br);
    can_decode(&w, 0, br, &b);
    if (!check_invariants(&w, &b)) st->invariant_fail++;
    classify(st, &a, &b);
    ev_free(&a);
    ev_free(&b);
    wf_free(&w);
}

void fuzz_run(uint32_t seed, int iterations, fuzz_stats_t stats[4], FILE *log)
{
    rng_t r;
    memset(stats, 0, 4 * sizeof stats[0]);
    for (int it = 0; it < iterations; it++) {
        /* each iteration has its own seed -> any failure is reproducible */
        uint32_t s = seed + (uint32_t)it * 7919u;
        rng_seed(&r, s);
        fuzz_uart(&r, &stats[0], log, s, it);
        fuzz_spi (&r, &stats[1], log, s, it);
        fuzz_i2c (&r, &stats[2], log, s, it);
        fuzz_can (&r, &stats[3], log, s, it);
    }
}

void fuzz_print(FILE *f, const fuzz_stats_t stats[4])
{
    static const char *names[4] = { "UART", "SPI", "I2C", "CAN" };
    fprintf(f, "  proto | round-trip ok/fail | faulty: detected  silent  harmless | invariant\n");
    fprintf(f, "  ------+--------------------+-----------------------------------+----------\n");
    for (int p = 0; p < 4; p++) {
        const fuzz_stats_t *s = &stats[p];
        int faulty = s->detected + s->silent + s->harmless;
        if (faulty == 0) faulty = 1;
        fprintf(f, "  %-5s | %8d / %-7d | %7.1f%% %6.1f%% %8.1f%%  | %d\n",
                names[p], s->clean_ok, s->clean_fail,
                100.0 * s->detected / faulty, 100.0 * s->silent / faulty,
                100.0 * s->harmless / faulty, s->invariant_fail);
    }
}
