/*
 * test_main.c - unit tests. Run with:  make test
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "waveform.h"
#include "events.h"
#include "uart.h"
#include "spi.h"
#include "i2c.h"
#include "can.h"
#include "wave_io.h"

static int g_pass, g_fail;

#define CHECK(cond) do {                                              \
        if (cond) g_pass++;                                           \
        else { g_fail++;                                              \
               printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
    } while (0)

/* Collect only events of one type into an array of values */
static int values_of(const event_list_t *l, ev_type_t t, uint32_t *vals, int max)
{
    int n = 0;
    for (int i = 0; i < l->count && n < max; i++)
        if (l->items[i].type == t)
            vals[n++] = l->items[i].value;
    return n;
}

/* ================================================================ UART */

static void uart_make(waveform_t *w, const uart_cfg_t *cfg, uint32_t rate,
                      const uint8_t *bytes, int n, const int *faults)
{
    wf_init(w, rate, 1, NULL);
    uart_gen_idle(w, 0, cfg, 3);
    for (int i = 0; i < n; i++) {
        uart_gen_byte(w, 0, cfg, bytes[i], faults ? faults[i] : 0);
        uart_gen_idle(w, 0, cfg, 1);
    }
    uart_gen_idle(w, 0, cfg, 3);
}

static void test_uart_roundtrip_all_formats(void)
{
    const uint8_t msg[] = { 'H', 'i', 0x00, 0xFF, 0x55, 0xA5, 0x80, 0x01 };
    const parity_t pars[] = { PARITY_NONE, PARITY_EVEN, PARITY_ODD };
    const uint32_t bauds[] = { 9600, 115200, 1000000 };

    for (int b = 0; b < 3; b++)
        for (int p = 0; p < 3; p++)
            for (int sb = 1; sb <= 2; sb++) {
                uart_cfg_t cfg;
                waveform_t w;
                event_list_t ev;
                uint32_t got[16];

                uart_cfg_default(&cfg, bauds[b]);
                cfg.parity = pars[p];
                cfg.stop_bits = sb;
                uart_make(&w, &cfg, 10000000, msg, 8, NULL);
                ev_init(&ev);
                uart_decode(&w, 0, &cfg, &ev);

                int n = values_of(&ev, EV_DATA, got, 16);
                int ok = (n == 8) && ev_count_errors(&ev) == 0;
                for (int i = 0; ok && i < 8; i++)
                    ok = (got[i] == msg[i]);
                CHECK(ok);
                if (!ok)
                    printf("    baud=%u parity=%d stop=%d n=%d\n",
                           bauds[b], p, sb, n);
                ev_free(&ev);
                wf_free(&w);
            }
}

static void test_uart_parity_and_framing_errors(void)
{
    const uint8_t msg[] = { 0x31, 0x32, 0x33 };
    const int faults[] = { 0, UART_FAULT_PARITY, UART_FAULT_STOP };
    uart_cfg_t cfg;
    waveform_t w;
    event_list_t ev;

    uart_cfg_default(&cfg, 115200);
    cfg.parity = PARITY_EVEN;
    uart_make(&w, &cfg, 10000000, msg, 3, faults);
    ev_init(&ev);
    uart_decode(&w, 0, &cfg, &ev);

    CHECK(ev.count == 3);
    CHECK(ev.items[0].errors == 0);
    CHECK(ev.items[1].errors == ERR_PARITY);
    CHECK(ev.items[1].value == 0x32);          /* data still recovered */
    CHECK(ev.items[2].errors & ERR_FRAMING);
    ev_free(&ev);
    wf_free(&w);
}

static void test_uart_break(void)
{
    uart_cfg_t cfg;
    waveform_t w;
    event_list_t ev;

    uart_cfg_default(&cfg, 9600);
    wf_init(&w, 1000000, 1, NULL);
    uart_gen_idle(&w, 0, &cfg, 2);
    uart_gen_byte(&w, 0, &cfg, 'A', 0);
    uart_gen_break(&w, 0, &cfg, 25);
    uart_gen_byte(&w, 0, &cfg, 'B', 0);
    uart_gen_idle(&w, 0, &cfg, 2);

    ev_init(&ev);
    uart_decode(&w, 0, &cfg, &ev);
    CHECK(ev.count == 3);
    CHECK(ev.items[0].type == EV_DATA && ev.items[0].value == 'A');
    CHECK(ev.items[1].type == EV_BREAK);
    CHECK(ev.items[2].type == EV_DATA && ev.items[2].value == 'B');
    ev_free(&ev);
    wf_free(&w);
}

static void test_uart_glitch_rejected(void)
{
    const uint8_t msg[] = { 'X' };
    uart_cfg_t cfg;
    waveform_t w;
    event_list_t ev;

    uart_cfg_default(&cfg, 115200);
    uart_make(&w, &cfg, 10000000, msg, 1, NULL);
    wf_invert(&w, 0, 50, 4);                  /* 0.4 us spike in idle  */
    ev_init(&ev);
    uart_decode(&w, 0, &cfg, &ev);
    CHECK(ev_count_type(&ev, EV_DATA) == 1);
    CHECK(ev.items[0].errors == ERR_GLITCH);
    CHECK(ev.items[ev.count - 1].value == 'X');
    ev_free(&ev);
    wf_free(&w);
}

static void test_uart_majority_vote_noise(void)
{
    const uint8_t msg[] = { 0x5A };
    uart_cfg_t cfg;
    waveform_t w;
    event_list_t ev;

    uart_cfg_default(&cfg, 115200);           /* 86.8 samples per bit */
    uart_make(&w, &cfg, 10000000, msg, 1, NULL);
    /* flip exactly the centre sample of data bit 0 */
    uint32_t start = wf_next_edge(&w, 0, 0);
    wf_invert(&w, 0, (uint32_t)(start + 86.8 * 1.5 + 0.5), 1);
    ev_init(&ev);
    uart_decode(&w, 0, &cfg, &ev);
    CHECK(ev.count == 1 && ev.items[0].value == 0x5A && ev.items[0].errors == 0);
    ev_free(&ev);
    wf_free(&w);
}

static void test_uart_autobaud(void)
{
    const uint32_t bauds[] = { 9600, 38400, 115200, 921600 };
    const uint8_t msg[] = "Hello, UART!";
    for (int b = 0; b < 4; b++) {
        uart_cfg_t cfg;
        waveform_t w;
        baud_result_t br;
        uart_cfg_default(&cfg, bauds[b]);
        uart_make(&w, &cfg, 20000000, msg, 12, NULL);
        CHECK(uart_detect_baud(&w, 0, &br) == 0);
        CHECK(br.standard_baud == bauds[b]);
        CHECK(fabs(br.error_pct) < 0.5);
        wf_free(&w);
    }
}

static void test_uart_baud_mismatch_timing(void)
{
    const uint8_t msg[] = "UUUU";
    uart_cfg_t tx, rx;
    waveform_t w;
    event_list_t ev;
    baud_result_t br;

    uart_cfg_default(&tx, 118000);            /* +2.4 % fast transmitter */
    uart_cfg_default(&rx, 115200);
    uart_make(&w, &tx, 10000000, msg, 4, NULL);
    uart_detect_baud(&w, 0, &br);
    CHECK(br.standard_baud == 115200);
    CHECK(br.error_pct > 2.0 && br.error_pct < 2.8);

    ev_init(&ev);
    uart_check_timing(&w, 0, &rx, &ev);
    CHECK(ev.count == 1 && ev.items[0].errors == ERR_TIMING);
    ev_free(&ev);
    wf_free(&w);
}

static void test_uart_truncated(void)
{
    const uint8_t msg[] = { 'A', 'B' };
    uart_cfg_t cfg;
    waveform_t w;
    event_list_t ev;

    uart_cfg_default(&cfg, 115200);
    uart_make(&w, &cfg, 10000000, msg, 2, NULL);
    wf_truncate(&w, w.len - 3 * 87 - 87 * 5);   /* cut inside 'B' */
    ev_init(&ev);
    uart_decode(&w, 0, &cfg, &ev);
    CHECK(ev.count == 2);
    CHECK(ev.items[0].value == 'A');
    CHECK(ev.items[1].errors == ERR_INCOMPLETE);
    ev_free(&ev);
    wf_free(&w);
}


/* ================================================================= SPI */

static void test_spi_roundtrip_all_modes(void)
{
    const uint32_t mo[] = { 0x9F, 0x00, 0xFF, 0xA5, 0x01, 0x80 };
    const uint32_t mi[] = { 0x00, 0xEF, 0x40, 0x18, 0x7E, 0x55 };
    for (int mode = 0; mode < 4; mode++)
        for (int lsb = 0; lsb <= 1; lsb++) {
            spi_cfg_t cfg;
            waveform_t w;
            event_list_t ev;
            spi_cfg_mode(&cfg, mode);
            cfg.lsb_first = lsb;
            spi_wf_init(&w, 20000000, &cfg);
            spi_gen_transfer(&w, &cfg, 1e6, mo, mi, 6);
            ev_init(&ev);
            spi_decode(&w, &cfg, &ev);

            int n = 0, ok = 1;
            for (int i = 0; i < ev.count; i++) {
                const event_t *e = &ev.items[i];
                if (e->type != EV_DATA)
                    continue;
                if (n >= 6 || e->value != mo[n] || e->value2 != mi[n] || e->errors)
                    ok = 0;
                n++;
            }
            CHECK(ok && n == 6 && ev_count_errors(&ev) == 0);
            if (!(ok && n == 6))
                printf("    mode=%d lsb=%d n=%d\n", mode, lsb, n);
            ev_free(&ev);
            wf_free(&w);
        }
}

static void test_spi_16bit_words(void)
{
    const uint32_t mo[] = { 0xBEEF, 0x1234 };
    spi_cfg_t cfg;
    waveform_t w;
    event_list_t ev;
    uint32_t got[4];

    spi_cfg_mode(&cfg, 1);
    cfg.bits = 16;
    spi_wf_init(&w, 10000000, &cfg);
    spi_gen_transfer(&w, &cfg, 500e3, mo, NULL, 2);
    ev_init(&ev);
    spi_decode(&w, &cfg, &ev);
    CHECK(values_of(&ev, EV_DATA, got, 4) == 2);
    CHECK(got[0] == 0xBEEF && got[1] == 0x1234);
    ev_free(&ev);
    wf_free(&w);
}

static void test_spi_wrong_mode_detected(void)
{
    const uint32_t mo[] = { 0xA5, 0x3C };
    spi_cfg_t tx, rx;
    waveform_t w;
    event_list_t ev;

    spi_cfg_mode(&tx, 0);
    spi_cfg_mode(&rx, 1);                      /* decoder set to wrong CPHA */
    spi_wf_init(&w, 10000000, &tx);
    spi_gen_transfer(&w, &tx, 1e6, mo, NULL, 2);
    ev_init(&ev);
    spi_decode(&w, &rx, &ev);
    int timing = 0;
    for (int i = 0; i < ev.count; i++)
        if (ev.items[i].type == EV_DATA && (ev.items[i].errors & ERR_TIMING))
            timing++;
    CHECK(timing >= 1);
    ev_free(&ev);

    spi_cfg_mode(&rx, 2);                      /* wrong CPOL */
    ev_init(&ev);
    spi_decode(&w, &rx, &ev);
    CHECK(ev_count_errors(&ev) >= 1);
    ev_free(&ev);
    wf_free(&w);
}

static void test_spi_detect_mode(void)
{
    const uint32_t mo[] = { 0x12, 0xA5, 0x3C, 0xF0 };
    for (int mode = 0; mode < 4; mode++) {
        spi_cfg_t cfg;
        waveform_t w;
        spi_cfg_mode(&cfg, mode);
        spi_wf_init(&w, 10000000, &cfg);
        spi_gen_transfer(&w, &cfg, 1e6, mo, NULL, 4);
        CHECK(spi_detect_mode(&w) == mode);
        wf_free(&w);
    }
}

static void test_spi_incomplete(void)
{
    const uint32_t mo[] = { 0x11, 0x22 };
    spi_cfg_t cfg;
    waveform_t w;
    event_list_t ev;
    uint32_t got[4];

    spi_cfg_mode(&cfg, 0);
    spi_wf_init(&w, 10000000, &cfg);
    spi_gen_transfer_cut(&w, &cfg, 1e6, mo, 2, 5);   /* 2nd word: 5 bits */
    ev_init(&ev);
    spi_decode(&w, &cfg, &ev);
    CHECK(values_of(&ev, EV_DATA, got, 4) == 1 && got[0] == 0x11);
    CHECK(ev_count_type(&ev, EV_ERROR) == 1);
    int found = 0;
    for (int i = 0; i < ev.count; i++)
        if (ev.items[i].errors == ERR_INCOMPLETE)
            found = 1;
    CHECK(found);
    ev_free(&ev);
    wf_free(&w);
}

/* ================================================================= I2C */

static int has_timing_violation(const event_list_t *l, const char *name)
{
    for (int i = 0; i < l->count; i++)
        if (l->items[i].type == EV_TIMING && (l->items[i].errors & ERR_TIMING)
            && strncmp(l->items[i].note, name, strlen(name)) == 0)
            return 1;
    return 0;
}

static void test_i2c_write(void)
{
    const uint8_t data[] = { 0x10, 0xAB, 0x00, 0xFF };
    const double freqs[] = { 100e3, 400e3 };
    const i2c_speed_t sp[] = { I2C_STANDARD, I2C_FAST };
    for (int f = 0; f < 2; f++) {
        i2c_timing_t t;
        waveform_t w;
        event_list_t ev;
        i2c_timing_for_freq(&t, freqs[f]);
        i2c_wf_init(&w, 20000000);
        i2c_gen_write(&w, &t, 0x50, data, 4, -1);
        ev_init(&ev);
        i2c_decode(&w, sp[f], &ev);

        CHECK(ev.items[0].type == EV_START);
        CHECK(ev.items[1].type == EV_ADDR && ev.items[1].value == 0x50
              && ev.items[1].rw == 0 && ev.items[1].ack == 1);
        int ok = 1;
        for (int i = 0; i < 4; i++)
            ok &= (ev.items[2 + i].type == EV_DATA && ev.items[2 + i].value == data[i]
                   && ev.items[2 + i].ack == 1);
        CHECK(ok);
        CHECK(ev.items[6].type == EV_STOP);
        CHECK(ev_count_errors(&ev) == 0);          /* compliant timing */
        ev_free(&ev);
        wf_free(&w);
    }
}

static void test_i2c_read_register_with_repeated_start(void)
{
    const uint8_t data[] = { 0x12, 0x34, 0x56 };
    i2c_timing_t t;
    waveform_t w;
    event_list_t ev;
    i2c_timing_for_freq(&t, 400e3);
    i2c_wf_init(&w, 20000000);
    i2c_gen_read_reg(&w, &t, 0x68, 0x3B, data, 3);
    ev_init(&ev);
    i2c_decode(&w, I2C_FAST, &ev);

    /* S, ADDR W, 0x3B, Sr, ADDR R, 12, 34, 56(NACK), P, SCL info */
    CHECK(ev.items[3].type == EV_START && strcmp(ev.items[3].note, "Sr") == 0);
    CHECK(ev.items[4].type == EV_ADDR && ev.items[4].rw == 1 && ev.items[4].value == 0x68);
    CHECK(ev.items[2].value == 0x3B);
    CHECK(ev.items[7].value == 0x56 && ev.items[7].ack == 0);
    CHECK(ev.items[7].errors == 0);          /* NACK at end of read is normal */
    CHECK(ev.items[8].type == EV_STOP);
    CHECK(ev_count_errors(&ev) == 0);
    ev_free(&ev);
    wf_free(&w);
}

static void test_i2c_nack(void)
{
    const uint8_t data[] = { 0x01, 0x02, 0x03 };
    i2c_timing_t t;
    waveform_t w;
    event_list_t ev;
    i2c_timing_for_freq(&t, 100e3);
    i2c_wf_init(&w, 10000000);
    i2c_gen_write(&w, &t, 0x3C, data, 3, 0);   /* address NACK */
    i2c_gen_write(&w, &t, 0x3C, data, 3, 2);   /* 2nd data byte NACK */
    ev_init(&ev);
    i2c_decode(&w, I2C_STANDARD, &ev);
    int addr_nack = 0, data_nack = 0;
    for (int i = 0; i < ev.count; i++) {
        if (ev.items[i].type == EV_ADDR && (ev.items[i].errors & ERR_NACK))
            addr_nack++;
        if (ev.items[i].type == EV_DATA && (ev.items[i].errors & ERR_NACK)
            && ev.items[i].value == 0x02)
            data_nack++;
    }
    CHECK(addr_nack == 1);
    CHECK(data_nack == 1);
    CHECK(ev_count_type(&ev, EV_ERROR) == 0);
    ev_free(&ev);
    wf_free(&w);
}

static void test_i2c_timing_violations(void)
{
    const uint8_t data[] = { 0xAA };
    i2c_timing_t t;
    waveform_t w;
    event_list_t ev;

    /* 400 kHz traffic checked against Standard-mode spec -> violations */
    i2c_timing_for_freq(&t, 400e3);
    i2c_wf_init(&w, 20000000);
    i2c_gen_write(&w, &t, 0x20, data, 1, -1);
    ev_init(&ev);
    i2c_decode(&w, I2C_STANDARD, &ev);
    CHECK(has_timing_violation(&ev, "tLOW"));
    CHECK(has_timing_violation(&ev, "tHIGH"));
    ev_free(&ev);
    wf_free(&w);

    /* Fast mode with too short SCL high (0.4 us < 0.6 us) */
    i2c_timing_for_freq(&t, 400e3);
    t.t_high = 0.4e-6;
    i2c_wf_init(&w, 20000000);
    i2c_gen_write(&w, &t, 0x20, data, 1, -1);
    ev_init(&ev);
    i2c_decode(&w, I2C_FAST, &ev);
    CHECK(has_timing_violation(&ev, "tHIGH"));
    CHECK(!has_timing_violation(&ev, "tLOW"));
    ev_free(&ev);
    wf_free(&w);

    /* Data setup too short (50 ns < 100 ns) */
    i2c_timing_for_freq(&t, 400e3);
    t.t_su_dat = 50e-9;
    i2c_wf_init(&w, 50000000);
    i2c_gen_write(&w, &t, 0x20, data, 1, -1);
    ev_init(&ev);
    i2c_decode(&w, I2C_FAST, &ev);
    CHECK(has_timing_violation(&ev, "tSU;DAT"));
    ev_free(&ev);
    wf_free(&w);
}

static void test_i2c_interrupted_byte(void)
{
    i2c_timing_t t;
    waveform_t w;
    event_list_t ev;
    i2c_timing_for_freq(&t, 100e3);
    i2c_wf_init(&w, 10000000);
    i2c_gen_start(&w, &t);
    i2c_gen_byte(&w, &t, 0xA0, 1);
    /* send only 4 bits of the next byte by cutting the capture */
    uint32_t cut = w.len + (uint32_t)(4 * 10e-6 * 10000000) + 100;
    i2c_gen_byte(&w, &t, 0x55, 1);
    wf_truncate(&w, cut);
    ev_init(&ev);
    i2c_decode(&w, I2C_STANDARD, &ev);
    int found = 0;
    for (int i = 0; i < ev.count; i++)
        if (ev.items[i].errors & ERR_INCOMPLETE)
            found = 1;
    CHECK(found);
    ev_free(&ev);
    wf_free(&w);
}

/* ================================================================= CAN */

static void test_can_crc15_reference(void)
{
    /* CRC catalogue check value: CRC-15/CAN("123456789") = 0x059E */
    const char *s = "123456789";
    uint8_t bits[72];
    int n = 0;
    for (int i = 0; i < 9; i++)
        for (int k = 7; k >= 0; k--)
            bits[n++] = (uint8_t)((s[i] >> k) & 1);
    CHECK(can_crc15(bits, n) == 0x059E);
}

static void can_make(waveform_t *w, const can_frame_t *f, int nf, const int *faults)
{
    can_wf_init(w, 10000000);
    can_gen_idle(w, 500000, 11);
    for (int i = 0; i < nf; i++)
        can_gen_frame(w, 500000, &f[i], faults ? faults[i] : 0);
    can_gen_idle(w, 500000, 5);
}

static void test_can_roundtrip(void)
{
    can_frame_t f[4];
    memset(f, 0, sizeof f);
    f[0].id = 0x123; f[0].dlc = 2; f[0].data[0] = 0x11; f[0].data[1] = 0x22;
    f[1].id = 0x7FF; f[1].dlc = 8;                 /* long runs of 1 -> stuffing */
    memset(f[1].data, 0xFF, 8);
    f[2].id = 0x000; f[2].dlc = 8;                 /* long runs of 0 -> stuffing */
    f[3].id = 0x2A5; f[3].rtr = 1; f[3].dlc = 4;   /* remote frame */

    waveform_t w;
    event_list_t ev;
    can_make(&w, f, 4, NULL);
    ev_init(&ev);
    can_decode(&w, 0, 500000, &ev);

    CHECK(ev.count == 4);
    CHECK(ev_count_errors(&ev) == 0);
    for (int i = 0; i < 4 && i < ev.count; i++) {
        const event_t *e = &ev.items[i];
        int ok = (e->value == f[i].id) && (e->dlc == f[i].dlc) && e->ack == 1;
        for (int k = 0; k < 8 && !f[i].rtr && k < f[i].dlc; k++)
            ok &= (e->data[k] == f[i].data[k]);
        CHECK(ok);
        if (!ok)
            printf("    frame %d: %s\n", i, e->note);
    }
    ev_free(&ev);
    wf_free(&w);
}

static void test_can_errors(void)
{
    can_frame_t f[5];
    const int faults[5] = { 0, CAN_FAULT_FLIP_BIT, CAN_FAULT_NO_STUFF,
                            CAN_FAULT_NO_ACK, CAN_FAULT_BAD_DELIM };
    memset(f, 0, sizeof f);
    for (int i = 0; i < 5; i++) {
        f[i].id = (uint16_t)(0x100 + i);
        f[i].dlc = 3;
        f[i].data[0] = 0x00; f[i].data[1] = 0x00; f[i].data[2] = 0xA5;  /* forces stuffing */
    }
    waveform_t w;
    event_list_t ev;
    can_make(&w, f, 5, faults);
    ev_init(&ev);
    can_decode(&w, 0, 500000, &ev);

    CHECK(ev.count == 5);
    if (ev.count == 5) {
        CHECK(ev.items[0].errors == 0);
        CHECK(ev.items[1].errors & ERR_CRC);
        CHECK(ev.items[2].errors & ERR_STUFF);
        CHECK(ev.items[3].errors == ERR_NACK);
        CHECK(ev.items[4].errors & ERR_FORM);
    } else {
        for (int i = 0; i < ev.count; i++)
            printf("    %d: %s err=%x\n", i, ev.items[i].note, ev.items[i].errors);
    }
    ev_free(&ev);
    wf_free(&w);
}

static void test_can_clock_mismatch(void)
{
    /* Transmitter 1 % fast: hard synchronisation on edges keeps decoding OK */
    can_frame_t f;
    waveform_t w;
    event_list_t ev;
    memset(&f, 0, sizeof f);
    f.id = 0x555; f.dlc = 8;
    for (int k = 0; k < 8; k++)
        f.data[k] = (uint8_t)(0x30 + k);
    can_wf_init(&w, 10000000);
    can_gen_idle(&w, 505000, 11);
    can_gen_frame(&w, 505000, &f, 0);
    ev_init(&ev);
    can_decode(&w, 0, 500000, &ev);
    CHECK(ev.count == 1 && ev.items[0].errors == 0 && ev.items[0].data[7] == 0x37);
    ev_free(&ev);
    wf_free(&w);
}

/* ============================================================= CSV I/O */

static void test_csv_roundtrip(void)
{
    can_frame_t f;
    waveform_t w, r1, r2;
    event_list_t a, b, c;
    memset(&f, 0, sizeof f);
    f.id = 0x321; f.dlc = 3; f.data[0] = 1; f.data[1] = 2; f.data[2] = 3;
    can_wf_init(&w, 8000000);
    can_gen_idle(&w, 250000, 11);
    can_gen_frame(&w, 250000, &f, 0);
    can_gen_idle(&w, 250000, 3);

    CHECK(wf_save_csv(&w, "build/t.csv") == 0);
    CHECK(wf_load_csv(&r1, "build/t.csv", 0) == 0);          /* uses header */
    CHECK(r1.sample_rate == 8000000 && r1.len == w.len);
    int same = 1;
    for (uint32_t i = 0; i < w.len && i < r1.len; i++)
        same &= (w.data[0][i] == r1.data[0][i]);
    CHECK(same);                                             /* bit-exact */

    CHECK(wf_load_csv(&r2, "build/t.csv", 1000000) == 0);    /* forced rate */
    CHECK(r2.sample_rate == 1000000);

    ev_init(&a); ev_init(&b); ev_init(&c);
    can_decode(&w, 0, 250000, &a);
    can_decode(&r1, 0, 250000, &b);
    can_decode(&r2, 0, 250000, &c);
    CHECK(ev_same_payload(&a, &b));
    CHECK(ev_same_payload(&a, &c));          /* 4 samples/bit still decodes */
    CHECK(wf_save_vcd(&w, "build/t.vcd") == 0);
    ev_free(&a); ev_free(&b); ev_free(&c);
    wf_free(&w); wf_free(&r1); wf_free(&r2);
}

int main(void)
{
    printf("UART\n");
    test_uart_roundtrip_all_formats();
    test_uart_parity_and_framing_errors();
    test_uart_break();
    test_uart_glitch_rejected();
    test_uart_majority_vote_noise();
    test_uart_autobaud();
    test_uart_baud_mismatch_timing();
    test_uart_truncated();

    printf("SPI\n");
    test_spi_roundtrip_all_modes();
    test_spi_16bit_words();
    test_spi_wrong_mode_detected();
    test_spi_detect_mode();
    test_spi_incomplete();

    printf("I2C\n");
    test_i2c_write();
    test_i2c_read_register_with_repeated_start();
    test_i2c_nack();
    test_i2c_timing_violations();
    test_i2c_interrupted_byte();

    printf("CAN\n");
    test_can_crc15_reference();
    test_can_roundtrip();
    test_can_errors();
    test_can_clock_mismatch();

    printf("CSV/VCD\n");
    test_csv_roundtrip();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
