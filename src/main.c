/*
 * main.c - command line interface.
 *
 *   pa demo   [outdir]                     simulate all protocols with faults
 *   pa analyze <file.csv> <proto> [opts]   decode a real/saved capture
 *   pa fuzz   [iterations] [seed]          robustness test
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "waveform.h"
#include "events.h"
#include "wave_io.h"
#include "uart.h"
#include "spi.h"
#include "i2c.h"
#include "can.h"
#include "report.h"
#include "fuzz.h"

static void usage(void)
{
    printf(
"Embedded Communication Protocol Analyzer\n\n"
"Usage:\n"
"  pa demo [outdir]                     simulate UART/SPI/I2C/CAN with injected\n"
"                                       faults, write report.html + .vcd + .csv\n"
"  pa analyze <file.csv> <proto> [options]\n"
"      proto: uart | spi | i2c | can\n"
"      --rate N         sample rate in Hz (default: guess from file)\n"
"      --ch N           channel index for uart/can (default 0)\n"
"      --baud N|auto    uart baud rate (default auto)\n"
"      --parity none|even|odd   --stop 1|2   --bits N\n"
"      --mode 0-3|auto  spi mode (default auto)   --lsb   spi LSB first\n"
"      --speed std|fast i2c timing spec (default std)\n"
"      --bitrate N      can bit rate (default 500000)\n"
"      --report FILE    write HTML report   --vcd FILE   write VCD\n"
"  pa fuzz [iterations] [seed]          robustness test (default 500, 1)\n");
}

/* ================================================================ demo */

#define N_DEMO 5

static int run_demo(const char *outdir)
{
    waveform_t   w[N_DEMO];
    event_list_t ev[N_DEMO];
    report_section_t sec[N_DEMO];
    char path[256];
    memset(sec, 0, sizeof sec);
    for (int i = 0; i < N_DEMO; i++)
        ev_init(&ev[i]);

    /* ---- 1. UART 115200 8E1 with parity/framing errors, break, glitch */
    {
        uart_cfg_t cfg;
        const char *msg = "Hello STM32!";
        baud_result_t br;
        uart_cfg_default(&cfg, 115200);
        cfg.parity = PARITY_EVEN;
        wf_init(&w[0], 10000000, 1, (const char *[]){ "UART_TX" });
        uart_gen_idle(&w[0], 0, &cfg, 4);
        for (int i = 0; msg[i]; i++) {
            int fault = (i == 4) ? UART_FAULT_PARITY : (i == 8) ? UART_FAULT_STOP : 0;
            uart_gen_byte(&w[0], 0, &cfg, (uint8_t)msg[i], fault);
            uart_gen_idle(&w[0], 0, &cfg, 1);
        }
        uart_gen_break(&w[0], 0, &cfg, 15);
        uart_gen_idle(&w[0], 0, &cfg, 6);
        wf_invert(&w[0], 0, w[0].len - 300, 5);         /* 0.5 us noise spike */
        uart_gen_byte(&w[0], 0, &cfg, '\n', 0);
        uart_gen_idle(&w[0], 0, &cfg, 4);

        uart_decode(&w[0], 0, &cfg, &ev[0]);
        uart_check_timing(&w[0], 0, &cfg, &ev[0]);
        uart_detect_baud(&w[0], 0, &br);
        snprintf(sec[0].title, sizeof sec[0].title, "UART - 115200 8E1");
        snprintf(sec[0].info, REPORT_INFO_LEN,
                 "Config  : 115200 baud, 8 data, even parity, 1 stop, 10 MS/s\n"
                 "Injected: parity error on byte 4, framing error on byte 8,\n"
                 "          BREAK, 0.5 us noise spike\n"
                 "Auto-baud: measured %.0f -> %u (%+.2f%%)",
                 br.measured_baud, br.standard_baud, br.error_pct);
    }

    /* ---- 2. UART with a transmitter clock 2.5 % too fast */
    {
        uart_cfg_t tx, rx;
        baud_result_t br;
        uart_cfg_default(&tx, 118080);
        uart_cfg_default(&rx, 115200);
        wf_init(&w[1], 10000000, 1, (const char *[]){ "UART_TX" });
        uart_gen_idle(&w[1], 0, &tx, 4);
        for (const char *p = "U*U*"; *p; p++)
            uart_gen_byte(&w[1], 0, &tx, (uint8_t)*p, 0);
        uart_gen_idle(&w[1], 0, &tx, 4);
        uart_decode(&w[1], 0, &rx, &ev[1]);
        uart_check_timing(&w[1], 0, &rx, &ev[1]);
        uart_detect_baud(&w[1], 0, &br);
        snprintf(sec[1].title, sizeof sec[1].title, "UART - baud rate mismatch");
        snprintf(sec[1].info, REPORT_INFO_LEN,
                 "Receiver expects 115200, transmitter clock is 2.5 %% fast\n"
                 "Auto-baud: measured %.0f -> nearest standard %u (%+.2f%%)\n"
                 "Bytes still decode, but the timing check flags the deviation\n"
                 "(UART tolerates about +-2 %% total mismatch)",
                 br.measured_baud, br.standard_baud, br.error_pct);
    }

    /* ---- 3. SPI mode 3: read JEDEC ID of a flash chip + cut transfer */
    {
        spi_cfg_t cfg;
        const uint32_t mo[] = { 0x9F, 0x00, 0x00, 0x00 };
        const uint32_t mi[] = { 0xFF, 0xEF, 0x40, 0x18 };
        const uint32_t mo2[] = { 0x03, 0x00 };
        spi_cfg_mode(&cfg, 3);
        spi_wf_init(&w[2], 20000000, &cfg);
        spi_gen_transfer(&w[2], &cfg, 2e6, mo, mi, 4);
        spi_gen_transfer_cut(&w[2], &cfg, 2e6, mo2, 2, 5);
        int detected = spi_detect_mode(&w[2]);
        spi_decode(&w[2], &cfg, &ev[2]);
        snprintf(sec[2].title, sizeof sec[2].title, "SPI - mode 3, 2 MHz");
        snprintf(sec[2].info, REPORT_INFO_LEN,
                 "Transfer 1: JEDEC ID read 0x9F -> flash answers EF 40 18\n"
                 "Transfer 2: CS released after 5 bits of 2nd byte (injected)\n"
                 "Auto mode detection: mode %d (CPOL=%d CPHA=%d)",
                 detected, (detected >> 1) & 1, detected & 1);
    }

    /* ---- 4. I2C: MPU6050 register read, absent device, too-fast timing */
    {
        i2c_timing_t t, bad;
        const uint8_t who[] = { 0x68 };
        const uint8_t cmd[] = { 0x00, 0xAE };
        i2c_timing_for_freq(&t, 400e3);
        i2c_wf_init(&w[3], 20000000);
        i2c_gen_read_reg(&w[3], &t, 0x68, 0x75, who, 1);   /* WHO_AM_I */
        i2c_gen_write(&w[3], &t, 0x3C, cmd, 2, 0);        /* no device */
        bad = t;
        bad.t_high = 0.45e-6;                              /* < 0.6 us */
        i2c_gen_write(&w[3], &bad, 0x68, cmd, 1, -1);
        i2c_decode(&w[3], I2C_FAST, &ev[3]);
        snprintf(sec[3].title, sizeof sec[3].title, "I2C - Fast-mode 400 kHz");
        snprintf(sec[3].info, REPORT_INFO_LEN,
                 "1. Read MPU6050 WHO_AM_I (reg 0x75) with repeated START\n"
                 "2. Write to 0x3C: nobody answers -> address NACK\n"
                 "3. Write with SCL high time 0.45 us (spec min 0.6 us)");
    }

    /* ---- 5. CAN 500 kbit/s: OK frame, CRC error, stuff error, no ACK */
    {
        can_frame_t f[4];
        const int faults[4] = { 0, CAN_FAULT_FLIP_BIT, CAN_FAULT_NO_STUFF, CAN_FAULT_NO_ACK };
        memset(f, 0, sizeof f);
        f[0].id = 0x123; f[0].dlc = 2; f[0].data[0] = 0x0F; f[0].data[1] = 0xA0;
        f[1].id = 0x2F0; f[1].dlc = 4; f[1].data[0] = 0xDE; f[1].data[1] = 0xAD;
        f[1].data[2] = 0xBE; f[1].data[3] = 0xEF;
        f[2].id = 0x100; f[2].dlc = 1; f[2].data[0] = 0x00;
        f[3].id = 0x7E8; f[3].dlc = 3; f[3].data[0] = 0x02; f[3].data[1] = 0x41;
        f[3].data[2] = 0x0D;
        can_wf_init(&w[4], 10000000);
        can_gen_idle(&w[4], 500000, 11);
        for (int i = 0; i < 4; i++)
            can_gen_frame(&w[4], 500000, &f[i], faults[i]);
        can_gen_idle(&w[4], 500000, 4);
        can_decode(&w[4], 0, 500000, &ev[4]);
        snprintf(sec[4].title, sizeof sec[4].title, "CAN - 500 kbit/s");
        snprintf(sec[4].info, REPORT_INFO_LEN,
                 "Frame 1: valid    Frame 2: one bit flipped -> CRC error\n"
                 "Frame 3: wrong stuff bit -> stuff error\n"
                 "Frame 4: no node acknowledges -> ACK error");
    }

    for (int i = 0; i < N_DEMO; i++) {
        sec[i].w  = &w[i];
        sec[i].ev = &ev[i];
        ev_sort_by_time(&ev[i]);
        printf("\n=== %s ===\n", sec[i].title);
        print_events(stdout, &w[i], &ev[i]);
        print_summary(stdout, &ev[i]);

        static const char *fname[N_DEMO] = { "uart", "uart_mismatch", "spi", "i2c", "can" };
        snprintf(path, sizeof path, "%s/%s.vcd", outdir, fname[i]);
        wf_save_vcd(&w[i], path);
        snprintf(path, sizeof path, "%s/%s.csv", outdir, fname[i]);
        wf_save_csv(&w[i], path);
    }

    snprintf(path, sizeof path, "%s/report.html", outdir);
    int r = report_write_html(path, "Protocol Analyzer - Demo", sec, N_DEMO);
    printf("\nReport: %s   (waveforms: %s/*.vcd for GTKWave, *.csv)\n", path, outdir);

    for (int i = 0; i < N_DEMO; i++) {
        ev_free(&ev[i]);
        wf_free(&w[i]);
    }
    return r;
}

/* ============================================================= analyze */

static const char *opt(int argc, char **argv, const char *name)
{
    for (int i = 0; i < argc - 1; i++)
        if (strcmp(argv[i], name) == 0)
            return argv[i + 1];
    return NULL;
}

static int flag(int argc, char **argv, const char *name)
{
    for (int i = 0; i < argc; i++)
        if (strcmp(argv[i], name) == 0)
            return 1;
    return 0;
}

static int run_analyze(int argc, char **argv)
{
    if (argc < 4) {
        usage();
        return 1;
    }
    const char *file = argv[2], *proto = argv[3];
    const char *v;
    uint32_t rate = (v = opt(argc, argv, "--rate")) ? (uint32_t)atol(v) : 0;
    int ch = (v = opt(argc, argv, "--ch")) ? atoi(v) : 0;
    waveform_t w;
    event_list_t ev;
    report_section_t sec;
    memset(&sec, 0, sizeof sec);

    if (wf_load_csv(&w, file, rate) != 0) {
        fprintf(stderr, "cannot read %s\n", file);
        return 1;
    }
    if (ch < 0 || ch >= w.num_ch) {
        fprintf(stderr, "channel %d not in file (%d channels)\n", ch, w.num_ch);
        wf_free(&w);
        return 1;
    }
    ev_init(&ev);
    printf("Loaded %s: %d channels, %u samples @ %u Hz (%.3f ms)\n", file,
           w.num_ch, w.len, w.sample_rate, 1e3 * w.len / w.sample_rate);

    if (strcmp(proto, "uart") == 0) {
        uart_cfg_t cfg;
        baud_result_t br;
        uart_cfg_default(&cfg, 115200);
        v = opt(argc, argv, "--baud");
        if (!v || strcmp(v, "auto") == 0) {
            if (uart_detect_baud(&w, ch, &br) == 0) {
                cfg.baud = br.standard_baud;
                printf("Auto-baud: measured %.0f -> using %u\n", br.measured_baud, cfg.baud);
            } else {
                printf("Auto-baud failed, using 115200\n");
            }
        } else {
            cfg.baud = (uint32_t)atol(v);
        }
        if ((v = opt(argc, argv, "--parity")))
            cfg.parity = !strcmp(v, "even") ? PARITY_EVEN : !strcmp(v, "odd") ? PARITY_ODD : PARITY_NONE;
        if ((v = opt(argc, argv, "--stop"))) cfg.stop_bits = atoi(v);
        if ((v = opt(argc, argv, "--bits"))) cfg.data_bits = atoi(v);
        uart_decode(&w, ch, &cfg, &ev);
        uart_check_timing(&w, ch, &cfg, &ev);
        snprintf(sec.info, REPORT_INFO_LEN, "UART %u baud, %d data bits, parity %s, %d stop",
                 cfg.baud, cfg.data_bits,
                 cfg.parity == PARITY_NONE ? "none" : cfg.parity == PARITY_EVEN ? "even" : "odd",
                 cfg.stop_bits);
    } else if (strcmp(proto, "spi") == 0) {
        spi_cfg_t cfg;
        int mode;
        v = opt(argc, argv, "--mode");
        if (!v || strcmp(v, "auto") == 0) {
            mode = spi_detect_mode(&w);
            printf("Auto mode: %d\n", mode);
            if (mode < 0) mode = 0;
        } else {
            mode = atoi(v) & 3;
        }
        spi_cfg_mode(&cfg, mode);
        cfg.lsb_first = flag(argc, argv, "--lsb");
        if ((v = opt(argc, argv, "--bits"))) cfg.bits = atoi(v);
        spi_decode(&w, &cfg, &ev);
        snprintf(sec.info, REPORT_INFO_LEN, "SPI mode %d, %d-bit words, %s first",
                 mode, cfg.bits, cfg.lsb_first ? "LSB" : "MSB");
    } else if (strcmp(proto, "i2c") == 0) {
        v = opt(argc, argv, "--speed");
        i2c_speed_t sp = (v && strcmp(v, "fast") == 0) ? I2C_FAST : I2C_STANDARD;
        i2c_decode(&w, sp, &ev);
        snprintf(sec.info, REPORT_INFO_LEN, "I2C, timing checked against %s spec",
                 sp == I2C_FAST ? "Fast-mode (400 kHz)" : "Standard-mode (100 kHz)");
    } else if (strcmp(proto, "can") == 0) {
        uint32_t br = (v = opt(argc, argv, "--bitrate")) ? (uint32_t)atol(v) : 500000;
        can_decode(&w, ch, br, &ev);
        snprintf(sec.info, REPORT_INFO_LEN, "CAN %u bit/s, channel %d", br, ch);
    } else {
        fprintf(stderr, "unknown protocol '%s'\n", proto);
        ev_free(&ev);
        wf_free(&w);
        return 1;
    }

    ev_sort_by_time(&ev);
    print_events(stdout, &w, &ev);
    print_summary(stdout, &ev);

    if ((v = opt(argc, argv, "--report"))) {
        snprintf(sec.title, sizeof sec.title, "%s", file);
        sec.w = &w;
        sec.ev = &ev;
        if (report_write_html(v, "Protocol Analyzer", &sec, 1) == 0)
            printf("Report: %s\n", v);
    }
    if ((v = opt(argc, argv, "--vcd")) && wf_save_vcd(&w, v) == 0)
        printf("VCD: %s\n", v);

    int errors = ev_count_errors(&ev);
    ev_free(&ev);
    wf_free(&w);
    return errors ? 2 : 0;          /* exit code 2 = errors found (useful in CI) */
}

/* ================================================================ main */

int main(int argc, char **argv)
{
    if (argc < 2) {
        usage();
        return 1;
    }
    if (strcmp(argv[1], "demo") == 0)
        return run_demo(argc > 2 ? argv[2] : ".") ? 1 : 0;
    if (strcmp(argv[1], "analyze") == 0)
        return run_analyze(argc, argv);
    if (strcmp(argv[1], "fuzz") == 0) {
        int iters = argc > 2 ? atoi(argv[2]) : 500;
        uint32_t seed = argc > 3 ? (uint32_t)strtoul(argv[3], NULL, 10) : 1;
        fuzz_stats_t st[4];
        printf("Fuzzing %d iterations x 4 protocols (seed %u)...\n", iters, seed);
        fuzz_run(seed, iters, st, stdout);
        fuzz_print(stdout, st);
        int bad = 0;
        for (int p = 0; p < 4; p++)
            bad += st[p].clean_fail + st[p].invariant_fail;
        printf(bad ? "FAILED\n" : "PASSED: all round trips exact, no invariant broken\n");
        return bad ? 1 : 0;
    }
    usage();
    return 1;
}
