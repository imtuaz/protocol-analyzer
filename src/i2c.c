#include <stdio.h>
#include "i2c.h"

/* ====================================================== timing ======= */

void i2c_timing_spec(i2c_timing_t *t, i2c_speed_t speed)
{
    if (speed == I2C_STANDARD) {           /* 100 kHz */
        t->t_low = 4.7e-6;  t->t_high = 4.0e-6;
        t->t_hd_sta = 4.0e-6; t->t_su_sta = 4.7e-6;
        t->t_su_dat = 250e-9; t->t_su_sto = 4.0e-6; t->t_buf = 4.7e-6;
    } else {                               /* 400 kHz */
        t->t_low = 1.3e-6;  t->t_high = 0.6e-6;
        t->t_hd_sta = 0.6e-6; t->t_su_sta = 0.6e-6;
        t->t_su_dat = 100e-9; t->t_su_sto = 0.6e-6; t->t_buf = 1.3e-6;
    }
}

void i2c_timing_for_freq(i2c_timing_t *t, double scl_hz)
{
    double period = 1.0 / scl_hz;
    t->t_low    = 0.56 * period;     /* low longer than high, like real masters */
    t->t_high   = 0.44 * period;
    t->t_hd_sta = 0.50 * period;
    t->t_su_sta = 0.50 * period;
    t->t_su_dat = 0.75 * t->t_low;   /* data changes 1/4 into the low phase */
    t->t_su_sto = 0.50 * period;
    t->t_buf    = 0.56 * period;
}

const char *i2c_timing_name(int index)
{
    static const char *names[I2C_NUM_TIMING] = {
        "tLOW", "tHIGH", "tHD;STA", "tSU;STA", "tSU;DAT", "tSU;STO", "tBUF"
    };
    return (index >= 0 && index < I2C_NUM_TIMING) ? names[index] : "?";
}

/* Read the timing struct as an array (same order as the names above) */
static double timing_get(const i2c_timing_t *t, int index)
{
    switch (index) {
    case 0: return t->t_low;
    case 1: return t->t_high;
    case 2: return t->t_hd_sta;
    case 3: return t->t_su_sta;
    case 4: return t->t_su_dat;
    case 5: return t->t_su_sto;
    default: return t->t_buf;
    }
}

/* ===================================================== generator ===== */

static uint8_t g_lv[2];            /* current SCL/SDA level while generating */

static void load_levels(const waveform_t *w)
{
    g_lv[I2C_CH_SCL] = wf_get(w, I2C_CH_SCL, w->len ? w->len - 1 : 0);
    g_lv[I2C_CH_SDA] = wf_get(w, I2C_CH_SDA, w->len ? w->len - 1 : 0);
}

static int emit(waveform_t *w, double seconds)
{
    return wf_emit(w, g_lv, seconds * w->sample_rate);
}

int i2c_wf_init(waveform_t *w, uint32_t sample_rate)
{
    const char *names[2] = { "SCL", "SDA" };
    uint8_t idle[2] = { 1, 1 };
    if (wf_init(w, sample_rate, 2, names) != 0)
        return -1;
    return wf_hold(w, idle, sample_rate / 100000 + 10);   /* ~10 us idle */
}

int i2c_gen_start(waveform_t *w, const i2c_timing_t *t)
{
    load_levels(w);
    g_lv[I2C_CH_SDA] = 0;                 /* SDA falls, SCL high */
    int r = emit(w, t->t_hd_sta);
    g_lv[I2C_CH_SCL] = 0;                 /* then SCL goes low   */
    return r;
}

/* Low phase of SCL: keep old SDA for a short hold time, then put the new
 * SDA level so it is stable (setup time) before SCL rises. */
static int low_phase(waveform_t *w, const i2c_timing_t *t, int sda)
{
    double hold = t->t_low - t->t_su_dat;
    int r = 0;
    g_lv[I2C_CH_SCL] = 0;
    if (hold > 0)
        r |= emit(w, hold);
    g_lv[I2C_CH_SDA] = (uint8_t)sda;
    r |= emit(w, t->t_su_dat);
    return r;
}

int i2c_gen_restart(waveform_t *w, const i2c_timing_t *t)
{
    load_levels(w);
    int r = low_phase(w, t, 1);           /* release SDA while SCL low */
    g_lv[I2C_CH_SCL] = 1;
    r |= emit(w, t->t_su_sta);
    g_lv[I2C_CH_SDA] = 0;                 /* START while SCL high      */
    r |= emit(w, t->t_hd_sta);
    g_lv[I2C_CH_SCL] = 0;
    return r;
}

int i2c_gen_stop(waveform_t *w, const i2c_timing_t *t)
{
    load_levels(w);
    int r = low_phase(w, t, 0);           /* SDA low while SCL low     */
    g_lv[I2C_CH_SCL] = 1;
    r |= emit(w, t->t_su_sto);
    g_lv[I2C_CH_SDA] = 1;                 /* SDA rises, SCL high: STOP */
    r |= emit(w, t->t_buf);
    return r;
}

int i2c_gen_byte(waveform_t *w, const i2c_timing_t *t, uint8_t byte, int ack)
{
    int r = 0;
    load_levels(w);
    for (int k = 7; k >= 0; k--) {        /* MSB first */
        r |= low_phase(w, t, (byte >> k) & 1);
        g_lv[I2C_CH_SCL] = 1;
        r |= emit(w, t->t_high);
    }
    r |= low_phase(w, t, ack ? 0 : 1);    /* 9th bit: 0 = ACK */
    g_lv[I2C_CH_SCL] = 1;
    r |= emit(w, t->t_high);
    g_lv[I2C_CH_SCL] = 0;
    return r;
}

int i2c_gen_write(waveform_t *w, const i2c_timing_t *t, uint8_t addr,
                  const uint8_t *data, int n, int nack_at)
{
    int r = i2c_gen_start(w, t);
    r |= i2c_gen_byte(w, t, (uint8_t)(addr << 1), nack_at != 0);
    for (int i = 0; i < n && nack_at != 0; i++) {
        r |= i2c_gen_byte(w, t, data[i], nack_at != i + 1);
        if (nack_at == i + 1)
            break;                        /* master stops after NACK */
    }
    r |= i2c_gen_stop(w, t);
    return r;
}

int i2c_gen_read_reg(waveform_t *w, const i2c_timing_t *t, uint8_t addr,
                     uint8_t reg, const uint8_t *data, int n)
{
    int r = i2c_gen_start(w, t);
    r |= i2c_gen_byte(w, t, (uint8_t)(addr << 1), 1);        /* addr + W */
    r |= i2c_gen_byte(w, t, reg, 1);
    r |= i2c_gen_restart(w, t);
    r |= i2c_gen_byte(w, t, (uint8_t)((addr << 1) | 1), 1);  /* addr + R */
    for (int i = 0; i < n; i++)
        r |= i2c_gen_byte(w, t, data[i], i < n - 1);         /* NACK last */
    r |= i2c_gen_stop(w, t);
    return r;
}

/* ======================================================= decoder ===== */

typedef struct {
    double   worst;       /* smallest measured value (seconds) */
    uint32_t where;       /* sample index of the worst value   */
    int      violations;  /* how many times below spec         */
} tstat_t;

static void tcheck(tstat_t *st, int idx, double measured, double spec_min,
                   double one_sample, uint32_t where)
{
    if (measured < st[idx].worst) {
        st[idx].worst = measured;
        st[idx].where = where;
    }
    /* allow 1 sample of measurement error */
    if (measured + one_sample < spec_min)
        st[idx].violations++;
}

void i2c_decode(const waveform_t *w, i2c_speed_t speed, event_list_t *out)
{
    const uint8_t *scl = w->data[I2C_CH_SCL];
    const uint8_t *sda = w->data[I2C_CH_SDA];
    double ts = 1.0 / w->sample_rate;            /* seconds per sample */
    i2c_timing_t spec;
    tstat_t st[I2C_NUM_TIMING];

    int      in_xfer = 0, is_addr = 0, read_mode = 0, bitcnt = 0;
    uint32_t byte = 0, byte_start = 0;
    uint32_t last_rise = 0, last_fall = 0, last_sda_change = 0;
    uint32_t start_at = 0, stop_at = 0;
    int      have_rise = 0, have_fall = 0, pending_hd_sta = 0, have_stop = 0;
    uint32_t min_period = 0xFFFFFFFFu, prev_rise = 0;

    if (w->num_ch < 2)
        return;
    i2c_timing_spec(&spec, speed);
    for (int k = 0; k < I2C_NUM_TIMING; k++) {
        st[k].worst = 1e9;
        st[k].where = 0;
        st[k].violations = 0;
    }

    for (uint32_t i = 1; i < w->len; i++) {
        int sda_changed = (sda[i] != sda[i - 1]);

        if (scl[i] == 1 && scl[i - 1] == 1 && sda_changed) {
            /* The SCL rise just before a START/STOP was counted as a bit,
             * but it was only the setup for this condition: discard it.
             * More than that bit -> a byte was really interrupted. */
            if (bitcnt > 1 && in_xfer) {
                event_t *e = ev_add(out, PROTO_I2C, EV_ERROR, byte_start, i);
                if (e) {
                    e->errors = ERR_FORM | ERR_INCOMPLETE;
                    snprintf(e->note, EV_NOTE_LEN, "byte interrupted after %d bits",
                             bitcnt - 1);
                }
            }
            if (sda[i] == 0) {                   /* ---- START ---- */
                event_t *e = ev_add(out, PROTO_I2C, EV_START, i, i);
                if (e)
                    snprintf(e->note, EV_NOTE_LEN, in_xfer ? "Sr" : "S");
                if (in_xfer && have_rise)
                    tcheck(st, 3, (i - last_rise) * ts, spec.t_su_sta, ts, i);
                if (!in_xfer && have_stop)
                    tcheck(st, 6, (i - stop_at) * ts, spec.t_buf, ts, i);
                in_xfer = 1;
                is_addr = 1;
                start_at = i;
                pending_hd_sta = 1;
            } else {                              /* ---- STOP ---- */
                event_t *e = ev_add(out, PROTO_I2C, EV_STOP, i, i);
                if (e)
                    snprintf(e->note, EV_NOTE_LEN, "P");
                if (have_rise)
                    tcheck(st, 5, (i - last_rise) * ts, spec.t_su_sto, ts, i);
                in_xfer = 0;
                have_stop = 1;
                stop_at = i;
            }
            bitcnt = 0;
            byte = 0;
            last_sda_change = i;
            continue;
        }
        if (sda_changed)
            last_sda_change = i;

        if (scl[i - 1] == 0 && scl[i] == 1) {    /* ---- SCL rising ---- */
            if (in_xfer) {
                if (have_fall) {
                    tcheck(st, 0, (i - last_fall) * ts, spec.t_low, ts, i);
                    if (last_sda_change > last_fall)
                        tcheck(st, 4, (i - last_sda_change) * ts, spec.t_su_dat, ts, i);
                }
                if (have_rise && i - prev_rise < min_period)
                    min_period = i - prev_rise;
                prev_rise = i;

                bitcnt++;
                if (bitcnt == 1)
                    byte_start = have_fall ? last_fall : i;
                if (bitcnt <= 8) {
                    byte = (byte << 1) | sda[i];
                } else {                         /* 9th bit = ACK/NACK */
                    int ack = (sda[i] == 0);
                    event_t *e;
                    if (is_addr) {
                        e = ev_add(out, PROTO_I2C, EV_ADDR, byte_start, i);
                        if (e) {
                            e->value = byte >> 1;
                            e->rw    = (int)(byte & 1);
                            e->ack   = ack;
                            if (!ack)
                                e->errors = ERR_NACK;
                            snprintf(e->note, EV_NOTE_LEN, "0x%02X %s %s",
                                     byte >> 1, (byte & 1) ? "R" : "W",
                                     ack ? "ACK" : "NACK (no device?)");
                        }
                        read_mode = (int)(byte & 1);
                        is_addr = 0;
                    } else {
                        e = ev_add(out, PROTO_I2C, EV_DATA, byte_start, i);
                        if (e) {
                            e->value = byte;
                            e->ack   = ack;
                            /* In a read, the master NACKs the last byte on
                             * purpose -> that is normal, not an error. */
                            if (!ack && !read_mode)
                                e->errors = ERR_NACK;
                            snprintf(e->note, EV_NOTE_LEN, "%s",
                                     ack ? "ACK" : (read_mode ? "NACK (end of read)"
                                                              : "NACK (slave refused)"));
                        }
                    }
                    bitcnt = 0;
                    byte = 0;
                }
            }
            last_rise = i;
            have_rise = 1;
        } else if (scl[i - 1] == 1 && scl[i] == 0) {   /* ---- SCL falling ---- */
            if (in_xfer && have_rise && !pending_hd_sta)
                tcheck(st, 1, (i - last_rise) * ts, spec.t_high, ts, i);
            if (pending_hd_sta) {
                tcheck(st, 2, (i - start_at) * ts, spec.t_hd_sta, ts, i);
                pending_hd_sta = 0;
            }
            last_fall = i;
            have_fall = 1;
        }
    }

    if (in_xfer && bitcnt != 0) {
        event_t *e = ev_add(out, PROTO_I2C, EV_ERROR, byte_start, w->len - 1);
        if (e) {
            e->errors = ERR_INCOMPLETE;
            snprintf(e->note, EV_NOTE_LEN, "capture ended inside a byte");
        }
    }

    /* One event per violated timing parameter, at its worst location */
    for (int k = 0; k < I2C_NUM_TIMING; k++) {
        if (st[k].violations == 0)
            continue;
        event_t *e = ev_add(out, PROTO_I2C, EV_TIMING, st[k].where, st[k].where);
        if (e) {
            e->errors = ERR_TIMING;
            snprintf(e->note, EV_NOTE_LEN, "%s min %.2f us < spec %.2f us (%d times)",
                     i2c_timing_name(k), st[k].worst * 1e6,
                     timing_get(&spec, k) * 1e6, st[k].violations);
        }
    }
    if (min_period != 0xFFFFFFFFu) {
        event_t *e = ev_add(out, PROTO_I2C, EV_TIMING, 0, 0);
        if (e)
            snprintf(e->note, EV_NOTE_LEN, "SCL max %.1f kHz",
                     w->sample_rate / (double)min_period / 1e3);
    }
}
