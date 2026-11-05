#include <stdio.h>
#include <string.h>
#include "can.h"

/* CRC-15/CAN, polynomial x^15+x^14+x^10+x^8+x^7+x^4+x^3+1 = 0x4599.
 * Computed bit by bit over the unstuffed bits from SOF to end of data. */
uint16_t can_crc15(const uint8_t *bits, int n)
{
    uint16_t crc = 0;
    for (int i = 0; i < n; i++) {
        int crc_next = bits[i] ^ ((crc >> 14) & 1);
        crc = (uint16_t)((crc << 1) & 0x7FFF);
        if (crc_next)
            crc ^= 0x4599;
    }
    return crc;
}

/* ===================================================== generator ===== */

int can_wf_init(waveform_t *w, uint32_t sample_rate)
{
    const char *names[1] = { "CAN_RX" };
    return wf_init(w, sample_rate, 1, names);
}

int can_gen_idle(waveform_t *w, uint32_t bitrate, double bits)
{
    uint8_t lv = 1;
    return wf_emit(w, &lv, bits * w->sample_rate / bitrate);
}

static void push_bits(uint8_t *arr, int *n, uint32_t value, int nbits)
{
    for (int k = nbits - 1; k >= 0; k--)          /* MSB first */
        arr[(*n)++] = (uint8_t)((value >> k) & 1);
}

int can_gen_frame(waveform_t *w, uint32_t bitrate, const can_frame_t *f, int faults)
{
    uint8_t raw[128], tx[200];
    int n = 0, m = 0;
    int dlc = (f->dlc > 8) ? 8 : f->dlc;
    int nbytes = f->rtr ? 0 : dlc;

    /* 1. Unstuffed bits SOF..DATA */
    raw[n++] = 0;                                 /* SOF */
    push_bits(raw, &n, f->id & 0x7FF, 11);
    raw[n++] = (uint8_t)(f->rtr ? 1 : 0);         /* RTR */
    raw[n++] = 0;                                 /* IDE: standard frame */
    raw[n++] = 0;                                 /* r0 */
    push_bits(raw, &n, (uint32_t)dlc, 4);
    for (int i = 0; i < nbytes; i++)
        push_bits(raw, &n, f->data[i], 8);

    /* 2. CRC over those bits, then append it */
    uint16_t crc = can_crc15(raw, n);
    push_bits(raw, &n, crc, 15);

    /* Fault: corrupt one bit AFTER the CRC was computed -> receiver sees CRC error */
    if (faults & CAN_FAULT_FLIP_BIT)
        raw[nbytes ? 19 : 5] ^= 1;

    /* 3. Bit stuffing: after 5 equal bits insert the opposite bit */
    int prev = -1, run = 0, broke_stuff = 0;
    for (int i = 0; i < n; i++) {
        tx[m++] = raw[i];
        if (raw[i] == prev) {
            run++;
        } else {
            prev = raw[i];
            run = 1;
        }
        if (run == 5) {
            int stuff = !prev;
            if ((faults & CAN_FAULT_NO_STUFF) && !broke_stuff) {
                stuff = prev;                     /* 6th equal bit: illegal */
                broke_stuff = 1;
            }
            tx[m++] = (uint8_t)stuff;
            prev = stuff;
            run = 1;
        }
    }

    /* 4. Fixed-form tail (never stuffed) */
    tx[m++] = (faults & CAN_FAULT_BAD_DELIM) ? 0 : 1;  /* CRC delimiter */
    tx[m++] = (faults & CAN_FAULT_NO_ACK) ? 1 : 0;     /* ACK slot: receiver drives 0 */
    tx[m++] = 1;                                       /* ACK delimiter */
    for (int i = 0; i < 7; i++)
        tx[m++] = 1;                                   /* EOF */
    for (int i = 0; i < 3; i++)
        tx[m++] = 1;                                   /* interframe space */

    int r = 0;
    double spb = (double)w->sample_rate / bitrate;
    for (int i = 0; i < m; i++)
        r |= wf_emit(w, &tx[i], spb);
    return r;
}

/* ======================================================= decoder ===== */

/* Hard synchronisation: if the line changes near the expected bit
 * boundary, the next bit starts at that edge (absorbs clock mismatch). */
static double next_bit(const waveform_t *w, int ch, double bit_start, double spb)
{
    uint32_t lo = (uint32_t)(bit_start + 0.75 * spb);
    uint32_t hi = (uint32_t)(bit_start + 1.25 * spb);
    uint32_t e  = wf_next_edge(w, ch, lo);
    if (e > lo && e <= hi)
        return e;
    return bit_start + spb;
}

/* Skip until the bus has been recessive for 7 bit times (after an error) */
static uint32_t wait_idle(const waveform_t *w, int ch, uint32_t from, double spb)
{
    uint32_t run = 0;
    for (uint32_t i = from; i < w->len; i++) {
        run = w->data[ch][i] ? run + 1 : 0;
        if (run >= 7 * spb)
            return i;
    }
    return w->len;
}

static uint32_t bits_value(const uint8_t *b, int from, int n)
{
    uint32_t v = 0;
    for (int i = 0; i < n; i++)
        v = (v << 1) | b[from + i];
    return v;
}

void can_decode(const waveform_t *w, int ch, uint32_t bitrate, event_list_t *out)
{
    const uint8_t *d = w->data[ch];
    double spb = (double)w->sample_rate / bitrate;
    uint32_t i = 1;

    if (w->len < 2)
        return;
    if (spb < 3.0) {
        event_t *e = ev_add(out, PROTO_CAN, EV_ERROR, 0, w->len - 1);
        if (e) {
            e->errors = ERR_TIMING;
            snprintf(e->note, EV_NOTE_LEN, "sample rate too low: %.1f samples/bit (need >= 3)", spb);
        }
        return;
    }

    while (i < w->len) {
        /* 1. SOF = recessive -> dominant edge */
        while (i < w->len && !(d[i - 1] == 1 && d[i] == 0))
            i++;
        if (i >= w->len)
            break;

        uint32_t sof = i;
        double   bs = i;                   /* start of current bit */
        uint8_t  bits[160];
        int      nb = 0, need = 19, run_val = -1, run_len = 0, expect_stuff = 0;
        uint32_t errors = 0;
        int      truncated = 0, ide = 0;
        char     why[40] = "";

        /* 2. Read and destuff until the end of the CRC field */
        while (nb < need || expect_stuff) {
            double c = bs + 0.5 * spb;
            if (c >= w->len) {
                truncated = 1;
                break;
            }
            int b = d[(uint32_t)c];
            bs = next_bit(w, ch, bs, spb);

            if (expect_stuff) {
                if (b == run_val) {
                    errors |= ERR_STUFF;
                    snprintf(why, sizeof why, "6 equal bits after bit %d", nb);
                    break;
                }
                run_val = b;               /* stuff bit: discard, new run */
                run_len = 1;
                expect_stuff = 0;
                continue;
            }

            bits[nb++] = (uint8_t)b;
            if (b == run_val) {
                run_len++;
            } else {
                run_val = b;
                run_len = 1;
            }
            if (run_len == 5)
                expect_stuff = 1;

            if (nb == 19) {                /* header complete: know DLC */
                ide = bits[13];
                if (ide) {
                    errors |= ERR_FORM;
                    snprintf(why, sizeof why, "extended (29-bit) ID not supported");
                    break;
                }
                int rtr = bits[12];
                int dlc = (int)bits_value(bits, 15, 4);
                int nbytes = rtr ? 0 : (dlc > 8 ? 8 : dlc);
                need = 19 + 8 * nbytes + 15;
            }
        }

        /* 3. Fixed-form tail: CRC delim, ACK slot, ACK delim, EOF x7 */
        int ack = -1;
        if (!errors && !truncated) {
            int tail[10];
            for (int k = 0; k < 10; k++) {
                double c = bs + 0.5 * spb;
                if (c >= w->len) {
                    truncated = 1;
                    break;
                }
                tail[k] = d[(uint32_t)c];
                bs = next_bit(w, ch, bs, spb);
            }
            if (!truncated) {
                if (tail[0] != 1) {
                    errors |= ERR_FORM;
                    snprintf(why, sizeof why, "CRC delimiter dominant");
                }
                ack = (tail[1] == 0);
                if (!ack)
                    errors |= ERR_NACK;
                if (tail[2] != 1) {
                    errors |= ERR_FORM;
                    snprintf(why, sizeof why, "ACK delimiter dominant");
                }
                for (int k = 3; k < 10; k++)
                    if (tail[k] != 1) {
                        errors |= ERR_FORM;
                        snprintf(why, sizeof why, "EOF bit %d dominant", k - 3);
                        break;
                    }
            }
        }
        if (truncated)
            errors |= ERR_INCOMPLETE;

        /* 4. Build the event */
        uint32_t end = (uint32_t)bs;
        if (end >= w->len)
            end = w->len - 1;
        event_t *e = ev_add(out, PROTO_CAN, EV_FRAME, sof, end);
        if (e) {
            e->ack = ack;
            e->dlc = -1;                       /* until the header is known */
            if (nb >= 12)
                e->value = bits_value(bits, 1, 11);
            if (nb >= 19 && !ide) {
                int rtr = bits[12];
                int dlc = (int)bits_value(bits, 15, 4);
                int nbytes = rtr ? 0 : (dlc > 8 ? 8 : dlc);
                e->dlc = dlc;
                if (nb == need) {
                    for (int k = 0; k < nbytes; k++)
                        e->data[k] = (uint8_t)bits_value(bits, 19 + 8 * k, 8);
                    uint16_t rx_crc = (uint16_t)bits_value(bits, need - 15, 15);
                    uint16_t calc   = can_crc15(bits, need - 15);
                    if (rx_crc != calc) {
                        errors |= ERR_CRC;
                        snprintf(why, sizeof why, "CRC rx 0x%04X calc 0x%04X", rx_crc, calc);
                    }
                }
                int pos = snprintf(e->note, EV_NOTE_LEN, "ID 0x%03X %sDLC %d",
                                   e->value, rtr ? "RTR " : "", dlc);
                if (why[0] && pos < EV_NOTE_LEN)
                    snprintf(e->note + pos, (size_t)(EV_NOTE_LEN - pos), " | %s", why);
            } else if (why[0]) {
                if (nb >= 12)
                    snprintf(e->note, EV_NOTE_LEN, "ID 0x%03X | %s", e->value, why);
                else
                    snprintf(e->note, EV_NOTE_LEN, "%s (ID not complete)", why);
            } else if (truncated) {
                snprintf(e->note, EV_NOTE_LEN, "frame cut at end of capture");
            }
            e->errors = errors;
        }

        /* 5. Continue. If decoding stopped in the middle of the frame
         *    (stuff error, unsupported format), the rest of that frame is
         *    garbage: wait until the bus is idle. Otherwise we are already
         *    past EOF and can look for the next SOF directly. */
        int stopped_early = (nb < need) || expect_stuff;
        i = stopped_early ? wait_idle(w, ch, (uint32_t)bs, spb) : (uint32_t)bs;
        if (i == 0)
            i = 1;
    }
}
