#include <stdio.h>
#include "spi.h"

void spi_cfg_mode(spi_cfg_t *cfg, int mode)
{
    cfg->cpol      = (mode >> 1) & 1;
    cfg->cpha      = mode & 1;
    cfg->bits      = 8;
    cfg->lsb_first = 0;
}

int spi_mode(const spi_cfg_t *cfg)
{
    return (cfg->cpol << 1) | cfg->cpha;
}

int spi_wf_init(waveform_t *w, uint32_t sample_rate, const spi_cfg_t *cfg)
{
    const char *names[4] = { "CS", "SCLK", "MOSI", "MISO" };
    uint8_t idle[4];
    if (wf_init(w, sample_rate, 4, names) != 0)
        return -1;
    idle[SPI_CH_CS]   = 1;
    idle[SPI_CH_SCLK] = (uint8_t)cfg->cpol;
    idle[SPI_CH_MOSI] = 0;
    idle[SPI_CH_MISO] = 0;
    return wf_hold(w, idle, 10);
}

/* ===================================================== generator ===== */

/* Bit number k (0 = first on the wire) of a word */
static int word_bit(uint32_t v, int k, const spi_cfg_t *cfg)
{
    int idx = cfg->lsb_first ? k : (cfg->bits - 1 - k);
    return (v >> idx) & 1;
}

/* Send n words but only total_bits bits in total, then release CS */
static int gen(waveform_t *w, const spi_cfg_t *cfg, double sclk_hz,
               const uint32_t *mosi, const uint32_t *miso, int n, int total_bits)
{
    double  half = w->sample_rate / (2.0 * sclk_hz);   /* samples per half clock */
    uint8_t lv[4];
    int     r = 0, sent = 0;

    for (int c = 0; c < 4; c++)
        lv[c] = wf_get(w, c, w->len ? w->len - 1 : 0);
    lv[SPI_CH_SCLK] = (uint8_t)cfg->cpol;

    lv[SPI_CH_CS] = 0;                       /* select slave */
    r |= wf_emit(w, lv, half);

    for (int i = 0; i < n && sent < total_bits; i++) {
        for (int k = 0; k < cfg->bits && sent < total_bits; k++, sent++) {
            uint8_t mo = (uint8_t)word_bit(mosi[i], k, cfg);
            uint8_t mi = (uint8_t)(miso ? word_bit(miso[i], k, cfg) : 0);
            if (cfg->cpha == 0) {
                /* data set while clock idle, sampled on 1st (leading) edge */
                lv[SPI_CH_MOSI] = mo;
                lv[SPI_CH_MISO] = mi;
                lv[SPI_CH_SCLK] = (uint8_t)cfg->cpol;
                r |= wf_emit(w, lv, half);
                lv[SPI_CH_SCLK] = (uint8_t)!cfg->cpol;
                r |= wf_emit(w, lv, half);
            } else {
                /* data set on 1st edge, sampled on 2nd (trailing) edge */
                lv[SPI_CH_SCLK] = (uint8_t)!cfg->cpol;
                lv[SPI_CH_MOSI] = mo;
                lv[SPI_CH_MISO] = mi;
                r |= wf_emit(w, lv, half);
                lv[SPI_CH_SCLK] = (uint8_t)cfg->cpol;
                r |= wf_emit(w, lv, half);
            }
        }
    }

    lv[SPI_CH_SCLK] = (uint8_t)cfg->cpol;    /* clock back to idle   */
    r |= wf_emit(w, lv, half);
    lv[SPI_CH_CS] = 1;                       /* deselect             */
    r |= wf_emit(w, lv, 4 * half);
    return r;
}

int spi_gen_transfer(waveform_t *w, const spi_cfg_t *cfg, double sclk_hz,
                     const uint32_t *mosi, const uint32_t *miso, int n)
{
    return gen(w, cfg, sclk_hz, mosi, miso, n, n * cfg->bits);
}

int spi_gen_transfer_cut(waveform_t *w, const spi_cfg_t *cfg, double sclk_hz,
                         const uint32_t *mosi, int n, int bits_sent)
{
    return gen(w, cfg, sclk_hz, mosi, NULL, n, (n - 1) * cfg->bits + bits_sent);
}

/* ======================================================= decoder ===== */

void spi_decode(const waveform_t *w, const spi_cfg_t *cfg, event_list_t *out)
{
    const uint8_t *cs   = w->data[SPI_CH_CS];
    const uint8_t *clk  = w->data[SPI_CH_SCLK];
    const uint8_t *mosi = w->data[SPI_CH_MOSI];
    const uint8_t *miso = (w->num_ch > SPI_CH_MISO) ? w->data[SPI_CH_MISO] : NULL;
    int sample_on_rising = (cfg->cpol == cfg->cpha);    /* modes 0 and 3 */

    if (w->num_ch < 3)
        return;

    for (uint32_t i = 1; i < w->len; i++) {
        if (!(cs[i - 1] == 1 && cs[i] == 0))
            continue;                            /* wait for CS falling */

        uint32_t cs_start = i, word_start = i, first_edge = 0;
        uint32_t vmo = 0, vmi = 0, err = 0;
        uint32_t min_period = 0xFFFFFFFFu, last_sample_edge = 0;
        int      nbits = 0, form_err = 0;
        uint32_t j;

        /* Clock must be at its idle level when CS goes low */
        if (clk[i] != cfg->cpol)
            form_err = 1;

        for (j = i + 1; j < w->len && cs[j] == 0; j++) {
            if (clk[j] == clk[j - 1])
                continue;                        /* no clock edge */
            if (first_edge == 0)
                first_edge = j;
            int rising = (clk[j] == 1);
            if (rising != sample_on_rising)
                continue;                        /* not a sampling edge */

            if (last_sample_edge && j - last_sample_edge < min_period)
                min_period = j - last_sample_edge;
            last_sample_edge = j;

            int bmo = mosi[j];
            int bmi = miso ? miso[j] : 0;

            /* Data must be stable before the sampling edge (setup time).
             * If it changes exactly on this edge, the mode is probably wrong. */
            if (mosi[j] != mosi[j - 1] || (miso && miso[j] != miso[j - 1]))
                err |= ERR_TIMING;

            if (cfg->lsb_first) {
                vmo |= (uint32_t)bmo << nbits;
                vmi |= (uint32_t)bmi << nbits;
            } else {
                vmo = (vmo << 1) | (uint32_t)bmo;
                vmi = (vmi << 1) | (uint32_t)bmi;
            }
            nbits++;

            if (nbits == cfg->bits) {
                event_t *e = ev_add(out, PROTO_SPI, EV_DATA, word_start, j);
                if (e) {
                    e->value  = vmo;
                    e->value2 = vmi;
                    e->errors = err | (form_err ? ERR_FORM : 0);
                    if (err & ERR_TIMING)
                        snprintf(e->note, EV_NOTE_LEN, "data changed on sampling edge (wrong CPHA?)");
                    else if (form_err)
                        snprintf(e->note, EV_NOTE_LEN, "SCLK idle level != CPOL");
                }
                nbits = 0;
                vmo = vmi = err = 0;
                word_start = j + 1;
            }
        }

        if (nbits > 0) {                         /* CS released too early */
            event_t *e = ev_add(out, PROTO_SPI, EV_ERROR, word_start, j - 1);
            if (e) {
                e->value  = vmo;
                e->errors = ERR_INCOMPLETE;
                snprintf(e->note, EV_NOTE_LEN, "CS released after %d of %d bits",
                         nbits, cfg->bits);
            }
        }

        /* Timing summary of this transaction */
        if (min_period != 0xFFFFFFFFu) {
            double f_hz = (double)w->sample_rate / min_period;
            double setup_half_periods = first_edge
                ? (double)(first_edge - cs_start) / (min_period / 2.0) : 0;
            event_t *e = ev_add(out, PROTO_SPI, EV_TIMING, cs_start, j - 1);
            if (e) {
                if (first_edge && setup_half_periods < 0.5) {
                    e->errors = ERR_TIMING;
                    snprintf(e->note, EV_NOTE_LEN, "CS->SCLK setup only %.2f half-periods",
                             setup_half_periods);
                } else {
                    snprintf(e->note, EV_NOTE_LEN, "SCLK %.3f MHz", f_hz / 1e6);
                }
            }
        }
        i = j;
    }
}

/* ================================================ mode detection ===== */

int spi_detect_mode(const waveform_t *w)
{
    const uint8_t *cs  = w->data[SPI_CH_CS];
    const uint8_t *clk = w->data[SPI_CH_SCLK];
    int cpol_votes[2] = { 0, 0 };
    int cpha_votes[2] = { 0, 0 };

    if (w->num_ch < 3)
        return -1;

    /* CPOL: clock level whenever CS goes low */
    for (uint32_t i = 1; i < w->len; i++)
        if (cs[i - 1] == 1 && cs[i] == 0)
            cpol_votes[clk[i]]++;
    if (cpol_votes[0] + cpol_votes[1] == 0)
        return -1;
    int cpol = cpol_votes[1] > cpol_votes[0];

    /* CPHA: look at data transitions that happen at (or just after) a clock
     * edge. Changing on the leading edge -> CPHA=1, trailing edge -> CPHA=0 */
    uint32_t last_edge = 0;
    int      last_leading = 0, have_edge = 0;
    for (uint32_t i = 1; i < w->len; i++) {
        if (cs[i] != 0) {
            have_edge = 0;
            continue;
        }
        if (clk[i] != clk[i - 1]) {
            have_edge    = 1;
            last_edge    = i;
            last_leading = (clk[i] != cpol);     /* moving away from idle */
        }
        for (int c = SPI_CH_MOSI; c < w->num_ch && c <= SPI_CH_MISO; c++) {
            if (w->data[c][i] != w->data[c][i - 1] && have_edge && i - last_edge <= 1)
                cpha_votes[last_leading ? 1 : 0]++;
        }
    }
    if (cpha_votes[0] + cpha_votes[1] == 0)
        return -1;
    int cpha = cpha_votes[1] > cpha_votes[0];
    return (cpol << 1) | cpha;
}
