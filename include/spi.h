/*
 * spi.h - SPI generator, decoder and mode detector.
 *
 * Channels: 0 = CS (active low), 1 = SCLK, 2 = MOSI, 3 = MISO
 *
 *  Mode | CPOL (idle clock) | CPHA | data sampled on
 *  -----+-------------------+------+----------------------------------
 *   0   |        0          |  0   | rising edge  (1st edge)
 *   1   |        0          |  1   | falling edge (2nd edge)
 *   2   |        1          |  0   | falling edge (1st edge)
 *   3   |        1          |  1   | rising edge  (2nd edge)
 */
#ifndef SPI_H
#define SPI_H

#include "waveform.h"
#include "events.h"

#define SPI_CH_CS   0
#define SPI_CH_SCLK 1
#define SPI_CH_MOSI 2
#define SPI_CH_MISO 3

typedef struct {
    int cpol;        /* clock idle level            */
    int cpha;        /* 0: sample 1st edge, 1: 2nd  */
    int bits;        /* word size, usually 8        */
    int lsb_first;   /* 0: MSB first (common)       */
} spi_cfg_t;

void spi_cfg_mode(spi_cfg_t *cfg, int mode);       /* 8-bit MSB first */
int  spi_mode(const spi_cfg_t *cfg);

/* Create an empty 4-channel SPI waveform in idle state */
int  spi_wf_init(waveform_t *w, uint32_t sample_rate, const spi_cfg_t *cfg);

/* One CS-low transaction of n words. miso may be NULL (reads 0). */
int  spi_gen_transfer(waveform_t *w, const spi_cfg_t *cfg, double sclk_hz,
                      const uint32_t *mosi, const uint32_t *miso, int n);

/* Same, but CS is released after `bits_sent` bits of the last word
 * (fault injection: incomplete transfer). */
int  spi_gen_transfer_cut(waveform_t *w, const spi_cfg_t *cfg, double sclk_hz,
                          const uint32_t *mosi, int n, int bits_sent);

void spi_decode(const waveform_t *w, const spi_cfg_t *cfg, event_list_t *out);

/* Guess CPOL from the idle clock level, and CPHA from which clock edge
 * MOSI changes on (data changes on the NON-sampling edge).
 * Returns the mode 0..3, or -1 if there is not enough activity. */
int  spi_detect_mode(const waveform_t *w);

#endif /* SPI_H */
