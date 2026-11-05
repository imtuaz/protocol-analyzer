/*
 * uart.h - UART waveform generator, decoder and baud-rate detector.
 *
 * Frame (idle = 1):  START(0) | D0 .. D(n-1) LSB first | [PARITY] | STOP(1) x1/x2
 */
#ifndef UART_H
#define UART_H

#include "waveform.h"
#include "events.h"

typedef enum { PARITY_NONE, PARITY_EVEN, PARITY_ODD } parity_t;

typedef struct {
    uint32_t baud;        /* bit/s                  */
    int      data_bits;   /* 5..9                   */
    parity_t parity;
    int      stop_bits;   /* 1 or 2                 */
} uart_cfg_t;

/* Fault injection for the generator */
#define UART_FAULT_PARITY  (1 << 0)   /* send the wrong parity bit */
#define UART_FAULT_STOP    (1 << 1)   /* send stop bit as 0        */

/* Allowed baud mismatch before reporting a timing violation (%) */
#define UART_BAUD_TOLERANCE_PCT 2.0

void uart_cfg_default(uart_cfg_t *cfg, uint32_t baud);   /* 8N1 */

/* --- generator ------------------------------------------------------- */
int uart_gen_idle(waveform_t *w, int ch, const uart_cfg_t *cfg, double bits);
int uart_gen_byte(waveform_t *w, int ch, const uart_cfg_t *cfg,
                  uint16_t value, int faults);
int uart_gen_break(waveform_t *w, int ch, const uart_cfg_t *cfg, double bits);

/* --- decoder --------------------------------------------------------- */
void uart_decode(const waveform_t *w, int ch, const uart_cfg_t *cfg,
                 event_list_t *out);

/* --- baud rate detection --------------------------------------------- */
typedef struct {
    int      ok;              /* 1 if a bit time could be found        */
    double   measured_baud;   /* from the waveform                     */
    uint32_t standard_baud;   /* nearest standard value                */
    double   error_pct;       /* (measured - standard) / standard * 100 */
} baud_result_t;

int  uart_detect_baud(const waveform_t *w, int ch, baud_result_t *res);

/* Adds an EV_TIMING event with ERR_TIMING if the measured baud differs
 * from cfg->baud by more than UART_BAUD_TOLERANCE_PCT. */
void uart_check_timing(const waveform_t *w, int ch, const uart_cfg_t *cfg,
                       event_list_t *out);

#endif /* UART_H */
