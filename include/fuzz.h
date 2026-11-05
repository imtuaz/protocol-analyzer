/*
 * fuzz.h - robustness testing of the decoders.
 *
 * For every iteration and protocol:
 *   1. random config + random payload -> clean waveform
 *   2. decode it: result must match exactly what was sent (round trip)
 *   3. inject random faults (glitches, inverted spans, noise, truncation)
 *   4. decode again: must not crash, events must stay inside the capture,
 *      and we classify the result:
 *        detected   : decoder reported at least one error
 *        silent     : payload changed but NO error reported
 *                     (e.g. a flipped data bit in UART without parity is
 *                      undetectable by design - the report shows how often)
 *        harmless   : fault did not change the decoded payload
 *
 * Run it under AddressSanitizer (make fuzz) to also catch memory bugs.
 */
#ifndef FUZZ_H
#define FUZZ_H

#include <stdio.h>
#include <stdint.h>

typedef struct {
    int clean_ok, clean_fail;          /* round-trip results              */
    int detected, silent, harmless;    /* faulty-waveform classification  */
    int invariant_fail;                /* event outside capture, etc.     */
} fuzz_stats_t;

/* stats must point to an array of 4 (UART, SPI, I2C, CAN) */
void fuzz_run(uint32_t seed, int iterations, fuzz_stats_t stats[4], FILE *log);
void fuzz_print(FILE *f, const fuzz_stats_t stats[4]);

#endif /* FUZZ_H */
