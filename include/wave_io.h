/*
 * wave_io.h - load/save waveforms.
 *
 * CSV  : "time_s,CH0,CH1,..." one row per change (or per sample).
 *        This is the format exported by PulseView / Saleae Logic, so a
 *        real capture from a cheap USB logic analyzer can be analyzed.
 * VCD  : Value Change Dump, open with GTKWave to view the waveform.
 */
#ifndef WAVE_IO_H
#define WAVE_IO_H

#include "waveform.h"

int wf_save_csv(const waveform_t *w, const char *path);

/* sample_rate = 0 -> guess it from the smallest time step in the file */
int wf_load_csv(waveform_t *w, const char *path, uint32_t sample_rate);

int wf_save_vcd(const waveform_t *w, const char *path);

#endif /* WAVE_IO_H */
