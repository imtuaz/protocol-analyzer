/*
 * i2c.h - I2C generator, decoder and timing checker.
 *
 * Channels: 0 = SCL, 1 = SDA (both idle high, open-drain bus)
 *   START : SDA falls while SCL is high
 *   STOP  : SDA rises while SCL is high
 *   bit   : SDA is read on the SCL rising edge
 *   byte  : 8 bits MSB first + 1 ACK bit (0 = ACK, 1 = NACK)
 *   1st byte after START = 7-bit address + R/W bit
 */
#ifndef I2C_H
#define I2C_H

#include "waveform.h"
#include "events.h"

#define I2C_CH_SCL 0
#define I2C_CH_SDA 1

typedef enum { I2C_STANDARD, I2C_FAST } i2c_speed_t;

/* Timing parameters in seconds (I2C spec UM10204 names) */
typedef struct {
    double t_low;      /* SCL low period                              */
    double t_high;     /* SCL high period                             */
    double t_hd_sta;   /* START hold: SDA fall -> SCL fall            */
    double t_su_sta;   /* repeated START setup: SCL rise -> SDA fall  */
    double t_su_dat;   /* data setup: SDA change -> SCL rise          */
    double t_su_sto;   /* STOP setup: SCL rise -> SDA rise            */
    double t_buf;      /* bus free between STOP and next START        */
} i2c_timing_t;

#define I2C_NUM_TIMING 7

/* Minimum values from the specification */
void i2c_timing_spec(i2c_timing_t *t, i2c_speed_t speed);
/* Spec-compliant timing for a given SCL frequency (used by generator) */
void i2c_timing_for_freq(i2c_timing_t *t, double scl_hz);
const char *i2c_timing_name(int index);

/* --- generator ------------------------------------------------------- */
int i2c_wf_init(waveform_t *w, uint32_t sample_rate);
int i2c_gen_start(waveform_t *w, const i2c_timing_t *t);
int i2c_gen_restart(waveform_t *w, const i2c_timing_t *t);
int i2c_gen_stop(waveform_t *w, const i2c_timing_t *t);
int i2c_gen_byte(waveform_t *w, const i2c_timing_t *t, uint8_t byte, int ack);

/* START, addr+W, data..., STOP. The slave ACKs every byte except the
 * byte at index nack_at (0 = address, 1.. = data), -1 = all ACKed.    */
int i2c_gen_write(waveform_t *w, const i2c_timing_t *t, uint8_t addr,
                  const uint8_t *data, int n, int nack_at);

/* Typical sensor register read:
 * START, addr+W, reg, Sr, addr+R, n bytes (master NACKs last), STOP  */
int i2c_gen_read_reg(waveform_t *w, const i2c_timing_t *t, uint8_t addr,
                     uint8_t reg, const uint8_t *data, int n);

/* --- decoder --------------------------------------------------------- */
/* Decodes transactions and checks every timing parameter against the
 * spec for `speed`. The worst value of each violated parameter is
 * reported as one EV_TIMING event. */
void i2c_decode(const waveform_t *w, i2c_speed_t speed, event_list_t *out);

#endif /* I2C_H */
