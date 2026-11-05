/*
 * can.h - Classical CAN (CAN 2.0A, 11-bit ID) generator and decoder.
 *
 * One channel = bus level after the transceiver (RX pin):
 *   0 = dominant, 1 = recessive (idle)
 *
 * Frame:  SOF | ID(11) | RTR | IDE | r0 | DLC(4) | DATA(0..8 bytes)
 *         | CRC(15) | CRC delim(1) | ACK slot | ACK delim(1) | EOF(7 x 1)
 *
 * Bit stuffing: from SOF to the end of CRC, after 5 equal bits the
 * transmitter inserts 1 opposite bit. 6 equal bits = stuff error.
 */
#ifndef CAN_H
#define CAN_H

#include "waveform.h"
#include "events.h"

typedef struct {
    uint16_t id;          /* 11-bit identifier            */
    int      rtr;         /* remote frame (no data)       */
    int      dlc;         /* 0..8                         */
    uint8_t  data[8];
} can_frame_t;

/* Fault injection */
#define CAN_FAULT_NO_ACK     (1 << 0)   /* nobody acknowledges            */
#define CAN_FAULT_FLIP_BIT   (1 << 1)   /* flip one data bit -> CRC error */
#define CAN_FAULT_NO_STUFF   (1 << 2)   /* omit 1st stuff bit -> stuff error */
#define CAN_FAULT_BAD_DELIM  (1 << 3)   /* CRC delimiter dominant -> form */

uint16_t can_crc15(const uint8_t *bits, int n);

int  can_wf_init(waveform_t *w, uint32_t sample_rate);
int  can_gen_idle(waveform_t *w, uint32_t bitrate, double bits);
int  can_gen_frame(waveform_t *w, uint32_t bitrate, const can_frame_t *f, int faults);

void can_decode(const waveform_t *w, int ch, uint32_t bitrate, event_list_t *out);

#endif /* CAN_H */
