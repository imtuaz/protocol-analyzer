/*
 * events.h - results produced by the decoders.
 *
 * Every decoder turns a waveform into a list of events, e.g.
 *   [START] [ADDR 0x50 W ACK] [DATA 0x12 ACK] [STOP]
 * Each event knows where it is in the waveform (start/end sample) and
 * carries a bit mask of detected errors.
 */
#ifndef EVENTS_H
#define EVENTS_H

#include <stdint.h>

typedef enum { PROTO_UART, PROTO_SPI, PROTO_I2C, PROTO_CAN } proto_t;

typedef enum {
    EV_DATA,      /* a data byte/word                          */
    EV_START,     /* I2C START or repeated START               */
    EV_STOP,      /* I2C STOP                                  */
    EV_ADDR,      /* I2C address byte                          */
    EV_BREAK,     /* UART break (line held low > 1 frame)      */
    EV_FRAME,     /* CAN complete frame                        */
    EV_TIMING,    /* timing measurement / violation            */
    EV_ERROR      /* error not attached to a valid data item   */
} ev_type_t;

/* Error flags (combine with |) */
#define ERR_PARITY      (1u << 0)  /* parity bit wrong                   */
#define ERR_FRAMING     (1u << 1)  /* stop bit not high                  */
#define ERR_GLITCH      (1u << 2)  /* start bit too short (noise)        */
#define ERR_NACK        (1u << 3)  /* I2C NACK / CAN no ACK              */
#define ERR_INCOMPLETE  (1u << 4)  /* transfer cut in the middle         */
#define ERR_TIMING      (1u << 5)  /* timing outside spec / tolerance    */
#define ERR_STUFF       (1u << 6)  /* CAN: 6 equal bits in a row         */
#define ERR_CRC         (1u << 7)  /* CAN: CRC mismatch                  */
#define ERR_FORM        (1u << 8)  /* fixed-format field has wrong value */
#define ERR_COUNT       9

#define EV_NOTE_LEN 72

typedef struct {
    proto_t   proto;
    ev_type_t type;
    uint32_t  start, end;     /* sample range in the waveform            */
    uint32_t  value;          /* byte / MOSI word / address / CAN ID     */
    uint32_t  value2;         /* SPI: MISO word                          */
    int       ack;            /* I2C: 1 ACK, 0 NACK, -1 not applicable    */
    int       rw;             /* I2C address: 0 write, 1 read             */
    uint8_t   data[8];        /* CAN payload                              */
    int       dlc;            /* CAN data length, -1 = header not decoded */
    uint32_t  errors;         /* ERR_* flags                              */
    uint32_t  seq;            /* creation order (keeps sort stable)       */
    char      note[EV_NOTE_LEN];
} event_t;

typedef struct {
    event_t *items;
    int      count;
    int      cap;
} event_list_t;

void     ev_init(event_list_t *l);
void     ev_free(event_list_t *l);
void     ev_clear(event_list_t *l);
event_t *ev_add(event_list_t *l, proto_t p, ev_type_t t, uint32_t start, uint32_t end);

/* Count events of type t (or all if t < 0) that have any error */
int ev_count_errors(const event_list_t *l);
int ev_count_type(const event_list_t *l, ev_type_t t);

const char *proto_name(proto_t p);
const char *ev_type_name(ev_type_t t);
const char *err_name(int bit_index);                 /* "PARITY", ... */
void        err_to_string(uint32_t errors, char *buf, int size);

/* Short label used on waveform annotations, e.g. "0x41 'A'", "0x50 W" */
void        ev_label(const event_t *e, char *buf, int size);

/* 1 if both lists decode to the same payload (timing events ignored) */
int         ev_same_payload(const event_list_t *a, const event_list_t *b);

/* Sort events by start time (stable) for display */
void        ev_sort_by_time(event_list_t *l);

#endif /* EVENTS_H */
