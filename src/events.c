#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "events.h"

void ev_init(event_list_t *l)
{
    l->items = NULL;
    l->count = 0;
    l->cap   = 0;
}

void ev_free(event_list_t *l)
{
    free(l->items);
    ev_init(l);
}

void ev_clear(event_list_t *l)
{
    l->count = 0;
}

event_t *ev_add(event_list_t *l, proto_t p, ev_type_t t, uint32_t start, uint32_t end)
{
    if (l->count == l->cap) {
        int new_cap = l->cap ? l->cap * 2 : 64;
        event_t *n = realloc(l->items, (size_t)new_cap * sizeof *n);
        if (!n)
            return NULL;
        l->items = n;
        l->cap   = new_cap;
    }
    event_t *e = &l->items[l->count++];
    memset(e, 0, sizeof *e);
    e->proto = p;
    e->type  = t;
    e->start = start;
    e->end   = end;
    e->ack   = -1;
    e->rw    = -1;
    e->seq   = (uint32_t)(l->count - 1);
    return e;
}

int ev_count_errors(const event_list_t *l)
{
    int n = 0;
    for (int i = 0; i < l->count; i++)
        if (l->items[i].errors)
            n++;
    return n;
}

int ev_count_type(const event_list_t *l, ev_type_t t)
{
    int n = 0;
    for (int i = 0; i < l->count; i++)
        if (l->items[i].type == t)
            n++;
    return n;
}

const char *proto_name(proto_t p)
{
    switch (p) {
    case PROTO_UART: return "UART";
    case PROTO_SPI:  return "SPI";
    case PROTO_I2C:  return "I2C";
    case PROTO_CAN:  return "CAN";
    }
    return "?";
}

const char *ev_type_name(ev_type_t t)
{
    switch (t) {
    case EV_DATA:   return "DATA";
    case EV_START:  return "START";
    case EV_STOP:   return "STOP";
    case EV_ADDR:   return "ADDR";
    case EV_BREAK:  return "BREAK";
    case EV_FRAME:  return "FRAME";
    case EV_TIMING: return "TIMING";
    case EV_ERROR:  return "ERROR";
    }
    return "?";
}

const char *err_name(int bit_index)
{
    static const char *names[ERR_COUNT] = {
        "PARITY", "FRAMING", "GLITCH", "NACK", "INCOMPLETE",
        "TIMING", "STUFF", "CRC", "FORM"
    };
    if (bit_index < 0 || bit_index >= ERR_COUNT)
        return "?";
    return names[bit_index];
}

void err_to_string(uint32_t errors, char *buf, int size)
{
    int pos = 0;
    buf[0] = '\0';
    for (int b = 0; b < ERR_COUNT && pos < size; b++) {
        if (errors & (1u << b))
            pos += snprintf(buf + pos, (size_t)(size - pos), "%s%s",
                            pos ? "," : "", err_name(b));
    }
}

void ev_label(const event_t *e, char *buf, int size)
{
    switch (e->type) {
    case EV_DATA:
        if (e->proto == PROTO_SPI)
            snprintf(buf, (size_t)size, "%02X/%02X", e->value, e->value2);
        else
            snprintf(buf, (size_t)size, "%02X", e->value);
        break;
    case EV_ADDR:  snprintf(buf, (size_t)size, "%02X %s", e->value, e->rw ? "R" : "W"); break;
    case EV_START: snprintf(buf, (size_t)size, "%s", e->note[0] ? e->note : "S"); break;
    case EV_STOP:  snprintf(buf, (size_t)size, "P"); break;
    case EV_BREAK: snprintf(buf, (size_t)size, "BREAK"); break;
    case EV_FRAME:
        if (e->dlc < 0)
            snprintf(buf, (size_t)size, "ID ?");
        else
            snprintf(buf, (size_t)size, "ID %03X", e->value);
        break;
    default:       err_to_string(e->errors, buf, size); break;
    }
}

/* Compare decoded content only (ignores timing info events) */
int ev_same_payload(const event_list_t *a, const event_list_t *b)
{
    int i = 0, j = 0;
    for (;;) {
        while (i < a->count && a->items[i].type == EV_TIMING) i++;
        while (j < b->count && b->items[j].type == EV_TIMING) j++;
        if (i == a->count || j == b->count)
            return (i == a->count) && (j == b->count);
        const event_t *x = &a->items[i], *y = &b->items[j];
        if (x->type != y->type || x->value != y->value || x->value2 != y->value2
            || x->ack != y->ack || x->dlc != y->dlc
            || memcmp(x->data, y->data, sizeof x->data) != 0)
            return 0;
        i++;
        j++;
    }
}

static int cmp_event_time(const void *a, const void *b)
{
    const event_t *x = a, *y = b;
    if (x->start != y->start)
        return (x->start > y->start) - (x->start < y->start);
    return (x->seq > y->seq) - (x->seq < y->seq);
}

void ev_sort_by_time(event_list_t *l)
{
    qsort(l->items, (size_t)l->count, sizeof l->items[0], cmp_event_time);
}
