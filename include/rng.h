/*
 * rng.h - tiny deterministic random generator (xorshift32).
 * Same seed -> same sequence, so every fuzz failure can be reproduced.
 */
#ifndef RNG_H
#define RNG_H

#include <stdint.h>

typedef struct { uint32_t s; } rng_t;

static inline void rng_seed(rng_t *r, uint32_t seed)
{
    r->s = seed ? seed : 0x12345678u;     /* state must never be 0 */
}

static inline uint32_t rng_next(rng_t *r)
{
    uint32_t x = r->s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    r->s = x;
    return x;
}

/* Uniform integer in [lo, hi] */
static inline uint32_t rng_range(rng_t *r, uint32_t lo, uint32_t hi)
{
    return lo + rng_next(r) % (hi - lo + 1);
}

#endif /* RNG_H */
