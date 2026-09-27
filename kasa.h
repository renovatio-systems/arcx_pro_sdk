/*
 * kasa.h
 *
 * Minimal client for TP-Link Kasa smart bulbs using the local TCP protocol on
 * port 9999: a 4-byte big-endian length followed by the JSON request, XOR
 * "autokey" encrypted with an initial key of 171.
 */

#ifndef KASA_H
#define KASA_H

#include <stdbool.h>
#include <stddef.h>

#define KASA_PORT "9999"

typedef struct
{
    char alias[64];
    char model[32];
    bool dimmable;
    bool on;
    int  brightness; /* 1..100; when off, the brightness the bulb turns on at */
} kasa_state_t;

/* Reads the bulb's state with system.get_sysinfo. Returns 0 on success, -1 on error
 * (message in err). */
int kasa_get_state(const char *host, kasa_state_t *state, char *err, size_t err_size);

/* Sets on/off and, if brightness > 0, brightness (1..100). transition_ms is the fade
 * time. Returns 0 on success, -1 on error. */
int kasa_set_light(const char *host, bool on, int brightness, int transition_ms, char *err,
                   size_t err_size);

#endif
