/*
 * firetv.h
 *
 * Minimal client for the Fire TV "remote" HTTPS API on port 8080 (the one the Fire TV
 * remote app uses; undocumented, reverse engineered by the community). Pairing shows a
 * PIN on the TV and yields a client token. HTTP is done by running the curl binary.
 */

#ifndef FIRETV_H
#define FIRETV_H

#include <stddef.h>

/* Starts the remote service (DIAL, port 8009); needed when port 8080 is closed. */
int firetv_wake(const char *host, char *err, size_t err_size);

/* Pairing step 1: makes the TV display a PIN. */
int firetv_pin_display(const char *host, const char *friendly_name, char *err, size_t err_size);

/* Pairing step 2: exchanges the PIN for a client token. */
int firetv_pin_verify(const char *host, const char *pin, char *token, size_t token_size,
                      char *err, size_t err_size);

/* Sends a key. action: dpad_up/down/left/right, select, home, back, menu, sleep (navigation)
 * or play, scan_forward, scan_back (media). */
int firetv_send(const char *host, const char *token, const char *action, char *err,
                size_t err_size);

/* Token file: ~/.config/arcx/firetv_<host>.token */
int firetv_token_load(const char *host, char *token, size_t token_size);
int firetv_token_save(const char *host, const char *token);

#endif
