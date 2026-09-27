/*
 * arcx_kasa_ctrl.c
 *
 * Controls a TP-Link Kasa bulb with an ArcX ring. The ring is reached through an
 * nRF52 dongle running Nordic's connectivity firmware, the bulb over the LAN.
 *
 *   Center   toggle the bulb on / off
 *   Forward  brightness up   (turns the bulb on if it is off)
 *   Backward brightness down (ignored while the bulb is off)
 *
 * Ring GATT (see README):
 *   Custom service         00001666-1212-efde-1523-785feabcd124
 *     Extended Key Event   00001669-...  (notify, ArcxRingState in byte 0)
 *   Battery service 0x180F / Battery Level 0x2A19 (notify)
 *
 * Key events arrive on the driver's thread; they are queued and the bulb is
 * driven from the main thread so BLE event handling never blocks on the network.
 *
 * Built against pc-ble-driver (SoftDevice API v5 or v6, see NRF_SD_BLE_API).
 *
 * Usage: arcx_kasa_ctrl [options] serial_port ring_name bulb_ip
 *   ring_name is matched as a prefix of the advertised name (case-insensitive).
 */

#include "ble.h"
#include "ble_hci.h"
#include "sd_rpc.h"

#include "kasa.h"

#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <limits.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define SERIAL_BY_ID_DIR       "/dev/serial/by-id"
#define DEFAULT_BAUD_RATE      1000000
#define DEFAULT_STEP           10 /* brightness change per press, percent */
#define DEFAULT_TRANSITION_MS  0

#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000) / (RESOLUTION))
#define UNIT_0_625_MS 625
#define UNIT_1_25_MS  1250
#define UNIT_10_MS    10000

#define SCAN_INTERVAL MSEC_TO_UNITS(100, UNIT_0_625_MS)
#define SCAN_WINDOW   MSEC_TO_UNITS(50, UNIT_0_625_MS)
#define SCAN_TIMEOUT  0 /* no timeout */

#define CONN_CFG_TAG 1

/* 00001666-1212-efde-1523-785feabcd124, little endian, 16-bit part zeroed. */
static const ble_uuid128_t ARCX_BASE_UUID = {
    {0x24, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
     0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}};

#define ARCX_UUID_SERVICE        0x1666
#define ARCX_UUID_EXT_KEY_EVENT  0x1669

#define UUID_BATTERY_LEVEL       0x2A19
#define UUID_CCCD                0x2902

/* From the firmware overview document, section 5. */
typedef enum {
    CENTER_PRESS = 0,
    CENTER_RELEASE,
    FORWARD_PRESS,
    FORWARD_RELEASE,
    BACKWARD_PRESS,
    BACKWARD_RELEASE,
    LEFT_PRESS,
    LEFT_RELEASE,
    RIGHT_PRESS,
    RIGHT_RELEASE,
    SOS
} ArcxRingState;

static const char *const ARCX_STATE_NAMES[] = {
    "CENTER_PRESS",   "CENTER_RELEASE", "FORWARD_PRESS", "FORWARD_RELEASE",
    "BACKWARD_PRESS", "BACKWARD_RELEASE", "LEFT_PRESS",  "LEFT_RELEASE",
    "RIGHT_PRESS",    "RIGHT_RELEASE",  "SOS"};

/* ---------------------------------------------------------------------------
 * State
 * ------------------------------------------------------------------------- */

#define MAX_CHARS 48
#define MAX_OPS   32

typedef struct
{
    ble_uuid_t uuid;
    uint16_t   handle_decl;
    uint16_t   handle_value;
    uint16_t   handle_cccd;
    bool       notify;
} gatt_char_t;

typedef enum { OP_DESC_DISC, OP_READ, OP_CCCD_ENABLE } op_type_t;

typedef struct
{
    op_type_t type;
    int       char_idx;
    uint16_t  start; /* descriptor discovery cursor */
} gatt_op_t;

static adapter_t  *m_adapter;
static const char *m_name_prefix;
static bool        m_verbose;
static bool        m_trace; /* -vv: driver packet trace */
static volatile sig_atomic_t m_quit;

/* Bulb settings and button presses queued by the BLE thread for the main thread. */
static const char     *m_bulb_host;
static int             m_step          = DEFAULT_STEP;
static int             m_transition_ms = DEFAULT_TRANSITION_MS;
static pthread_mutex_t m_press_lock    = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  m_press_cond    = PTHREAD_COND_INITIALIZER;
static int             m_press_toggles;
static int             m_press_steps; /* +1 per up press, -1 per down press */

static uint8_t  m_vs_uuid_type;
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;
static bool     m_connecting;
static bool     m_pairing;
static bool     m_pair_attempted;

static gatt_char_t m_chars[MAX_CHARS];
static int         m_char_count;

static gatt_op_t m_ops[MAX_OPS];
static int       m_op_head, m_op_tail;
static bool      m_op_in_flight;

#if NRF_SD_BLE_API >= 6
static uint8_t    m_scan_buffer_data[BLE_GAP_SCAN_BUFFER_MIN];
static ble_data_t m_scan_buffer = {m_scan_buffer_data, sizeof(m_scan_buffer_data)};
#endif

static const ble_gap_scan_params_t m_scan_params = {
#if NRF_SD_BLE_API >= 6
    .extended               = 0,
    .report_incomplete_evts = 0,
    .active                 = 1, /* name may be in the scan response */
    .filter_policy          = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .scan_phys              = BLE_GAP_PHY_1MBPS,
    .interval               = SCAN_INTERVAL,
    .window                 = SCAN_WINDOW,
    .timeout                = SCAN_TIMEOUT,
    .channel_mask           = {0},
#else
    .active         = 1,
    .use_whitelist  = 0,
    .adv_dir_report = 0,
    .interval       = SCAN_INTERVAL,
    .window         = SCAN_WINDOW,
    .timeout        = SCAN_TIMEOUT,
#endif
};

static const ble_gap_conn_params_t m_conn_params = {
    .min_conn_interval = MSEC_TO_UNITS(15, UNIT_1_25_MS),
    .max_conn_interval = MSEC_TO_UNITS(30, UNIT_1_25_MS),
    .slave_latency     = 0,
    .conn_sup_timeout  = MSEC_TO_UNITS(4000, UNIT_10_MS),
};

/* ---------------------------------------------------------------------------
 * Output helpers
 * ------------------------------------------------------------------------- */

static void log_ts(const char *tag, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/* Called from both the driver thread and the main thread. */
static void log_ts(const char *tag, const char *fmt, ...)
{
    struct timeval tv;
    struct tm      tm;
    va_list        ap;

    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &tm);
    flockfile(stdout);
    printf("[%02d:%02d:%02d.%03ld] %-6s ", tm.tm_hour, tm.tm_min, tm.tm_sec,
           (long)(tv.tv_usec / 1000), tag);
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    putchar('\n');
    fflush(stdout);
    funlockfile(stdout);
}

static void hex_string(const uint8_t *data, uint16_t len, char *out, size_t out_size)
{
    size_t pos = 0;
    out[0]     = '\0';
    for (uint16_t i = 0; i < len && pos + 4 < out_size; i++)
    {
        pos += (size_t)snprintf(out + pos, out_size - pos, i ? "-%02X" : "%02X", data[i]);
    }
}

static const char *char_label(const gatt_char_t *c)
{
    if (c->uuid.type == m_vs_uuid_type && c->uuid.uuid == ARCX_UUID_EXT_KEY_EVENT)
        return "Extended Key Event";
    if (c->uuid.type == BLE_UUID_TYPE_BLE && c->uuid.uuid == UUID_BATTERY_LEVEL)
        return "Battery Level";
    return NULL;
}

/* ---------------------------------------------------------------------------
 * Bulb control
 * ------------------------------------------------------------------------- */

/* Driver thread: record a press for the main thread. */
static void press_queue(int toggles, int steps)
{
    pthread_mutex_lock(&m_press_lock);
    m_press_toggles += toggles;
    m_press_steps += steps;
    pthread_cond_signal(&m_press_cond);
    pthread_mutex_unlock(&m_press_lock);
}

/* Main thread: waits up to 100 ms for presses. Presses made while the bulb was busy
 * are merged, so several quick up presses become one larger step. */
static bool press_take(int *toggles, int *steps)
{
    struct timespec deadline;

    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_nsec += 100 * 1000 * 1000;
    if (deadline.tv_nsec >= 1000000000L)
    {
        deadline.tv_sec++;
        deadline.tv_nsec -= 1000000000L;
    }

    pthread_mutex_lock(&m_press_lock);
    while (!m_press_toggles && !m_press_steps && !m_quit)
        if (pthread_cond_timedwait(&m_press_cond, &m_press_lock, &deadline) == ETIMEDOUT)
            break;
    *toggles        = m_press_toggles;
    *steps          = m_press_steps;
    m_press_toggles = 0;
    m_press_steps   = 0;
    pthread_mutex_unlock(&m_press_lock);
    return *toggles || *steps;
}

static void bulb_apply(int toggles, int steps)
{
    kasa_state_t st;
    char         err[160];
    bool         on;
    int          brightness;

    /* Read the current state each time so changes from the Kasa app are respected. */
    if (kasa_get_state(m_bulb_host, &st, err, sizeof(err)) != 0)
    {
        log_ts("ERROR", "Bulb: %s", err);
        return;
    }

    on         = st.on;
    brightness = st.brightness;
    if (toggles & 1)
        on = !on;

    if (steps && !st.dimmable)
    {
        log_ts("BULB", "%s is not dimmable, ignoring brightness change", st.alias);
        steps = 0;
    }
    else if (steps < 0 && !on)
    {
        log_ts("BULB", "Light is off, ignoring brightness down");
        steps = 0;
    }
    else if (steps)
    {
        brightness += steps * m_step;
        if (brightness > 100)
            brightness = 100;
        if (brightness < 1)
            brightness = 1;
        on = true;
    }

    if (on == st.on && (!steps || brightness == st.brightness))
    {
        if (steps)
            log_ts("BULB", "Brightness already at %d%%", brightness);
        return;
    }

    if (kasa_set_light(m_bulb_host, on, steps ? brightness : 0, m_transition_ms, err,
                       sizeof(err)) != 0)
    {
        log_ts("ERROR", "Bulb: %s", err);
        return;
    }
    if (on)
        log_ts("BULB", "%s on, brightness %d%%", st.alias, brightness);
    else
        log_ts("BULB", "%s off", st.alias);
}

/* ---------------------------------------------------------------------------
 * Scanning / connecting
 * ------------------------------------------------------------------------- */

static uint32_t scan_start(void)
{
    uint32_t err = sd_ble_gap_scan_start(m_adapter, &m_scan_params
#if NRF_SD_BLE_API >= 6
                                         , &m_scan_buffer
#endif
    );
    if (err != NRF_SUCCESS)
        log_ts("ERROR", "sd_ble_gap_scan_start failed: 0x%X", err);
    else
        log_ts("SCAN", "Scanning for devices named \"%s*\"...", m_name_prefix);
    return err;
}

#if NRF_SD_BLE_API >= 6
static void scan_continue(void)
{
    uint32_t err = sd_ble_gap_scan_start(m_adapter, NULL, &m_scan_buffer);
    if (err != NRF_SUCCESS && err != NRF_ERROR_INVALID_STATE)
        log_ts("ERROR", "scan continue failed: 0x%X", err);
}
#endif

/* Find the (complete or shortened) local name in an advertising payload. */
static bool adv_find_name(const uint8_t *data, uint16_t len, char *name, size_t name_size)
{
    uint16_t i = 0;
    while (i + 1 < len)
    {
        uint8_t field_len = data[i];
        uint8_t type      = data[i + 1];
        if (field_len == 0 || i + 1 + field_len > len)
            break;
        if (type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME || type == BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME)
        {
            size_t n = field_len - 1;
            if (n >= name_size)
                n = name_size - 1;
            memcpy(name, &data[i + 2], n);
            name[n] = '\0';
            return true;
        }
        i += field_len + 1;
    }
    return false;
}

/* Verbose mode: list each advertiser once (and again once its name is seen). */
static void log_adv_once(const ble_gap_evt_adv_report_t *report, const char *name)
{
    static struct { uint8_t addr[6]; bool named; } seen[64];
    static int seen_count;
    const uint8_t *a = report->peer_addr.addr;
    int i;

    for (i = 0; i < seen_count; i++)
        if (memcmp(seen[i].addr, a, 6) == 0)
            break;
    if (i < seen_count && (seen[i].named || !name))
        return;
    if (i == seen_count)
    {
        if (seen_count == 64)
            return;
        memcpy(seen[seen_count++].addr, a, 6);
    }
    seen[i].named = name != NULL;
    log_ts("ADV", "%02X:%02X:%02X:%02X:%02X:%02X  RSSI %4d dBm  %s", a[5], a[4], a[3], a[2],
           a[1], a[0], report->rssi, name ? name : "(no name)");
}

static void on_adv_report(const ble_gap_evt_adv_report_t *report)
{
    char name[32];

#if NRF_SD_BLE_API >= 6
    const uint8_t *data = report->data.p_data;
    uint16_t       len  = report->data.len;
#else
    const uint8_t *data = report->data;
    uint16_t       len  = report->dlen;
#endif

    bool has_name = adv_find_name(data, len, name, sizeof(name));
    if (m_verbose)
        log_adv_once(report, has_name ? name : NULL);

    if (!m_connecting && m_conn_handle == BLE_CONN_HANDLE_INVALID && has_name &&
        strncasecmp(name, m_name_prefix, strlen(m_name_prefix)) == 0)
    {
        const uint8_t *a = report->peer_addr.addr;
        log_ts("SCAN", "Found %s [%02X:%02X:%02X:%02X:%02X:%02X] RSSI %d dBm, connecting...",
               name, a[5], a[4], a[3], a[2], a[1], a[0], report->rssi);

        uint32_t err = sd_ble_gap_connect(m_adapter, &report->peer_addr, &m_scan_params,
                                          &m_conn_params, CONN_CFG_TAG);
        if (err == NRF_SUCCESS)
        {
            m_connecting = true;
            return;
        }
        log_ts("ERROR", "sd_ble_gap_connect failed: 0x%X", err);
    }

#if NRF_SD_BLE_API >= 6
    /* With API v6, scanning pauses after every report until resumed. */
    scan_continue();
#endif
}

/* ---------------------------------------------------------------------------
 * GATT operation queue (discovery -> reads -> enable notifications)
 * ------------------------------------------------------------------------- */

static void op_push(op_type_t type, int char_idx)
{
    if (m_op_tail >= MAX_OPS)
        return;
    m_ops[m_op_tail].type     = type;
    m_ops[m_op_tail].char_idx = char_idx;
    m_ops[m_op_tail].start    = m_chars[char_idx].handle_value + 1;
    m_op_tail++;
}

static uint16_t char_range_end(int idx)
{
    /* Characteristics were discovered in handle order. */
    return (idx + 1 < m_char_count) ? m_chars[idx + 1].handle_decl - 1 : 0xFFFF;
}

static void ops_run(void);

static uint32_t op_issue(gatt_op_t *op)
{
    gatt_char_t *c = &m_chars[op->char_idx];

    switch (op->type)
    {
        case OP_DESC_DISC: {
            ble_gattc_handle_range_t range = {op->start, char_range_end(op->char_idx)};
            if (range.start_handle > range.end_handle)
                return NRF_ERROR_NOT_FOUND;
            return sd_ble_gattc_descriptors_discover(m_adapter, m_conn_handle, &range);
        }
        case OP_READ:
            return sd_ble_gattc_read(m_adapter, m_conn_handle, c->handle_value, 0);

        case OP_CCCD_ENABLE: {
            uint8_t                  cccd[2] = {BLE_GATT_HVX_NOTIFICATION, 0};
            ble_gattc_write_params_t wp;
            memset(&wp, 0, sizeof(wp));
            wp.write_op = BLE_GATT_OP_WRITE_REQ;
            wp.handle   = c->handle_cccd;
            wp.len      = sizeof(cccd);
            wp.p_value  = cccd;
            return sd_ble_gattc_write(m_adapter, m_conn_handle, &wp);
        }
    }
    return NRF_ERROR_INVALID_PARAM;
}

/* Complete the in-flight op and start the next one. */
static void op_done(void)
{
    m_op_in_flight = false;
    m_op_head++;
    ops_run();
}

static void ops_run(void)
{
    while (!m_op_in_flight && m_op_head < m_op_tail && m_conn_handle != BLE_CONN_HANDLE_INVALID &&
           !m_pairing)
    {
        uint32_t err = op_issue(&m_ops[m_op_head]);
        if (err == NRF_SUCCESS)
        {
            m_op_in_flight = true;
            return;
        }
        if (err != NRF_ERROR_NOT_FOUND)
            log_ts("ERROR", "GATT op %d failed to start: 0x%X", m_ops[m_op_head].type, err);
        m_op_head++;
    }

    if (!m_op_in_flight && m_op_head == m_op_tail && m_op_tail > 0)
    {
        m_op_head = m_op_tail = 0;
        log_ts("READY", "Center: on/off, forward: brighter, backward: dimmer (Ctrl-C to quit)");
    }
}

static void pairing_start(void)
{
    ble_gap_sec_params_t sp;
    memset(&sp, 0, sizeof(sp));
    sp.bond          = 0;
    sp.mitm          = 0;
    sp.io_caps       = BLE_GAP_IO_CAPS_NONE;
    sp.min_key_size  = 7;
    sp.max_key_size  = 16;

    uint32_t err = sd_ble_gap_authenticate(m_adapter, m_conn_handle, &sp);
    m_pair_attempted = true;
    if (err == NRF_SUCCESS)
    {
        m_pairing = true;
        log_ts("SEC", "Pairing (Just Works)...");
    }
    else
    {
        log_ts("ERROR", "sd_ble_gap_authenticate failed: 0x%X", err);
    }
}

/* Returns true if the op should be retried once the link is encrypted. */
static bool needs_security(uint16_t gatt_status)
{
    if ((gatt_status == BLE_GATT_STATUS_ATTERR_INSUF_AUTHENTICATION ||
         gatt_status == BLE_GATT_STATUS_ATTERR_INSUF_ENCRYPTION) &&
        !m_pair_attempted)
    {
        m_op_in_flight = false; /* keep op at head, retry after pairing */
        if (!m_pairing)
            pairing_start();
        return true;
    }
    return false;
}

/* ---------------------------------------------------------------------------
 * Discovery
 * ------------------------------------------------------------------------- */

static void char_discovery(uint16_t start)
{
    ble_gattc_handle_range_t range = {start, 0xFFFF};
    uint32_t err = sd_ble_gattc_characteristics_discover(m_adapter, m_conn_handle, &range);
    if (err != NRF_SUCCESS)
        log_ts("ERROR", "characteristics discovery failed: 0x%X", err);
}

static void on_char_discovery_done(void)
{
    bool found_ext = false;

    log_ts("GATT", "Discovered %d characteristics", m_char_count);

    for (int i = 0; i < m_char_count; i++)
    {
        const char *label = char_label(&m_chars[i]);
        if (m_verbose)
            log_ts("GATT", "  decl 0x%04X value 0x%04X uuid 0x%04X (type %d)%s%s",
                   m_chars[i].handle_decl, m_chars[i].handle_value, m_chars[i].uuid.uuid,
                   m_chars[i].uuid.type, label ? " " : "", label ? label : "");
        if (!label)
            continue;
        if (m_chars[i].uuid.type == m_vs_uuid_type)
            found_ext = true;

        /* Read the battery level once. */
        if (m_chars[i].uuid.type == BLE_UUID_TYPE_BLE)
            op_push(OP_READ, i);
        if (m_chars[i].notify)
            op_push(OP_DESC_DISC, i);
    }

    if (!found_ext)
        log_ts("WARN", "Extended Key Event characteristic (0x1669) not found on this device");

    ops_run();
}

static void on_char_disc_rsp(const ble_gattc_evt_t *evt)
{
    const ble_gattc_evt_char_disc_rsp_t *rsp = &evt->params.char_disc_rsp;

    if (evt->gatt_status == BLE_GATT_STATUS_ATTERR_ATTRIBUTE_NOT_FOUND || rsp->count == 0)
    {
        on_char_discovery_done();
        return;
    }
    if (evt->gatt_status != BLE_GATT_STATUS_SUCCESS)
    {
        log_ts("ERROR", "characteristic discovery status 0x%X", evt->gatt_status);
        on_char_discovery_done();
        return;
    }

    uint16_t last = 0;
    for (int i = 0; i < rsp->count && m_char_count < MAX_CHARS; i++)
    {
        gatt_char_t *c  = &m_chars[m_char_count++];
        c->uuid         = rsp->chars[i].uuid;
        c->handle_decl  = rsp->chars[i].handle_decl;
        c->handle_value = rsp->chars[i].handle_value;
        c->handle_cccd  = 0;
        c->notify       = rsp->chars[i].char_props.notify || rsp->chars[i].char_props.indicate;
        last            = rsp->chars[i].handle_value;
    }

    if (last < 0xFFFF && m_char_count < MAX_CHARS)
        char_discovery(last + 1);
    else
        on_char_discovery_done();
}

static void on_desc_disc_rsp(const ble_gattc_evt_t *evt)
{
    gatt_op_t   *op = &m_ops[m_op_head];
    gatt_char_t *c  = &m_chars[op->char_idx];
    const ble_gattc_evt_desc_disc_rsp_t *rsp = &evt->params.desc_disc_rsp;

    if (evt->gatt_status == BLE_GATT_STATUS_SUCCESS)
    {
        uint16_t last = 0;
        for (int i = 0; i < rsp->count; i++)
        {
            last = rsp->descs[i].handle;
            if (rsp->descs[i].uuid.type == BLE_UUID_TYPE_BLE && rsp->descs[i].uuid.uuid == UUID_CCCD)
            {
                c->handle_cccd = rsp->descs[i].handle;
                break;
            }
        }
        if (c->handle_cccd == 0 && last != 0 && last < char_range_end(op->char_idx))
        {
            /* Continue discovering in the rest of this characteristic's range. */
            op->start      = last + 1;
            m_op_in_flight = false;
            ops_run();
            return;
        }
    }

    if (c->handle_cccd)
        op_push(OP_CCCD_ENABLE, op->char_idx);
    else
        log_ts("WARN", "No CCCD found for %s", char_label(c));
    op_done();
}

static void on_read_rsp(const ble_gattc_evt_t *evt)
{
    gatt_char_t *c = &m_chars[m_ops[m_op_head].char_idx];

    if (needs_security(evt->gatt_status))
        return;

    if (evt->gatt_status == BLE_GATT_STATUS_SUCCESS)
    {
        const ble_gattc_evt_read_rsp_t *r = &evt->params.read_rsp;
        if (c->uuid.uuid == UUID_BATTERY_LEVEL && r->len >= 1)
            log_ts("BATT", "Battery level: %u%%", r->data[0]);
    }
    else
    {
        log_ts("WARN", "Read of %s failed, status 0x%X", char_label(c), evt->gatt_status);
    }
    op_done();
}

static void on_write_rsp(const ble_gattc_evt_t *evt)
{
    gatt_char_t *c = &m_chars[m_ops[m_op_head].char_idx];

    if (needs_security(evt->gatt_status))
        return;

    if (evt->gatt_status == BLE_GATT_STATUS_SUCCESS)
        log_ts("GATT", "Notifications enabled: %s", char_label(c));
    else
        log_ts("ERROR", "Enabling notifications on %s failed, status 0x%X", char_label(c),
               evt->gatt_status);
    op_done();
}

/* ---------------------------------------------------------------------------
 * Notifications -> bulb actions
 * ------------------------------------------------------------------------- */

static void on_key_event(const uint8_t *data, uint16_t len)
{
    char raw[64];
    hex_string(data, len, raw, sizeof(raw));

    if (len == 0 || data[0] > SOS)
    {
        log_ts("KEY", "Unknown key event [%s]", raw);
        return;
    }
    if (m_verbose)
        log_ts("KEY", "%-17s (%2u)    [%s]", ARCX_STATE_NAMES[data[0]], data[0], raw);

    /* Act on presses; releases and the other buttons are ignored. */
    switch (data[0])
    {
        case CENTER_PRESS:   press_queue(1, 0);  break;
        case FORWARD_PRESS:  press_queue(0, 1);  break;
        case BACKWARD_PRESS: press_queue(0, -1); break;
        default: break;
    }
}

static void on_hvx(const ble_gattc_evt_t *evt)
{
    const ble_gattc_evt_hvx_t *hvx = &evt->params.hvx;

    if (hvx->type == BLE_GATT_HVX_INDICATION)
        sd_ble_gattc_hv_confirm(m_adapter, m_conn_handle, hvx->handle);

    for (int i = 0; i < m_char_count; i++)
    {
        gatt_char_t *c = &m_chars[i];
        if (c->handle_value != hvx->handle)
            continue;

        if (c->uuid.type == m_vs_uuid_type && c->uuid.uuid == ARCX_UUID_EXT_KEY_EVENT)
            on_key_event(hvx->data, hvx->len);
        else if (c->uuid.type == BLE_UUID_TYPE_BLE && c->uuid.uuid == UUID_BATTERY_LEVEL && hvx->len)
            log_ts("BATT", "Battery level: %u%%", hvx->data[0]);
        else
            break;
        return;
    }

    if (m_verbose)
    {
        char raw[64];
        hex_string(hvx->data, hvx->len, raw, sizeof(raw));
        log_ts("NOTIFY", "handle 0x%04X: %s", hvx->handle, raw);
    }
}

/* ---------------------------------------------------------------------------
 * Connection lifecycle
 * ------------------------------------------------------------------------- */

static void reset_connection_state(void)
{
    m_conn_handle  = BLE_CONN_HANDLE_INVALID;
    m_connecting   = false;
    m_pairing      = false;
    m_pair_attempted = false;
    m_char_count   = 0;
    m_op_head      = 0;
    m_op_tail      = 0;
    m_op_in_flight = false;
}

static void ble_evt_dispatch(adapter_t *adapter, ble_evt_t *p_ble_evt)
{
    (void)adapter;
    if (!p_ble_evt)
        return;

    const ble_gap_evt_t   *gap   = &p_ble_evt->evt.gap_evt;
    const ble_gattc_evt_t *gattc = &p_ble_evt->evt.gattc_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
            on_adv_report(&gap->params.adv_report);
            break;

        case BLE_GAP_EVT_CONNECTED:
            m_connecting  = false;
            m_conn_handle = gap->conn_handle;
            log_ts("CONN", "Connected (handle %u), discovering services...", m_conn_handle);
            char_discovery(0x0001);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            log_ts("CONN", "Disconnected, reason 0x%02X", gap->params.disconnected.reason);
            reset_connection_state();
            if (!m_quit)
                scan_start();
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (gap->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                log_ts("CONN", "Connection attempt timed out");
                m_connecting = false;
            }
            scan_start();
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            sd_ble_gap_conn_param_update(m_adapter, gap->conn_handle,
                                         &gap->params.conn_param_update_request.conn_params);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
            ble_gap_phys_t phys = {BLE_GAP_PHY_AUTO, BLE_GAP_PHY_AUTO};
            sd_ble_gap_phy_update(m_adapter, gap->conn_handle, &phys);
            break;
        }

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
            sd_ble_gap_data_length_update(m_adapter, gap->conn_handle, NULL, NULL);
            break;

        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            sd_ble_gatts_exchange_mtu_reply(m_adapter, p_ble_evt->evt.gatts_evt.conn_handle,
                                            BLE_GATT_ATT_MTU_DEFAULT);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            sd_ble_gatts_sys_attr_set(m_adapter, p_ble_evt->evt.gatts_evt.conn_handle, NULL, 0, 0);
            break;

        case BLE_GAP_EVT_SEC_REQUEST:
            /* Peripheral asks for security. */
            if (!m_pairing)
                pairing_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST: {
            ble_gap_sec_keyset_t keyset;
            memset(&keyset, 0, sizeof(keyset));
            sd_ble_gap_sec_params_reply(m_adapter, gap->conn_handle, BLE_GAP_SEC_STATUS_SUCCESS,
                                        NULL, &keyset);
            break;
        }

        case BLE_GAP_EVT_AUTH_STATUS:
            m_pairing = false;
            if (gap->params.auth_status.auth_status == BLE_GAP_SEC_STATUS_SUCCESS)
                log_ts("SEC", "Link encrypted");
            else
                log_ts("SEC", "Pairing failed, status 0x%02X", gap->params.auth_status.auth_status);
            ops_run(); /* retry whatever was waiting on security */
            break;

        case BLE_GAP_EVT_CONN_SEC_UPDATE:
            if (m_verbose)
                log_ts("SEC", "Security level %u",
                       gap->params.conn_sec_update.conn_sec.sec_mode.lv);
            break;

        case BLE_GATTC_EVT_CHAR_DISC_RSP:
            on_char_disc_rsp(gattc);
            break;

        case BLE_GATTC_EVT_DESC_DISC_RSP:
            on_desc_disc_rsp(gattc);
            break;

        case BLE_GATTC_EVT_READ_RSP:
            on_read_rsp(gattc);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_rsp(gattc);
            break;

        case BLE_GATTC_EVT_HVX:
            on_hvx(gattc);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            log_ts("ERROR", "GATT timeout, disconnecting");
            sd_ble_gap_disconnect(m_adapter, gattc->conn_handle,
                                  BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;

        default:
            if (m_verbose)
                log_ts("EVT", "Unhandled event 0x%X", p_ble_evt->header.evt_id);
            break;
    }
}

/* ---------------------------------------------------------------------------
 * Driver / stack setup
 * ------------------------------------------------------------------------- */

static void status_handler(adapter_t *adapter, sd_rpc_app_status_t code, const char *message)
{
    (void)adapter;
    log_ts("STATUS", "%d: %s", (int)code, message);
}

static void log_handler(adapter_t *adapter, sd_rpc_log_severity_t severity, const char *message)
{
    (void)adapter;
    if (severity >= SD_RPC_LOG_WARNING || m_verbose)
        log_ts("DRIVER", "%s", message);
}

static adapter_t *adapter_init(const char *port, uint32_t baud)
{
    physical_layer_t *phy = sd_rpc_physical_layer_create_uart(port, baud, SD_RPC_FLOW_CONTROL_NONE,
                                                              SD_RPC_PARITY_NONE);
    data_link_layer_t *dl = sd_rpc_data_link_layer_create_bt_three_wire(phy, 250);
    transport_layer_t *tl = sd_rpc_transport_layer_create(dl, 1500);
    return sd_rpc_adapter_create(tl);
}

static uint32_t ble_stack_init(void)
{
    ble_cfg_t cfg;
    uint32_t  err;

    memset(&cfg, 0, sizeof(cfg));
#if NRF_SD_BLE_API >= 6
    cfg.gap_cfg.role_count_cfg.adv_set_count = BLE_GAP_ADV_SET_COUNT_DEFAULT;
#endif
    cfg.gap_cfg.role_count_cfg.periph_role_count  = 0;
    cfg.gap_cfg.role_count_cfg.central_role_count = 1;
    cfg.gap_cfg.role_count_cfg.central_sec_count  = 1;
    err = sd_ble_cfg_set(m_adapter, BLE_GAP_CFG_ROLE_COUNT, &cfg, 0);
    if (err != NRF_SUCCESS)
    {
        log_ts("ERROR", "sd_ble_cfg_set(ROLE_COUNT) failed: 0x%X", err);
        return err;
    }

    memset(&cfg, 0, sizeof(cfg));
    cfg.conn_cfg.conn_cfg_tag                 = CONN_CFG_TAG;
    cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = BLE_GATT_ATT_MTU_DEFAULT;
    err = sd_ble_cfg_set(m_adapter, BLE_CONN_CFG_GATT, &cfg, 0);
    if (err != NRF_SUCCESS)
    {
        log_ts("ERROR", "sd_ble_cfg_set(CONN_CFG_GATT) failed: 0x%X", err);
        return err;
    }

    err = sd_ble_enable(m_adapter, NULL);
    if (err != NRF_SUCCESS && err != NRF_ERROR_INVALID_STATE)
    {
        log_ts("ERROR", "sd_ble_enable failed: 0x%X", err);
        return err;
    }

    err = sd_ble_uuid_vs_add(m_adapter, &ARCX_BASE_UUID, &m_vs_uuid_type);
    if (err != NRF_SUCCESS)
    {
        log_ts("ERROR", "sd_ble_uuid_vs_add failed: 0x%X", err);
        return err;
    }
    return NRF_SUCCESS;
}

/* ---------------------------------------------------------------------------
 * Dongle port handling
 * ------------------------------------------------------------------------- */

/* Map the requested port to a stable /dev/serial/by-id path if one exists. */
static void resolve_port(const char *requested, char *out, size_t out_size)
{
    char           want[PATH_MAX];
    DIR           *dir;
    struct dirent *de;

    snprintf(out, out_size, "%s", requested);
    if (!realpath(requested, want) || !(dir = opendir(SERIAL_BY_ID_DIR)))
        return;
    while ((de = readdir(dir)))
    {
        char link[PATH_MAX], target[PATH_MAX];
        snprintf(link, sizeof(link), "%s/%s", SERIAL_BY_ID_DIR, de->d_name);
        if (de->d_name[0] == '.' || !realpath(link, target))
            continue;
        if (strcmp(target, want) == 0)
        {
            snprintf(out, out_size, "%s", link);
            break;
        }
    }
    closedir(dir);
}

static adapter_t *adapter_open(const char *port, uint32_t baud)
{
    adapter_t *adapter = adapter_init(port, baud);

    sd_rpc_log_handler_severity_filter_set(adapter, m_trace ? SD_RPC_LOG_TRACE : SD_RPC_LOG_INFO);
    if (sd_rpc_open(adapter, status_handler, ble_evt_dispatch, log_handler) != NRF_SUCCESS)
    {
        sd_rpc_adapter_delete(adapter);
        return NULL;
    }
    return adapter;
}

static void on_signal(int sig)
{
    (void)sig;
    m_quit = 1;
}

static void usage(const char *prog)
{
    fprintf(stderr,
            "Usage: %s [-b baud] [-s step] [-t ms] [-v] serial_port ring_name bulb_ip\n"
            "  serial_port  serial port of the nRF52 connectivity dongle, e.g. /dev/ttyACM0\n"
            "  ring_name    advertised name of the ring, or a prefix of it, e.g. arcx-e24e\n"
            "  bulb_ip      IP address (or host name) of the Kasa bulb\n"
            "  -b  baud rate (default %d)\n"
            "  -s  brightness step per press in percent (default %d)\n"
            "  -t  fade time in ms for each change (default %d)\n"
            "  -v  verbose output (key events, advertisers, GATT table); -vv adds driver trace\n"
            "Ring buttons: center toggles the bulb, forward/backward raise/lower brightness.\n",
            prog, DEFAULT_BAUD_RATE, DEFAULT_STEP, DEFAULT_TRANSITION_MS);
}

int main(int argc, char *argv[])
{
    char         port[PATH_MAX];
    char         err[160];
    uint32_t     baud = DEFAULT_BAUD_RATE;
    kasa_state_t st;
    int          opt, toggles, steps;

    while ((opt = getopt(argc, argv, "b:s:t:vh")) != -1)
    {
        switch (opt)
        {
            case 'b': baud = (uint32_t)strtoul(optarg, NULL, 10); break;
            case 's': m_step = atoi(optarg); break;
            case 't': m_transition_ms = atoi(optarg); break;
            case 'v': m_trace = m_verbose; m_verbose = true; break;
            default: usage(argv[0]); return 1;
        }
    }
    if (argc - optind != 3 || m_step < 1 || m_step > 100 || m_transition_ms < 0)
    {
        usage(argv[0]);
        return 1;
    }
    m_name_prefix = argv[optind + 1];
    m_bulb_host   = argv[optind + 2];

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    /* Check the bulb first so a wrong address fails fast. */
    if (kasa_get_state(m_bulb_host, &st, err, sizeof(err)) != 0)
    {
        log_ts("ERROR", "Bulb: %s", err);
        return 1;
    }
    log_ts("BULB", "%s (%s) at %s is %s, brightness %d%%%s", st.alias, st.model, m_bulb_host,
           st.on ? "on" : "off", st.brightness, st.dimmable ? "" : " (not dimmable)");

    resolve_port(argv[optind], port, sizeof(port));
    log_ts("INIT", "Port %s @ %u baud, SoftDevice API v%d", port, baud, NRF_SD_BLE_API);

    m_adapter = adapter_open(port, baud);
    if (!m_adapter)
    {
        log_ts("ERROR", "Could not open the dongle on %s (is it flashed with connectivity "
                        "firmware for API v%d?)", port, NRF_SD_BLE_API);
        return 1;
    }

    if (ble_stack_init() != NRF_SUCCESS || scan_start() != NRF_SUCCESS)
    {
        sd_rpc_close(m_adapter);
        return 1;
    }

    while (!m_quit)
        if (press_take(&toggles, &steps))
            bulb_apply(toggles, steps);

    log_ts("EXIT", "Shutting down");
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        sd_ble_gap_disconnect(m_adapter, m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        usleep(300 * 1000);
    }
    sd_rpc_close(m_adapter);
    return 0;
}
