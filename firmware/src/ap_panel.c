/* SPDX-License-Identifier: GPL-3.0-only */
/* The autopilot panel mode, ported from the fm1_open AP panel build
 * (app/ap_panel.c, screen.c): a PC bridge (fm1_bridge.py) polls MSFS 2024
 * via SimConnect and streams one compact state SysEx every 100 ms
 * (F0 7D 41 <type> ... F7; usb.c's collector hands it over without F0/F7),
 * while the panel streams its keys as note on/off 48..70 and its knobs as
 * relative CC 20..23 on channel 1 back to the bridge.
 *
 * Everything here is additive. Entering the mode (hold HOME ~1 s) takes
 * over the input, the LEDs and the screen; leaving restores FMini-1's page
 * untouched -- fa_input / fa_draw / fa_leds just route here first and fall
 * through to the synth when the mode is off. */
#include "ap_panel.h"

struct ap_panel_state ap_state;
static uint8_t ap_on;
static uint32_t ap_frame_ms;                /* fm1_ms of the last bridge frame */
static uint32_t ap_keys_down;               /* the switch keys held (note key bits) */
static uint32_t ap_hold_t0;                 /* HOME's press edge */
static uint8_t ap_hold_done;                /* this press already toggled */
static uint16_t ap_popup;                   /* 0xffff none; 0..3 a knob, 12+i a switch */
static uint32_t ap_popup_until;

#define AP_SW_N 12u
/* the switches sit on 12 white keys of the 27 (the black keys keep their
 * printed letters clear for the AP button overlay). The keyboard is F3..G5,
 * so the whites are F G A B | C D E F G A B | C D E F G: taking the first
 * twelve gives F3 G3 A3 B3 C4 D4 E4 F4 G4 A4 B4 C5 and the bridge's
 * NOTE_EVENTS follow the same order (notes 48,50,52,54,55,57,59,60,62,64,66,67). */
static const uint8_t AP_KEYS[AP_SW_N] = {0, 2, 4, 6, 7, 9, 11, 12, 14, 16, 18, 19};
static const uint8_t AP_CC[4] = {20, 21, 22, 23};   /* HDG ALT SPD VS */
static const char *const AP_LBL[AP_SW_N] = {
    "AP", "FD", "AT", "NV", "VN", "HD", "AL", "VS", "SP", "LC", "PR", "MA"};
static const char *const AP_NAME[AP_SW_N] = {
    "AP", "FD", "A/T", "NAV", "VNAV", "HDG", "ALT", "VS", "SPD", "LOC", "APR", "MACH"};
static const char *const AP_KNB[4] = {"HDG", "ALT", "SPD", "VS"};

/* A320 FCU colours (RGB565): amber digits on black, white labels, like the FCU */
#define AP_C_BG  0x0000u
#define AP_C_NUM 0xFE60u            /* the big seven-segment values: amber yellow */
#define AP_C_ON  0xFE60u            /* a switch that is on */
#define AP_C_OFF 0x52AAu            /* a switch that is off / the dim labels */
#define AP_C_TXT 0xD69Au            /* the small white-grey text */
#define AP_C_OK  0x07E0u            /* BRIDGE */
#define AP_C_BAD 0xF800u            /* no bridge frames */

static uint32_t ap_unpack3(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 7 | (uint32_t)p[2] << 14;
}

static unsigned ap_online(void)
{
    return (uint32_t)(fm1_ms - ap_frame_ms) < 500u;
}

int ap_panel_frame(const uint8_t *m, uint32_t n)
{
    if (n == 15u && m[0] == 0u) {
        ap_state.hdg = (uint16_t)ap_unpack3(m + 1);
        ap_state.alt = (uint16_t)ap_unpack3(m + 4);
        ap_state.spd = (uint16_t)ap_unpack3(m + 7);
        ap_state.vs = (int16_t)ap_unpack3(m + 10);
        ap_state.flags = (uint16_t)(m[13] | (unsigned)m[14] << 7);
    } else if (n == 27u && m[0] == 1u) {
        ap_state.hdg = (uint16_t)ap_unpack3(m + 1);
        ap_state.alt = (uint16_t)ap_unpack3(m + 4);
        ap_state.spd = (uint16_t)ap_unpack3(m + 7);
        ap_state.vs = (int16_t)ap_unpack3(m + 10);
        ap_state.ias = (uint16_t)ap_unpack3(m + 13);
        ap_state.alt_act = (uint16_t)ap_unpack3(m + 16);
        ap_state.hdg_act = (uint16_t)ap_unpack3(m + 19);
        ap_state.vs_act = (int16_t)ap_unpack3(m + 22);
        ap_state.flags = (uint16_t)(m[25] | (unsigned)m[26] << 7);
        ap_state.has_actual = 1;
    } else
        return 0;
    ++ap_state.revision;
    ap_frame_ms = fm1_ms;
    return 1;
}

uint8_t ap_active(void)
{
    return ap_on;
}

static void ap_set_mode(uint8_t on)
{
    if (on == ap_on)
        return;
    ap_on = on;
    ui.force = 1;                           /* the screen switches with the mode */
}

/* HOME held ~1 s enters / leaves the mode (a short press stays HOME) */
void ap_mode_tick(uint32_t pressed)
{
    uint32_t b = panel.btn[B_HOME];
    if ((pressed >> b) & 1u) {
        ap_hold_t0 = fm1_ms;
        ap_hold_done = 0;
    } else if (((fm1_in.buttons >> b) & 1u) && !ap_hold_done &&
               fm1_ms - ap_hold_t0 >= 1000u) {
        ap_hold_done = 1;
        ap_set_mode(!ap_on);
    }
}

uint8_t ap_popup_on(void)
{
    return ap_popup != 0xFFFFu && (uint32_t)(fm1_ms - ap_popup_until) >= 0x80000000u;
}

/* K1..K4 stream relative CC 20..23 (holding OCT- x10) so the bridge echoes
 * every move; with no bridge frames for 500 ms the panel also edits its own
 * values locally, otherwise the simulator owns the numbers. */
void ap_input(void)
{
    uint32_t ne = fm1_input_note_edges(), k;
    unsigned online = ap_online();
    int scale = ((fm1_in.buttons >> panel.btn[B_OCTDN]) & 1u) ? 10 : 1;
    (void)panel_enc(EN_SELECT);             /* drained: no jump on leaving the mode */
    (void)panel_enc(EN_ALGO);
    (void)panel_enc(EN_PRESET);
    for (k = 0; k < 4u; k++) {
        int32_t d = panel_enc(EN_K1 + k) * scale, send;
        if (!d)
            continue;
        send = d < -63 ? -63 : d > 63 ? 63 : d;
        midi_out_event(0x0Bu | 0xB0u << 8 | (uint32_t)AP_CC[k] << 16 |
                       (uint32_t)(64 + send) << 24);
        if (!online) {
            if (k == 0) {
                int32_t h = (int32_t)ap_state.hdg + d;
                h %= 360;
                if (h < 0)
                    h += 360;
                ap_state.hdg = (uint16_t)h;
            } else if (k == 1) {
                int32_t a = (int32_t)ap_state.alt + d * 100;
                ap_state.alt = (uint16_t)(a < 0 ? 0 : a > 65000 ? 65000 : a);
            } else if (k == 2) {
                int32_t s = (int32_t)ap_state.spd + d;
                ap_state.spd = (uint16_t)(s < 0 ? 0 : s > 999 ? 999 : s);
            } else {
                int32_t w = (int32_t)ap_state.vs + d * 100;
                ap_state.vs = (int16_t)(w < -9900 ? -9900 : w > 9900 ? 9900 : w);
            }
            ++ap_state.revision;
        }
        ap_popup = (uint16_t)k;
        ap_popup_until = fm1_ms + 2000u;
    }
    /* the switches stream note on/off on channel 1; the bridge maps their
     * notes to simulator events. Offline, a press toggles the local flag so
     * the LED and the screen still respond. */
    for (k = 0; k < AP_SW_N; k++) {
        uint32_t key = AP_KEYS[k], bit = 1u << key;
        if ((ne & bit) && !(ap_keys_down & bit)) {
            midi_out_event(0x09u | 0x90u << 8 | (48u + key) << 16 | 100u << 24);
            ap_keys_down |= bit;
            ap_popup = (uint16_t)(AP_SW_N + k);
            ap_popup_until = fm1_ms + 2000u;
            if (!online) {
                ap_state.flags ^= (uint16_t)(1u << k);
                ++ap_state.revision;
            }
        } else if ((ap_keys_down & ~fm1_in.notes) & bit) {
            midi_out_event(0x08u | 0x80u << 8 | (48u + key) << 16);
            ap_keys_down &= ~bit;
        }
    }
}

/* the panel's LEDs: idle switches at half duty as a backlight for the
 * printed label, active switches full on (as the fm1_open panel did) */
void ap_leds_mirror(uint8_t *nl, uint8_t *nd)
{
    uint32_t k;
    for (k = 0; k < NB; k++)
        led_put(nd, panel.btn[k], 1);       /* the buttons' glow */
    led_put(nl, panel.btn[B_HOME], 1);      /* HOME: the way out */
    for (k = 0; k < AP_SW_N; k++) {
        uint32_t id = (uint32_t)LED_KEY0 + AP_KEYS[k];
        int on = (ap_state.flags >> k) & 1u;
        led_put(nl, id, on);
        led_put(nd, id, !on);
    }
}

/* ------------------------------------------------------------- screen --- */
static void ap_tx(int32_t x, int32_t ytop, const aafont_t *f, const char *s, uint16_t fg)
{
    cv_text_on(x, ytop - fa_cap_y(f), f, s, fg, AP_C_BG);
}
static void ap_tr(int32_t xr, int32_t ytop, const aafont_t *f, const char *s, uint16_t fg)
{
    cv_text_r(xr, ytop - fa_cap_y(f), f, s, fg, AP_C_BG);
}
static void ap_tc(int32_t cx, int32_t ytop, const aafont_t *f, const char *s, uint16_t fg)
{
    cv_text_on(cx - text_w(f, s) / 2, ytop - fa_cap_y(f), f, s, fg, AP_C_BG);
}
static void ap_num(char *b, int32_t v)
{
    char d[6];
    uint32_t n = 0;
    if (v < 0) {
        *b++ = '-';
        v = -v;
    }
    do {
        d[n++] = (char)('0' + v % 10);
        v /= 10;
    } while (v);
    while (n)
        *b++ = d[--n];
    *b = 0;
}

/* ---- the seven-segment digits (the A320 number-tube look) -------------- */
/* bars are plain cv_rect's: a/b/c/d/e/f/g in bits 0..6; the verticals overlap
 * the horizontals so the joints are solid. vh is one vertical bar's length. */
static const uint8_t AP_SEG[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

static int32_t ap_seg_count(const char *s)
{
    int32_t n = 0;
    while (*s)
        n++, s++;
    return n;
}

static void ap_seg_at(int32_t x, int32_t y, int32_t w, int32_t h, int32_t gap, const char *s, uint16_t c)
{
    /* the FCU's stroke is thin: about a ninth of the digit's height, and never
     * a third of its width (the 5-digit popup cells are much narrower) */
    int32_t t = h / 9u, t2 = w / 5u, vh, y2;
    if (t2 < t)
        t = t2;
    if (t < 2)
        t = 2;
    vh = (h - 3 * t) / 2;
    y2 = y + t + vh;
    for (; *s; s++, x += w + gap) {
        uint8_t sg = 0;
        if (*s == '-')
            sg = 0x40u;
        else if (*s >= '0' && *s <= '9')
            sg = AP_SEG[*s - '0'];
        if (!sg)
            continue;
        if (sg & 0x01u) cv_rect(x + t, y, w - 2 * t, t, c);              /* a  top */
        if (sg & 0x40u) cv_rect(x + t, y2, w - 2 * t, t, c);             /* g  middle */
        if (sg & 0x08u) cv_rect(x + t, y + h - t, w - 2 * t, t, c);      /* d  bottom */
        if (sg & 0x20u) cv_rect(x, y + t, t, vh, c);                     /* f  upper left */
        if (sg & 0x02u) cv_rect(x + w - t, y + t, t, vh, c);             /* b  upper right */
        if (sg & 0x10u) cv_rect(x, y2 + t, t, vh, c);                    /* e  lower left */
        if (sg & 0x04u) cv_rect(x + w - t, y2 + t, t, vh, c);            /* c  lower right */
    }
}

static int32_t ap_seg_w(int32_t n, int32_t w, int32_t gap)
{
    return n * (w + gap) - gap;
}

/* the four value rows: the label small and white at the left, the number in
 * seven-segment amber right-aligned at the screen's right edge */
static void ap_value_row(int32_t y, const char *label, int32_t v)
{
    char b[8];
    ap_tx(8, y + 10, &AF_S, label, AP_C_TXT);
    ap_num(b, v);
    ap_seg_at(234 - ap_seg_w(ap_seg_count(b), 16, 3), y, 16, 30, 3, b, AP_C_NUM);
}

static void ap_scene(void)                  /* absolute coordinates 0..239 */
{
    unsigned online = ap_online(), actual = online && ap_state.has_actual, i;
    if (ap_popup_on()) {                    /* the touched control, nearly the full screen */
        if (ap_popup < 4u) {
            int32_t values[4], n, w;
            char b[8];
            values[0] = ap_state.hdg;
            values[1] = ap_state.alt;
            values[2] = ap_state.spd;
            values[3] = ap_state.vs;
            ap_tc(120, 30, &AF_M, AP_KNB[ap_popup], AP_C_TXT);
            ap_num(b, values[ap_popup]);
            n = ap_seg_count(b);
            w = (224 - (n - 1) * 8) / n;    /* 5 chars still fit: 38 px wide each */
            ap_seg_at((240 - ap_seg_w(n, w, 8)) / 2, 72, w, 110, 8, b, AP_C_NUM);
        } else {
            unsigned on;
            i = ap_popup - AP_SW_N;
            on = (ap_state.flags >> i) & 1u;
            ap_tc(120, 74, &AF_L, AP_NAME[i], on ? AP_C_ON : AP_C_TXT);
            ap_tc(120, 118, &AF_L, on ? "ON" : "OFF", on ? AP_C_ON : AP_C_OFF);
        }
        return;
    }
    /* normal view: actual flight data at large size; targets only when no
     * bridge is feeding actuals (offline editing) */
    ap_tx(8, 4, &AF_S, "AP PANEL", AP_C_OFF);
    ap_tr(232, 4, &AF_S, online ? "BRIDGE" : "OFFLINE", online ? AP_C_OK : AP_C_BAD);
    ap_value_row(22, "HDG", actual ? ap_state.hdg_act : ap_state.hdg);
    ap_value_row(66, "ALT", actual ? ap_state.alt_act : ap_state.alt);
    ap_value_row(110, "SPD", actual ? ap_state.ias : ap_state.spd);
    ap_value_row(154, "VS", actual ? ap_state.vs_act : ap_state.vs);
    for (i = 0; i < AP_SW_N; i++)
        ap_tx(8 + (int32_t)(i % 6u) * 38, 198 + (i / 6u) * 14, &AF_X, AP_LBL[i],
              (ap_state.flags >> i) & 1u ? AP_C_ON : AP_C_OFF);
}

void ap_draw(void)
{
    uint32_t band;
    for (band = 0; band < 2u; band++) {
        cv_begin(240, 120, AP_C_BG);
        cv_oy = -(int32_t)band * 120;
        cv_cy0 = 0;
        ap_scene();
        cv_oy = 0;
        cv_blit(0, band * 120u);
    }
}
