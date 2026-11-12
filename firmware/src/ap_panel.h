/* SPDX-License-Identifier: GPL-3.0-only */
/* Autopilot panel mode: shared declarations. The definitions live in
 * ap_panel.c, which felucca.c includes after fmini_app.c (this header is
 * included before editor.c so the SysEx frame service can hand over the
 * bridge's frames). */
#ifndef FM1_AP_PANEL_H
#define FM1_AP_PANEL_H

/* Autopilot panel state, filled by the PC bridge over custom SysEx. */
struct ap_panel_state {
    uint16_t hdg;       /* heading bug, degrees 0..359 */
    uint16_t alt;       /* selected altitude, feet */
    uint16_t spd;       /* selected speed, knots */
    int16_t vs;         /* selected vertical speed, fpm (signed) */
    uint16_t ias;       /* indicated airspeed, knots (frame v1) */
    uint16_t alt_act;   /* indicated altitude, feet (frame v1) */
    uint16_t hdg_act;   /* magnetic heading, degrees (frame v1) */
    int16_t vs_act;     /* vertical speed, fpm, signed (frame v1) */
    uint16_t has_actual;/* actuals valid since the last type-1 frame */
    uint16_t flags;     /* switch feedback, bit 0..11, see ap_panel.c */
    uint32_t revision;  /* bumped on every accepted state frame */
};

extern struct ap_panel_state ap_state;

/* One bridge frame: the type byte + 7-bit data, F0/F7 already stripped by
 * usb.c's collector. Type 0: 14 data bytes (hdg alt spd vs flags), type 1:
 * 26 (ias alt_act hdg_act vs_act added before flags). Returns 1 when the
 * frame was ours. */
static int ap_panel_frame(const uint8_t *m, uint32_t n);

static uint8_t ap_active(void);              /* the AP panel mode owns the panel */
static void ap_mode_tick(uint32_t pressed);  /* HOME held ~1 s toggles the mode */
static void ap_input(void);                  /* the mode's knobs and keys */
static void ap_leds_mirror(uint8_t *nl, uint8_t *nd);
static void ap_draw(void);                   /* the mode's screen (two 240x120 bands) */
static uint8_t ap_popup_on(void);            /* a touched control is shown big */

#endif
