#include "table_walk.h"
#include <string.h>
#include <math.h>

/* ---- Waveform tables, from TinyMathWalk.xlsx "20 Frames" sheet ------ */
/* Extracted directly from the spreadsheet (rows 6-9), values confirmed
 * to 4 decimal places against the source file. */

static const float TABLE_SIN[TW_NUM_FRAMES] = {
    0.0000f,  0.3090f,  0.5878f,  0.8090f,  0.9511f,  1.0000f,  0.9511f,
    0.8090f,  0.5878f,  0.3090f,  0.0000f, -0.3090f, -0.5878f, -0.8090f,
   -0.9511f, -1.0000f, -0.9511f, -0.8090f, -0.5878f, -0.3090f
};

static const float TABLE_SIN3[TW_NUM_FRAMES] = {
    0.0000f,  0.0295f,  0.2031f,  0.5295f,  0.8602f,  1.0000f,  0.8602f,
    0.5295f,  0.2031f,  0.0295f,  0.0000f, -0.0295f, -0.2031f, -0.5295f,
   -0.8602f, -1.0000f, -0.8602f, -0.5295f, -0.2031f, -0.0295f
};

static const float TABLE_PLOT1[TW_NUM_FRAMES] = {
    0.70f,  0.85f,  0.95f,  0.90f,  0.60f,  0.10f, -0.40f, -0.80f, -0.95f,
   -0.95f, -0.80f, -0.65f, -0.50f, -0.35f, -0.20f, -0.05f,  0.10f,  0.25f,
    0.40f,  0.55f
};

static const float TABLE_PLOT2[TW_NUM_FRAMES] = {
   -0.80f, -0.65f, -0.50f, -0.35f, -0.20f, -0.05f,  0.10f,  0.25f,  0.40f,
    0.55f,  0.70f,  0.85f,  0.95f,  0.90f,  0.60f,  0.10f, -0.40f, -0.80f,
   -0.95f, -0.95f
};

/* ---- Per-servo sign/gain matrices (kazz's LinkDef* arrays) ----------- */
/* Index order documented in table_walk.h. */

static const int LINKDEF_LEN[TW_NUM_SERVOS] = {
     2,  4,  2, -1,   /* left foot (thigh, knee, ankle) + R shoulder */
    -2, -4, -2,  1,   /* right foot (thigh, knee, ankle) + L shoulder */
     0,  0,  0,  0,   /* rolls: unaffected by LEN */
     0,  0,           /* yaws: unaffected by LEN */
     0,  0,  0        /* head: unaffected by LEN */
};

static const int LINKDEF_ROLL[TW_NUM_SERVOS] = {
     0,  0,  0,  0,
     0,  0,  0,  0,
     2,  2, -2, -2,   /* R hip roll, L hip roll, R ankle roll, L ankle roll */
     0,  0,
     0,  0,  0
};

static const int LINKDEF_PITCH[TW_NUM_SERVOS] = {
    -1,  0,  1,  1,   /* left foot pitch group + R shoulder */
     1,  0, -1, -1,   /* right foot pitch group + L shoulder */
     0,  0,  0,  0,
     0,  0,
     0,  0,  0
};

/* .pma-style servo ID for each ServoVal[] index -- matches kazz's
 * ServoId[] exactly, cross-verified against the community .pma servo
 * map (see table_walk.h header comment). */
static const uint8_t SERVO_PMA_ID[TW_NUM_SERVOS] = {
    0x10, 0x14, 0x18, 0x02,
    0x0E, 0x12, 0x16, 0x04,
    0x0A, 0x0C, 0x1A, 0x1C,
    0x08, 0x06,
    0x05, 0x07, 0x03
};

uint8_t table_walk_servo_pma_id(int index) {
    if (index < 0 || index >= TW_NUM_SERVOS) return 0;
    return SERVO_PMA_ID[index];
}

void tw_params_init(tw_params_t *p) {
    p->dura_len   = 140;
    p->dura_roll  = 80;
    p->dura_pitch = 600;
    p->dura_squat = 200;
}

void tw_walker_init(tw_walker_t *w) {
    memset(w, 0, sizeof(*w));
    w->state = TW_STOPPED;
    for (int i = 0; i < TW_NUM_SERVOS; i++) w->servo_val[i] = TW_CENTER;
}

int table_walk_is_stopped(const tw_walker_t *w) {
    return w->state == TW_STOPPED;
}

/* Small helper matching C#'s float->int truncation-toward-zero used
 * throughout MathWalk() (C# (int) cast on a float truncates toward
 * zero, same as C's (int) cast -- no rounding step needed to match
 * behavior exactly). */
static inline int tw_trunc(float v) {
    return (int)v;
}

void table_walk_update(tw_walker_t *w, const tw_params_t *params) {
    int sv[TW_NUM_SERVOS];
    for (int i = 0; i < TW_NUM_SERVOS; i++) sv[i] = TW_CENTER;

    /* ---- Posture control (works even while stopped) ------------------ */
    /* waist-yaw "decoration" driven by x2 */
    if (fabsf(w->x2) > 0.1f) {
        sv[12] += tw_trunc(600.0f * w->x2);
        sv[13] += tw_trunc(600.0f * w->x2);
        sv[14] += tw_trunc(600.0f * w->x2); /* head yaw follows */

        for (int i = 0; i < 3; i++) {
            sv[i + 0] += tw_trunc((140.0f * w->x2) * (float)LINKDEF_PITCH[i + 0]);
            sv[i + 4] -= tw_trunc((140.0f * w->x2) * (float)LINKDEF_PITCH[i + 4]);
        }
    }

    /* forward/back lean or crouch, driven by y2 */
    if (fabsf(w->y2) > 0.1f) {
        sv[15] += tw_trunc(300.0f * w->x2); /* head roll, note: uses x2 per original */
        sv[16] -= tw_trunc(300.0f * w->y2); /* head pitch */

        if (w->y2 < 0.0f) {
            /* crouch */
            for (int i = 0; i < 8; i++) {
                sv[i] -= tw_trunc((400.0f * w->y2) * (float)LINKDEF_LEN[i]);
            }
        } else {
            /* lean forward -- hardcoded servos 0/3/4/7 exactly as in
             * the original (kazz's own comment: "直値 配列もつけてない。
             * 適当…" -- "hardcoded, didn't even make an array, rough") */
            sv[0] += tw_trunc(150.0f * w->y2);
            sv[4] -= tw_trunc(150.0f * w->y2);
            sv[3] -= tw_trunc(450.0f * w->y2);
            sv[7] += tw_trunc(450.0f * w->y2);
        }
    }

    /* ---- Walking state machine --------------------------------------- */
    if (w->state == TW_STOPPED && w->lt_held) {
        w->state = TW_STARTING;
        w->cur_tick = 0;
    }

    if (w->state != TW_STOPPED) {
        w->cur_tick++;
        if (w->cur_tick >= TW_NUM_FRAMES) {
            w->cur_tick = 0;
            if (w->state == TW_STARTING) w->state = TW_WALKING;
            if (!w->lt_held) w->state = TW_STOPPING;
        }

        float rate_len  = TABLE_SIN3[w->cur_tick];
        float rate_roll = TABLE_SIN[w->cur_tick];
        float rate_p1   = TABLE_PLOT1[w->cur_tick];
        float rate_p2   = TABLE_PLOT2[w->cur_tick];

        int ratio = 2;

        if (w->state == TW_STARTING) {
            /* marking time before full walking kicks in */
            if (w->cur_tick < (TW_NUM_FRAMES / 4)) ratio = 1;
            else w->state = TW_WALKING;
        }
        if (w->state == TW_STOPPING) {
            /* marking time before coming to a full stop */
            if (w->cur_tick < (TW_NUM_FRAMES / 4)) ratio = 1;
            else w->state = TW_STOPPED;
        }

        /* ROLL: weight shift left/right */
        for (int i = 8; i < 12; i++) {
            sv[i] += tw_trunc((rate_roll * (float)params->dura_roll) * (float)LINKDEF_ROLL[i] * (float)ratio);
        }

        /* LEN: foot lift/extend, mirrored depending on sign of rate_len */
        int base = 0;
        float len_mag = rate_len;
        if (len_mag < 0.0f) {
            base = 4;
            len_mag = -len_mag;
        }
        for (int i = base; i < base + 4; i++) {
            sv[i] += tw_trunc((len_mag * (float)params->dura_len) * (float)LINKDEF_LEN[i] * (float)ratio);
        }

        if (w->state == TW_WALKING) {
            /* PITCH: stride, stick-driven (y1) -- only active once fully walking */
            for (int i = 0; i < 4; i++) {
                sv[i + 0] += tw_trunc((rate_p1 * (float)params->dura_pitch) * (float)LINKDEF_PITCH[i + 0] * w->y1);
                sv[i + 4] += tw_trunc((rate_p2 * (float)params->dura_pitch) * (float)LINKDEF_PITCH[i + 4] * w->y1);
            }

            /* YAW: turning, accumulated steering bias (x1) */
            if (w->walking_steering != 0 || fabsf(w->x1) > 0.1f) {
                int steering = -tw_trunc(w->x1 * 250.0f);

                if (w->cur_tick == 2 || w->cur_tick == 3 || w->cur_tick == 4) {
                    if (steering > 0) w->walking_steering += steering;
                    else w->walking_steering = (w->cur_tick == 4) ? 0 : w->walking_steering / 2;
                }
                if (w->cur_tick == 7 || w->cur_tick == 8 || w->cur_tick == 9) {
                    if (steering < 0) w->walking_steering -= steering;
                    else w->walking_steering = (w->cur_tick == 9) ? 0 : w->walking_steering / 2;
                }
            }

            sv[12] += w->walking_steering;
            sv[13] -= w->walking_steering;
        }
    }

    for (int i = 0; i < TW_NUM_SERVOS; i++) {
        w->servo_val[i] = sv[i];
    }
}
