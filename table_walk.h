/*
 * table_walk.h
 *
 * Lightweight table-based walking gait generator for PremaidAI.
 *
 * Ported from kazz's (twitter @kazzlog) PreMaidMathWalkSample.cs,
 * released 2019-07-31 as a stripped-down single-file reference
 * (originally combined into izm's PremaindAI_TechVerification
 * remote-controller tool). Source: pm_sample-master.zip,
 * TinyMathWalk.xlsx "20 Frames" sheet.
 *
 * This REPLACES the IKSolver3D + WalkGenerator classes entirely.
 * Instead of solving inverse kinematics (atan2/acos/sqrt per leg,
 * per tick), this drives all 17 walking-related servos from 4
 * precomputed waveform tables (sin, sin^3, and two phase-shifted
 * "plot" curves) combined through fixed per-servo sign/gain
 * matrices. Far cheaper on an STM32F102 (48MHz, no FPU) -- just
 * table lookups, multiplies, and adds, no transcendental math per
 * tick.
 *
 * Servo ordering (index into ServoVal[] / the LinkDef* tables)
 * matches kazz's original comments and has been cross-verified
 * against the community .pma servo ID map (see analysis notes):
 *
 *   idx  pma_id  role
 *   0    0x10    L thigh pitch     \
 *   1    0x14    L knee pitch       > left foot
 *   2    0x18    L ankle pitch     /
 *   3    0x02    R shoulder pitch  (arm swing counter to L leg)
 *   4    0x0E    R thigh pitch     \
 *   5    0x12    R knee pitch       > right foot
 *   6    0x16    R ankle pitch     /
 *   7    0x04    L shoulder pitch  (arm swing counter to R leg)
 *   8    0x0A    R hip roll        \
 *   9    0x0C    L hip roll         > roll (weight shift)
 *   10   0x1A    R ankle roll      |
 *   11   0x1C    L ankle roll     /
 *   12   0x08    L hip yaw         \ turning
 *   13   0x06    R hip yaw        /
 *   14   0x05    head yaw          \
 *   15   0x07    head roll          > head (decorative, per kazz's
 *   16   0x03    head pitch        /  own comment: "首まわり制御は飾り")
 *
 * All 17 of these map onto HV bus servos except indices 14-16
 * (head, MV bus). See table_walk_servo_map() below for the actual
 * bus + ICS-ID + firmware angle-limit lookup, which pulls from your
 * existing servoList[] (ServoInfo) rather than duplicating limits
 * here -- this module ONLY computes target angles, it does not
 * decide safety clamping; the caller is expected to clamp against
 * the authoritative ServoInfo.minAngle/maxAngle before writing to
 * the ICS bus, exactly like every other command path in the
 * firmware (S HV/MV, PMA player, etc.) already does.
 */

#ifndef TABLE_WALK_H
#define TABLE_WALK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TW_NUM_SERVOS   17
#define TW_NUM_FRAMES   20      /* "20 Frames" table from TinyMathWalk.xlsx */
#define TW_CENTER       7500    /* neutral pulse value, matches ICS convention */

/* Walking state machine, mirrors kazz's WalkingCondition:
 *   0 = stopped
 *   1 = starting up (marking time in place for half a cycle)
 *   2 = walking (stick input live)
 *   3 = winding down (marking time before stopping) */
typedef enum {
    TW_STOPPED   = 0,
    TW_STARTING  = 1,
    TW_WALKING   = 2,
    TW_STOPPING  = 3
} tw_walk_state_t;

typedef struct {
    tw_walk_state_t state;
    int      cur_tick;        /* 0..TW_NUM_FRAMES-1 position in the gait cycle */
    int      walking_steering; /* accumulated turn bias, mirrors WalkingSteering */

    /* Inputs, refreshed by the caller each tick before calling
     * table_walk_update(). Ranges roughly [-1,1] like a joystick axis. */
    float x1; /* turn stick (yaw) */
    float y1; /* forward/back stick (stride) */
    float x2; /* posture: waist yaw "decoration" */
    float y2; /* posture: crouch / lean forward */
    int   lt_held; /* "walk" trigger button, mirrors LT */

    /* Output: raw ICS pulse targets for the 17 servos, index order
     * as documented above. Caller reads this after table_walk_update(). */
    int servo_val[TW_NUM_SERVOS];
} tw_walker_t;

/* Tunable durations, mirror kazz's DuraLen/DuraRoll/DuraPitch/DuraSquat.
 * Exposed as a struct instead of #defines so you can wire these up to
 * your existing "IK SET ..." style tuning commands if you want to keep
 * that UX; defaults match the original sample exactly. */
typedef struct {
    int dura_len;    /* default 140 -- foot lift/extend amplitude */
    int dura_roll;   /* default  80 -- weight-shift roll amplitude */
    int dura_pitch;  /* default 600 -- stride pitch amplitude (stick-driven) */
    int dura_squat;  /* default 200 -- unused directly in MathWalk() but
                        kept for parity / future crouch feature */
} tw_params_t;

void tw_params_init(tw_params_t *p);
void tw_walker_init(tw_walker_t *w);

void table_walk_update(tw_walker_t *w, const tw_params_t *params);

int table_walk_is_stopped(const tw_walker_t *w);

uint8_t table_walk_servo_pma_id(int index);

#ifdef __cplusplus
}
#endif

#endif /* TABLE_WALK_H */