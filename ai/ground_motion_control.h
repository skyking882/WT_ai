#ifndef GROUND_MOTION_CONTROL_H
#define GROUND_MOTION_CONTROL_H

#include "ground_motion.h"

#define GM_PRODUCTION_PUBLISH_MS 90u
#define GM_PRODUCTION_CRUISE_MPS 10.0f
#define GM_PRODUCTION_ACCEL_MPS2 3.0f
#define GM_PRODUCTION_BRAKE_MPS2 2.5f
#define GM_PRODUCTION_MAX_YAW_RADPS 0.65f
#define GM_PRODUCTION_YAW_ACCEL_RADPS2 1.8f
#define GM_PRODUCTION_COMMAND_DELAY_S 0.080f
#define GM_PRODUCTION_MAX_TRANSPORT_DELAY_MS 160u
#define GM_PRODUCTION_MAX_CONSUME_DELAY_MS 40u
#define GM_PRODUCTION_MAX_COMMAND_DELAY_MS (GM_PRODUCTION_MAX_TRANSPORT_DELAY_MS + GM_PRODUCTION_MAX_CONSUME_DELAY_MS)
#define GM_PRODUCTION_OPENING_SPAWN_MS 30000u
#define GM_PRODUCTION_SPAWN_SITE_MATCH_M 0.60f
#define GM_PRODUCTION_SPAWN_HALF_LENGTH_M 3.10f
#define GM_PRODUCTION_SPAWN_HALF_WIDTH_M 1.65f
#define GM_CONTROL_HISTORY_CAPACITY 16
#define GM_PRODUCTION_REVERSE_MPS 1.1f
#define GM_PRODUCTION_TIGHT_ARRIVAL_M 3.0f
#define GM_PRODUCTION_DEFAULT_ARRIVAL_M 3.5f
#define GM_PRODUCTION_MIN_ARRIVAL_M 1.0f
#define GM_PRODUCTION_MAX_ARRIVAL_M 12.0f

typedef struct {
    uint8_t throttle, steer, brake_left, brake_right;
} gm_controls_t;

/* Zero-initialize on a new physical vehicle life. A route change retains
 * the measured actuator response. Releasing ownership clears the transient. */
typedef struct {
    uint64_t command_ms, sample_ms, steady_ms;
    float heading, rate, gain, steer_reference, sample_gain;
    unsigned mode;
} gm_yaw_response_t;

void gm_control_yaw_note(gm_yaw_response_t *state, gm_controls_t written, uint64_t ms);
void gm_control_yaw_release(gm_yaw_response_t *state);
float gm_control_yaw_observe(gm_yaw_response_t *state, float heading, uint64_t ms);
float gm_control_yaw_limit(const gm_yaw_response_t *state, float nominal);

/* Only committed packets enter the per-life history. Consecutive
 * equal packets share one timestamp, from the actual change of command. */
typedef struct {
    uint64_t ms;
    gm_controls_t packet;
} gm_control_commit_t;

typedef struct {
    gm_control_commit_t samples[GM_CONTROL_HISTORY_CAPACITY];
    unsigned first, count;
} gm_control_history_t;

void gm_control_history_note(gm_control_history_t *history, gm_controls_t packet, uint64_t ms);
int gm_control_history_snapshot(const gm_control_history_t *history, gm_pending_command_t *out, uint64_t ms);
int gm_control_spawn_site_open(const gm_spawn_site_t *site, const gm_vehicle_t *vehicles, int count);

/* Full use of the available actuators when a matching plan is absent.
 * A matching plan can intentionally steer and release both brakes. */
gm_controls_t gm_control_track(const gm_vehicle_t *vehicle, const gm_vehicle_plan_t *plan,
                               const gm_pose_t *measured, float age_s);

/* Approximate the last actuator demand. Brakes oppose measured signed motion. */
void gm_control_pending(const gm_vehicle_t *vehicle, gm_controls_t action, float measured_speed,
                        float *acceleration, float *yaw_target);

#endif
