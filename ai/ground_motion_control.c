#include "ground_motion_control.h"
#include <math.h>

#define GM_CONTROL_COAST_BAND 1.0f      /* m/s above the target speed that is left to rolling resistance */
#define GM_CONTROL_SPEED_HORIZON_S 0.6f /* a speed error is taken out over this time */
#define GM_CONTROL_REST_MPS 0.3f        /* below this the hull is at rest, whatever sign its speed reads */
/* Turning on the spot. Measured in the battles of 2026-11-14 and 2026-11-15
 * (23 tanks, 290 s of it): standing with the steering hard over and no
 * throttle a tank in the game does not turn, median 0.00 to 0.04 rad/s on
 * every one; with throttle the same tanks turn 0.4 to 0.8 rad/s, about
 * 0.42 rad for every metre they roll. A turn "on the spot" is therefore
 * driven: throttle until the hull turns at GM_CONTROL_PIVOT_SHARE of the rate
 * asked or rolls at GM_CONTROL_PIVOT_MPS, and the brakes only for what rolls
 * clearly faster. The metres it creeps are the ones the planner's turn from
 * rest is checked for (GM_PIVOT_CREEP, 3 m per radian). */
#define GM_CONTROL_PIVOT_MPS 1.2f
#define GM_CONTROL_PIVOT_THROTTLE 0.5f
#define GM_CONTROL_PIVOT_SHARE 0.6f

static float gm_control_clamp(float value, float lower, float upper)
{
    return fminf(upper, fmaxf(lower, value));
}

static uint8_t gm_control_axis(float value)
{
    const float axis = gm_control_clamp(value, -1.0f, 1.0f);
    return (uint8_t)lroundf(128.0f + axis * (axis < 0.0f ? 128.0f : 127.0f));
}

static float gm_control_decode_axis(uint8_t axis)
{
    return ((float)axis - 128.0f) / (axis < 128 ? 128.0f : 127.0f);
}

void gm_control_history_note(gm_control_history_t *history, gm_controls_t packet, uint64_t ms)
{
    if (history->count) {
        const unsigned last = (history->first + history->count - 1u) % GM_CONTROL_HISTORY_CAPACITY;
        const gm_control_commit_t *prior = &history->samples[last];
        if (ms < prior->ms) {
            history->first = history->count = 0;
        } else if (ms == prior->ms) {
            history->samples[last].packet = packet;
            return;
        } else if (packet.throttle == prior->packet.throttle && packet.steer == prior->packet.steer
                   && packet.brake_left == prior->packet.brake_left && packet.brake_right == prior->packet.brake_right) {
            return;
        }
    }
    if (history->count == GM_CONTROL_HISTORY_CAPACITY) {
        history->first = (history->first + 1u) % GM_CONTROL_HISTORY_CAPACITY;
        --history->count;
    }
    const unsigned next = (history->first + history->count) % GM_CONTROL_HISTORY_CAPACITY;
    history->samples[next] = (gm_control_commit_t){ms, packet};
    ++history->count;
}

int gm_control_history_snapshot(const gm_control_history_t *history, gm_pending_command_t *out, uint64_t ms)
{
    unsigned begin = 0;
    while (begin + 1 < history->count) {
        const unsigned next = (history->first + begin + 1u) % GM_CONTROL_HISTORY_CAPACITY;
        const uint64_t time = history->samples[next].ms;
        if (time > ms || ms - time < GM_PRODUCTION_MAX_COMMAND_DELAY_MS)
            break;
        ++begin;
    }

    int count = 0;
    for (unsigned i = begin; i < history->count; ++i) {
        const gm_control_commit_t *entry = &history->samples[(history->first + i) % GM_CONTROL_HISTORY_CAPACITY];
        if (entry->ms > ms)
            break;
        const gm_controls_t packet = entry->packet;
        const float age_s = 0.001f * (float)(ms - entry->ms);
        const float brake = fmaxf(packet.brake_left, packet.brake_right) / 255.0f;
        out[count++] = (gm_pending_command_t){.age_s = age_s, .drive = gm_control_decode_axis(packet.throttle),
            .brake = brake, .steer = gm_control_decode_axis(packet.steer)};
    }
    return count;
}

int gm_control_spawn_site_open(const gm_spawn_site_t *site, const gm_vehicle_t *vehicles, int count)
{
    const float radius = hypotf(site->half_length, site->half_width);
    for (int i = 0; i < count; ++i) {
        const gm_vehicle_t *v = &vehicles[i];
        const float dx = site->p.x - v->x, dz = site->p.z - v->z;
        const float limit = radius + hypotf(0.5f * v->length, 0.5f * v->width);
        if (dx * dx + dz * dz < limit * limit)
            return 0;
    }
    return 1;
}

void gm_control_yaw_note(gm_yaw_response_t *state, gm_controls_t written, uint64_t ms)
{
    const float steer = gm_control_decode_axis(written.steer);
    const unsigned mode = written.brake_left || written.brake_right || written.throttle < 128 ? 0u
        : written.throttle == 128 ? 1u : 2u;
    if (!state->command_ms || ms < state->command_ms || state->mode != mode
        || fabsf(steer - state->steer_reference) > 0.06f) {
        state->command_ms = ms;
        state->steer_reference = steer;
        state->mode = mode;
        state->steady_ms = 0;
        state->sample_gain = 0.0f;
    }
}

void gm_control_yaw_release(gm_yaw_response_t *state)
{
    const float gain = state->gain;
    *state = (gm_yaw_response_t){.gain = gain};
}

float gm_control_yaw_observe(gm_yaw_response_t *state, float heading, uint64_t ms)
{
    if (!isfinite(heading)) {
        gm_control_yaw_release(state);
        return 0.0f;
    }
    const uint64_t prev_ms = state->sample_ms;
    const float prev_heading = state->heading;
    state->sample_ms = ms;
    state->heading = heading;
    if (!prev_ms || ms <= prev_ms || ms - prev_ms > 500u) {
        state->steady_ms = 0;
        state->sample_gain = 0.0f;
        state->rate = 0.0f;
        return 0.0f;
    }
    state->rate = remainderf(heading - prev_heading, 6.2831853f) * 1000.0f / (float)(ms - prev_ms);

    /* Ignore samples whose measurement interval could contain an earlier
     * command (160 ms device delay plus sampling uncertainty). Require a
     * steady normalized response before learning from that command. */
    if (!state->mode || !state->command_ms || prev_ms < state->command_ms + 200u
        || fabsf(state->steer_reference) < 0.55f)
        return state->rate;
    const float gain = state->rate / state->steer_reference;
    if (gain < 0.30f || gain > 1.10f || !isfinite(gain)) {
        state->steady_ms = 0;
        state->sample_gain = 0.0f;
        return state->rate;
    }
    if (!state->steady_ms || fabsf(gain - state->sample_gain) > 0.09f) {
        state->steady_ms = ms;
        state->sample_gain = gain;
        return state->rate;
    }
    if (ms - state->steady_ms >= 160u) {
        const float estimate = 0.5f * (gain + state->sample_gain);
        const float previous = state->gain > 0.0f ? state->gain : GM_PRODUCTION_MAX_YAW_RADPS;
        state->gain = previous + gm_control_clamp(estimate - previous, -0.05f, 0.05f);
        state->sample_gain = estimate;
    }
    return state->rate;
}

float gm_control_yaw_limit(const gm_yaw_response_t *state, float nominal)
{
    return state->gain > 0.0f ? state->gain : nominal;
}

gm_controls_t gm_control_track(const gm_vehicle_t *vehicle, const gm_vehicle_plan_t *plan,
                               const gm_pose_t *measured, float age_s)
{
    gm_controls_t action = {128, 128, 255, 255};
    if (!vehicle || !plan || !measured || vehicle->id != plan->id || vehicle->life_id != plan->life_id
        || vehicle->route_id != plan->route_id || !isfinite(age_s) || age_s < 0.0f || age_s > 0.001f * GM_PLAN_VALID_MS
        || !isfinite(measured->x) || !isfinite(measured->z) || !isfinite(measured->speed)
        || !isfinite(measured->heading) || !isfinite(measured->yaw_rate)
        || !isfinite(plan->target_speed) || !isfinite(plan->target_yaw_rate))
        return action;

    const float target_speed = plan->target_speed;
    const int reverse = target_speed < 0.0f;
    if (reverse && (!plan->terrain_recovery || plan->geometry != GM_CLEAR || plan->blocker_id
        || !isfinite(vehicle->reverse_speed) || -target_speed > vehicle->reverse_speed))
        return action;

    const float accel = fmaxf(0.1f, vehicle->acceleration);
    const float braking = fmaxf(0.1f, vehicle->braking);
    /* Reverse terrain legs are straight. The native steering sign can depend
     * on track direction, so their steering byte stays centered. */
    const float target_yaw = reverse ? 0.0f : plan->target_yaw_rate;
    if (plan->local_drive && isfinite(plan->curvature) && target_speed != 0.0f) {
        /* s13 local planner: a curvature per signed metre, forwards and
         * backwards. The game steers a hull like a car: at a crawl full
         * steering gives about turn_per_m rad per metre rolled, faster the yaw
         * rate limit caps it, and rolling back the same steering turns the
         * heading the other way per second but the same way per signed metre
         * (2026-11-15: the reverse sign learned live is -1 in yaw-rate terms).
         * At rest the hull is about to roll: the least speed that counts. */
        const float per_m = vehicle->turn_per_m > 0.0f ? vehicle->turn_per_m
                            : vehicle->can_pivot ? GM_TRACKED_TURN_PER_M : GM_WHEELED_TURN_PER_M;
        const float rolled = fmaxf(fabsf(measured->speed), GM_CONTROL_REST_MPS);
        const float yaw = fminf(fmaxf(0.1f, vehicle->max_yaw_rate), GM_LOCAL_YAW_CAP);
        const float most = fminf(per_m, yaw / rolled);
        action.steer = gm_control_axis(plan->curvature / fmaxf(most, 0.01f));
    } else {
        if (target_speed == 0.0f && fabsf(target_yaw) <= 0.001f)
            return action;
        action.steer = gm_control_axis(target_yaw / fmaxf(0.1f, vehicle->max_yaw_rate));
    }

    const float direction = reverse ? -1.0f : 1.0f;
    /* Rolling the other way it brakes first. A hull at rest reads a speed
     * within a fraction of a millimetre per second of either sign: taking a
     * negative reading for rolling back held the brakes on a standing tank
     * that had just been told to drive. */
    const float along = direction * measured->speed;
    if (target_speed != 0.0f && along < -GM_CONTROL_REST_MPS)
        return action;
    const float speed = target_speed == 0.0f ? fabsf(measured->speed) : fmaxf(0.0f, along);

    /* Speed is held with part throttle, a little too fast just coasts, and
     * the brakes come on only when clearly too fast: full throttle against
     * full brake every few tenths of a second makes the hull lurch. */
    const float excess = speed - fabsf(target_speed);
    if (target_speed == 0.0f) {
        const float limit = GM_CONTROL_PIVOT_MPS + 0.3f;
        const float brake = speed > limit ? gm_control_clamp((speed - limit) / 2.0f, 0.05f, 1.0f) : 0.0f;
        action.brake_left = action.brake_right = (uint8_t)lroundf(255.0f * brake);
        if (brake == 0.0f && speed < GM_CONTROL_PIVOT_MPS
            && fabsf(measured->yaw_rate) < GM_CONTROL_PIVOT_SHARE * fabsf(target_yaw))
            action.throttle = gm_control_axis(GM_CONTROL_PIVOT_THROTTLE);
        return action;
    }
    /* Slowing for another hull, or for an obstacle ahead, is not cruise
     * control: brake at once. */
    const int at_once = plan->blocker_id || plan->firm;
    const float band = at_once ? 0.2f : GM_CONTROL_COAST_BAND;
    if (excess > band) {
        const float brake = gm_control_clamp((excess - band) / (braking * (at_once ? 0.3f : GM_CONTROL_SPEED_HORIZON_S)), 0.1f, 1.0f);
        action.brake_left = action.brake_right = (uint8_t)lroundf(255.0f * brake);
        return action;
    }
    action.brake_left = action.brake_right = 0;
    if (excess < 0.0f)
        action.throttle = gm_control_axis(direction * gm_control_clamp(-excess / (accel * GM_CONTROL_SPEED_HORIZON_S), 0.12f, 1.0f));
    return action;
}

void gm_control_pending(const gm_vehicle_t *vehicle, gm_controls_t action, float measured_speed,
                        float *acceleration, float *yaw_target)
{
    const float brake = (float)fmaxf(action.brake_left, action.brake_right) / 255.0f;
    const float drive = gm_control_decode_axis(action.throttle);
    const float steer = gm_control_decode_axis(action.steer);
    if (brake > 0.0f) {
        const float direction = measured_speed < 0.0f ? 1.0f : measured_speed > 0.0f ? -1.0f : 0.0f;
        *acceleration = direction * brake * vehicle->braking;
    } else {
        *acceleration = drive * vehicle->acceleration;
    }
    *yaw_target = steer * vehicle->max_yaw_rate;
}
