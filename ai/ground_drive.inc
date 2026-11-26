/* ---- s22 2026-11-17: the WASD driver (single-vehicle-control handoff, task 2) ----
 *
 * One driver per bot, like a player: it presses W / S / A / D and nothing else. The game's input layer
 * (of the frozen game build) turns the two bytes into throttle, track brakes and gear; gd_layer below is a C
 * transcription of it (stages/s19_tank_phys_re/layer_check.py, checked there on 2875 logged rows), used only by
 * the offline test and to tell the planner's bookkeeping what the physics will be given.
 *
 * Keys are bytes a player can produce: throttle 255 (W), 0 (S) or 128 (none); steer 128 + 127 * s for s > 0,
 * 128 + 128 * s for s < 0 (s = -1 .. 1). Steer sign as the s12..s20 tracker: a positive heading error
 * (target towards +z of the nose, heading = atan2(fz, fx)) gives a byte above 128, which the layer turns into a
 * brake on the track of the left brake byte (the inner one).
 *
 * Two states (plan table):
 *   pivot  entered when |e| > 30 deg. Above 1 m/s: the brake key alone. At or below: A or D alone, full steer,
 *          until |e| < 8 deg. Reverse leg or a reverse-gear pivot asked for: S + A / D, steer negated.
 *          A vehicle that cannot pivot on A / D alone: W (reverse: S) + full steer.
 *   drive  steer = e / 30 deg. Two limits (s22b, after the verification battle: W released at 0.8 * cruise
 *          left the tanks at 0.8 of the planner's speed): cruise = in->cruise (s22c: INFINITY when the planner
 *          did not lower its speed on purpose: hold W; 0 = v_cap); hard = the stop-distance limit, and
 *          v_cap too when the plan is firm, else none. W while along < cruise and along < 0.8 * hard; the brake
 *          key when along > hard or along > cruise + 1 m/s; nothing in between.
 *   hold   (v_allow 0, or no plan for 500 ms): the brake key above 0.5 m/s, else nothing: the game's idle brake
 *          stops and parks the hull. Never S held at rest (the gearbox would engage reverse and back out).
 *          A hold with a heading to turn to (align) uses the pivot row: it pivots while |e| >= 8 deg, and once
 *          settled only again when the heading asked for moves by 8 deg or more, or |e| exceeds 30 deg.
 * Reverse leg: W and S swap, steer is negated, e is measured from the tail.
 * The brake key is the one that brakes in the gear in use: S in a forward gear or neutral, W in a reverse gear
 * (the layer: both brakes = the throttle against the gear's direction). In the normal case this is the leg's
 * swapped key; it differs only while a hull rolls the other way in the other gear.
 *
 * Stop-distance limit sqrt(2 * a_use * (d - 1 m)), a_use = 0.7 * maxBrakeForce / mass - g * sin(down slope);
 * v_allow (logged) = min(cruise, hard). Stage A: no stop distance d (NAN) and no slope.
 * No vehicle model: the only physical value is the brake deceleration read from the game object. */
#include <math.h>
#include <stdint.h>

#define GD_PIVOT_ENTER_DEG 30.0f   /* larger errors: stop, then turn on the spot */
#define GD_PIVOT_EXIT_DEG 8.0f     /* pivot until the nose is this close */
#define GD_STEER_FULL_DEG 30.0f    /* drive: steer = e / 30 deg */
#define GD_PIVOT_MAX_MPS 1.0f      /* pivot only at or below this speed; above: brake first */
#define GD_HOLD_RELEASE_MPS 0.5f   /* hold: release the brake key at or below this speed */
#define GD_W_SHARE 0.8f            /* W below 0.8 * hard limit: a band without keys between W and the brake */
#define GD_OVER_MPS 1.0f           /* s22b: above the cruise speed lift off; brake only above cruise + 1 m/s */
#define GD_BRAKE_SHARE 0.7f        /* a_use = 0.7 * a: margin for grip */
#define GD_STOP_MARGIN_M 1.0f      /* stop this short of the stop distance */
#define GD_PLAN_STALE_MS 500u      /* no plan update for longer: hold */
#define GD_AIM_MIN_M 1.0f          /* s22: an aim point closer than this has no direction: e = 0 (not in the plan) */
#define GD_G 9.81f

#define GD_KEY_W 255u
#define GD_KEY_S 0u
#define GD_KEY_NONE 128u

enum { GD_HOLD, GD_PIVOT, GD_DRIVE };
enum { GD_AIM_NONE, GD_AIM_POINT, GD_AIM_HEADING };

/* What the planner asks for (stage A: derived from gm_vehicle_plan_t, ground_vehicle_control.inc). */
typedef struct {
    uint64_t ms;          /* publication the input comes from; 0 = none */
    int aim;              /* GD_AIM_* */
    float x, z;           /* GD_AIM_POINT: aim point P */
    float heading;        /* GD_AIM_HEADING: the nose heading asked for (rad, atan2(fz, fx)), also on reverse legs */
    float v_cap;          /* m/s, >= 0; 0 = stop and hold; the hard limit when firm */
    int firm;             /* s22b: v_cap is also a hard limit (slowing for a hull or an obstacle ahead) */
    float cruise;         /* s22c: the speed to lift off at; INFINITY = none (hold W); 0 or NAN = v_cap */
    int reverse;          /* reverse leg */
    int pivot_reverse;    /* turn on the spot in reverse gear (S + A / D); stage B, 0 in stage A */
    int align;            /* holding, with the hull to be turned to `heading` (pivot row) */
    float stop_m;         /* stop distance d; NAN = none (stage A) */
    /* s24 path mode (gd_drive_path): go = it may drive (else hold); stop_m counts from (from_x, from_z), the
     * pose it was measured at: the distance driven since is taken off; cruise = the speed cap (INFINITY none);
     * route_rev: the nav revision followed (probe only). */
    int path, go;
    float from_x, from_z;
    unsigned route_rev;
} gd_input_t;

/* What the driver reads from the game object, every physics step. */
typedef struct {
    float x, z, heading;  /* position, heading atan2(fz, fx) */
    float speed;          /* signed along the hull, + forwards */
    int reverse_gear;     /* the gear in use has a negative ratio */
    int can_pivot;        /* tracked, and separate brakes or the neutral-steer boost set (read once per life) */
    float brake_mps2;     /* maxBrakeForce / mass (gh_vphys_drive_params), read once per life */
    float down_slope;     /* rad, nose-down pitch along the travel direction; stage A: 0 */
} gd_vehicle_t;

typedef struct {
    int mode;             /* GD_HOLD / GD_PIVOT / GD_DRIVE */
    int settled;          /* align: the pivot ended for settled_heading */
    float settled_heading;
} gd_memory_t;

typedef struct {
    uint8_t throttle, steer;
} gd_keys_t;

typedef struct {
    int mode, stale;
    float e_deg, v_allow, a_use;
    float d_m;            /* s24 path mode: the stop distance in use (published d less the distance driven since) */
} gd_info_t;

static float gd_wrap(float a)
{
    return remainderf(a, 6.2831853f);
}

static uint8_t gd_steer_byte(float s)
{
    const float c = fminf(1.0f, fmaxf(-1.0f, s));
    return (uint8_t)lroundf(128.0f + c * (c < 0.0f ? 128.0f : 127.0f));
}

/* ---- s24 path mode: the driver only drives (design rule: W / S / A / D only). ----
 * Inputs (planner, gm_path_decide): go, the aim point (or, holding, a heading to turn to: the s22 align row), the
 * stop distance d, a reverse leg, a speed cap. It chooses nothing:
 *   hold   only when told to stop (go 0) or when the plan is stale (500 ms): the brake key above 0.5 m/s, else
 *          nothing (the game's idle brake parks the hull; S is never held at rest).
 *   pivot  |e| > 30 deg: the brake key above 1 m/s, then steer alone (full) until |e| < 8 deg.
 *   drive  steer = e / 30 deg. d limits forward speed only, never turning: hard = sqrt(2 a_use (d - 1 m)); the
 *          brake key above max(hard, 0.5 m/s) or above the cap + 1 m/s; W below 0.8 hard and below the cap;
 *          nothing in between. No room ahead (d <= 1 m): |e| >= 8 deg turns on the spot towards the aim (steer
 *          alone), under 8 deg no keys at all.
 *   d counts from the pose it was measured at (from_x, from_z): the distance driven since is taken off.
 * Reverse legs (W and S swapped, e from the tail, S + A / D to turn), the brake key by gear, W + full steer for
 * a vehicle that cannot pivot on A / D: as s22. */
static void gd_pivot_keys(gd_keys_t *k, const gd_input_t *in, const gd_vehicle_t *v, float e)
{
    const int rev = in->reverse || in->pivot_reverse;
    const int positive = (e > 0.0f) != (rev != 0);
    k->steer = positive ? 255u : 0u;
    if (rev)
        k->throttle = GD_KEY_S;
    else if (!v->can_pivot)
        k->throttle = GD_KEY_W;
}

static gd_keys_t gd_drive_path(gd_memory_t *m, const gd_input_t *in, const gd_vehicle_t *v, uint64_t ms, gd_info_t *info)
{
    gd_keys_t k = {GD_KEY_NONE, GD_KEY_NONE};
    const int stale = !in->ms || ms < in->ms || ms - in->ms > GD_PLAN_STALE_MS || !isfinite(v->speed) || !isfinite(v->heading);
    const uint8_t go = in->reverse ? GD_KEY_S : GD_KEY_W;
    const uint8_t brake = v->reverse_gear ? GD_KEY_W : GD_KEY_S;
    const float leg = in->reverse ? -1.0f : 1.0f;
    const float a_use = GD_BRAKE_SHARE * v->brake_mps2 - GD_G * sinf(v->down_slope);
    float d = INFINITY;
    if (!stale && isfinite(in->stop_m)) {
        d = in->stop_m;
        if (isfinite(in->from_x) && isfinite(in->from_z) && isfinite(v->x) && isfinite(v->z))
            d -= hypotf(v->x - in->from_x, v->z - in->from_z);
        d = fmaxf(0.0f, d);
    }
    const float hard = isfinite(d) ? sqrtf(fmaxf(0.0f, 2.0f * a_use * (d - GD_STOP_MARGIN_M))) : INFINITY;
    const float cap = !stale && in->cruise > 0.0f ? in->cruise : INFINITY;
    const int hold = stale || !in->go;

    float e = 0.0f;
    int has_aim = 0;
    if (!stale && in->aim == GD_AIM_POINT && isfinite(in->x) && isfinite(in->z) && isfinite(v->x) && isfinite(v->z)) {
        const float dx = in->x - v->x, dz = in->z - v->z;
        if (hypotf(dx, dz) >= GD_AIM_MIN_M) {
            e = gd_wrap(atan2f(dz, dx) - v->heading - (in->reverse ? 3.14159265f : 0.0f));
            has_aim = 1;
        }
    } else if (!stale && in->aim == GD_AIM_HEADING && isfinite(in->heading)) {
        e = gd_wrap(in->heading - v->heading);
        has_aim = 1;
    }
    const float deg = fabsf(e) * 57.29578f;

    const int aligning = hold && !stale && in->align && has_aim && in->aim == GD_AIM_HEADING;
    if (!aligning || (m->settled && fabsf(gd_wrap(in->heading - m->settled_heading)) * 57.29578f >= GD_PIVOT_EXIT_DEG))
        m->settled = 0;
    if (hold && !aligning) {
        m->mode = GD_HOLD;
    } else if (aligning) {
        if (m->mode == GD_PIVOT) {
            if (deg < GD_PIVOT_EXIT_DEG) {
                m->mode = GD_HOLD;
                m->settled = 1;
                m->settled_heading = in->heading;
            }
        } else {
            m->mode = deg > GD_PIVOT_ENTER_DEG || (!m->settled && deg >= GD_PIVOT_EXIT_DEG) ? GD_PIVOT : GD_HOLD;
        }
    } else if (m->mode == GD_PIVOT) {
        if (deg < GD_PIVOT_EXIT_DEG)
            m->mode = GD_DRIVE;
    } else {
        m->mode = deg > GD_PIVOT_ENTER_DEG ? GD_PIVOT : GD_DRIVE;
    }

    if (m->mode == GD_HOLD) {
        if (fabsf(v->speed) > GD_HOLD_RELEASE_MPS)
            k.throttle = brake;
    } else if (m->mode == GD_PIVOT) {
        if (fabsf(v->speed) > GD_PIVOT_MAX_MPS)
            k.throttle = brake;
        else
            gd_pivot_keys(&k, in, v, e);
    } else {
        const float along = leg * v->speed;
        if (along > fmaxf(hard, GD_HOLD_RELEASE_MPS) || along > cap + GD_OVER_MPS) {
            k.steer = gd_steer_byte(leg * e * 57.29578f / GD_STEER_FULL_DEG);
            k.throttle = brake;
        } else if (d <= GD_STOP_MARGIN_M) {
            if (deg >= GD_PIVOT_EXIT_DEG)
                gd_pivot_keys(&k, in, v, e); /* no room ahead: turn on the spot towards the aim */
        } else {
            k.steer = gd_steer_byte(leg * e * 57.29578f / GD_STEER_FULL_DEG);
            if (along < GD_W_SHARE * hard && along < cap)
                k.throttle = go;
        }
    }
    if (info)
        *info = (gd_info_t){.mode = m->mode, .stale = stale, .e_deg = e * 57.29578f, .v_allow = fminf(cap, hard),
                            .a_use = a_use, .d_m = d};
    return k;
}

static gd_keys_t gd_drive(gd_memory_t *m, const gd_input_t *in, const gd_vehicle_t *v, uint64_t ms, gd_info_t *info)
{
    if (in->path)
        return gd_drive_path(m, in, v, ms, info);
    gd_keys_t k = {GD_KEY_NONE, GD_KEY_NONE};
    const int stale = !in->ms || ms < in->ms || ms - in->ms > GD_PLAN_STALE_MS || !isfinite(in->v_cap)
        || !isfinite(v->speed) || !isfinite(v->heading);
    const uint8_t go = in->reverse ? GD_KEY_S : GD_KEY_W;
    const uint8_t brake = v->reverse_gear ? GD_KEY_W : GD_KEY_S;
    const float leg = in->reverse ? -1.0f : 1.0f;
    const float a_use = GD_BRAKE_SHARE * v->brake_mps2 - GD_G * sinf(v->down_slope);
    const float cap = stale ? 0.0f : fmaxf(0.0f, in->v_cap);
    /* s22c: the speed to lift off at; INFINITY = none (hold W). NAN or <= 0 falls back to v_cap. */
    const float cruise = stale ? 0.0f : in->cruise > 0.0f ? in->cruise : cap;
    float hard = INFINITY;
    if (!stale && isfinite(in->stop_m))
        hard = sqrtf(fmaxf(0.0f, 2.0f * a_use * (in->stop_m - GD_STOP_MARGIN_M)));
    if (!stale && in->firm)
        hard = fminf(hard, cap);
    const float v_allow = fminf(cruise, hard);
    const int hold = stale || !(cap > 0.0f) || !(cruise > 0.0f) || !(hard > 0.0f);

    float e = 0.0f;
    int has_aim = 0;
    if (!stale && in->aim == GD_AIM_POINT && isfinite(in->x) && isfinite(in->z) && isfinite(v->x) && isfinite(v->z)) {
        const float dx = in->x - v->x, dz = in->z - v->z;
        if (hypotf(dx, dz) >= GD_AIM_MIN_M) {
            e = gd_wrap(atan2f(dz, dx) - v->heading - (in->reverse ? 3.14159265f : 0.0f));
            has_aim = 1;
        }
    } else if (!stale && in->aim == GD_AIM_HEADING && isfinite(in->heading)) {
        e = gd_wrap(in->heading - v->heading);
        has_aim = 1;
    }
    const float deg = fabsf(e) * 57.29578f;

    const int aligning = hold && !stale && in->align && has_aim && in->aim == GD_AIM_HEADING;
    if (!aligning || (m->settled && fabsf(gd_wrap(in->heading - m->settled_heading)) * 57.29578f >= GD_PIVOT_EXIT_DEG))
        m->settled = 0;
    if (hold && !aligning) {
        m->mode = GD_HOLD;
    } else if (aligning) {
        if (m->mode == GD_PIVOT) {
            if (deg < GD_PIVOT_EXIT_DEG) {
                m->mode = GD_HOLD;
                m->settled = 1;
                m->settled_heading = in->heading;
            }
        } else {
            m->mode = deg > GD_PIVOT_ENTER_DEG || (!m->settled && deg >= GD_PIVOT_EXIT_DEG) ? GD_PIVOT : GD_HOLD;
        }
    } else if (m->mode == GD_PIVOT) {
        if (deg < GD_PIVOT_EXIT_DEG)
            m->mode = GD_DRIVE;
    } else {
        m->mode = deg > GD_PIVOT_ENTER_DEG ? GD_PIVOT : GD_DRIVE;
    }

    if (m->mode == GD_HOLD) {
        if (fabsf(v->speed) > GD_HOLD_RELEASE_MPS)
            k.throttle = brake;
    } else if (m->mode == GD_PIVOT) {
        if (fabsf(v->speed) > GD_PIVOT_MAX_MPS) {
            k.throttle = brake;
        } else {
            const int rev = in->reverse || in->pivot_reverse;
            const int positive = (e > 0.0f) != (rev != 0);
            k.steer = positive ? 255u : 0u;
            if (rev)
                k.throttle = GD_KEY_S;
            else if (!v->can_pivot)
                k.throttle = GD_KEY_W;
        }
    } else {
        k.steer = gd_steer_byte(leg * e * 57.29578f / GD_STEER_FULL_DEG);
        const float along = leg * v->speed;
        if (along < cruise && along < GD_W_SHARE * hard)
            k.throttle = go;
        else if (along > hard || along > cruise + GD_OVER_MPS)
            k.throttle = brake;
    }
    if (info)
        *info = (gd_info_t){.mode = m->mode, .stale = stale, .e_deg = e * 57.29578f, .v_allow = v_allow, .a_use = a_use};
    return k;
}

/* ---- The game's input layer for tracked vehicles, steer type 1, automatic gearbox, direction modes
 * off (stages/s19_tank_phys_re/layer_check.py, line by line). Input: the throttle and steer bytes it finds; output:
 * the four bytes it writes back. boost: separate brakes or the neutral-steer boost set (throttle rewrite while steering). separate:
 * separate brakes set; without separate brakes the two brakes are merged to max(left, right) (s19 notes on the input chain;
 * not checked against logged rows: every logged vehicle had separate brakes). idle: no throttle and no steer:
 * the brakes then come from the idle brake (timer and speed), which this port does not model. */
typedef struct {
    uint8_t throttle, steer, brake_left, brake_right;
    int idle;
} gd_layer_t;

static double gd_dec(uint8_t b)
{
    return b == 255 ? 1.0 : (double)b / 128.0 - 1.0;
}

static uint8_t gd_enc_axis(double x)
{
    return (uint8_t)(int)fmax(0.0, fmin(255.0, x * 128.0 + 128.0));
}

static uint8_t gd_enc_brake(double x)
{
    return (uint8_t)(int)fmax(0.0, fmin(255.0, x * 256.0));
}

static gd_layer_t gd_layer(uint8_t throttle, uint8_t steer_byte, int reverse_gear, int boost, int separate)
{
    double thr = gd_dec(throttle);
    const double steer = gd_dec(steer_byte);
    if (steer != 0.0 && boost) {
        if (thr >= 1e-5) {
            thr = thr < 1.0 ? 0.7 * thr + 0.3 : 1.0;
        } else {
            const double most = fmax(fabs(thr), fabs(steer));
            thr = thr >= 0.0 ? most : -most;
        }
    }
    const int idle = fabs(thr) < 1e-5;
    const double against = reverse_gear ? thr : -thr;
    const double demand = against >= 1.0 ? 1.0 : fmax(against, 0.0);
    double left = fmax(steer >= 1.0 ? 1.0 : fmax(steer, 0.0), demand);
    double right = fmax(steer <= -1.0 ? 1.0 : fmax(-steer, 0.0), demand);
    if (!separate)
        left = right = fmax(left, right);
    return (gd_layer_t){gd_enc_axis(thr), steer_byte, gd_enc_brake(left), gd_enc_brake(right), idle};
}
