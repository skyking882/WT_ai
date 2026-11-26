/* ground_fire.inc (s49, s50): the fire control (turret pre-aim, stabiliser, weak-spot and penetration aiming,
 * fire gate; the functions tools/check_protected.py protects), in their order of ground_brain.c, with the tables,
 * types and constants only they use. s50: public like the rest of the AI: the game is reached only through
 * ground_host.h (single facts, sequences, gh_guard thunks: tools_split_fire_table.py), the bodies are otherwise
 * the s49 bodies (tools_split_fire_check.py folds the s50 bodies back into them). Included by ground_brain.c
 * where the last of them was. */

/* ==== preaim_new.c : replacement for the idle turret pre-aim in ground_brain.c ====
 *
 * REPLACES, in ground_brain.c, everything from the line
 *     #define PREAIM_RATE 25.0f       / * deg/s * /
 * down to and including the closing brace of
 *     static void preaim(void *host, u8 *unit, lane_seat_t *s, int seat, uint64_t ms)
 * (i.e. the PREAIM_* defines, yaw_sign / yaw_votes / turret_log_next / turret_csv /
 * last_yaw / last_yaw_ms statics, wrap180() and preaim()). The comment block above
 * "#define PREAIM_RATE" may stay. The call site in lanes_tick() is unchanged:
 *     preaim(host, unit, s, seat, ms);
 * Uses only things that already exist above it in the file: cfg.preaim, cfg.team,
 * cfg.log_dir, lanes[], seen_by[], hit_dir[], contact_get(), zone_owner(), dist2d(),
 * open_csv(), event(), t0, lane_seat_t (x, z, fx, fz, sx, sz, lane, army, task,
 * task_tgt, gt_slot, born_ms).
 *
 * Behaviour (design rule: idle turrets must not sweep or twitch):
 *  - never writes while the gunner has a target (s->gt_slot >= 0), for PREAIM_IDLE_MS
 *    after it lost it, or while this bot saw an enemy tank within PREAIM_IDLE_MS;
 *  - no sinusoidal sweep. The turret holds a gaze: a world bearing taken from
 *    hit direction > intercept target > nearest team contact (600 m) > the lane's
 *    point when it is not ours > the approach direction (start -> point);
 *    with the yaw sign still unknown it only centres the turret (0 deg is the
 *    same under both signs);
 *  - a different gaze source must persist PREAIM_SWITCH_MS (3 s) before the gaze
 *    moves to it (a fresh hit bearing switches at once: PREAIM_HIT_NOW);
 *  - every 6-10 s the gaze point shifts by at most PREAIM_SHIFT_DEG (20 deg) and
 *    stays within +-PREAIM_SHIFT_DEG of the source bearing (+-PREAIM_THREAT_DEG
 *    when the source is a concrete threat);
 *  - turns at <= PREAIM_RATE (12 deg/s); once on the gaze it stops writing until
 *    the error exceeds PREAIM_START_DEG (hull turning / driving), so there are no
 *    per-tick micro corrections;
 *  - every written value is wrap180'd (old bug: the turret yaw left +-180, 8 % of samples);
 *  - turrets with limited traverse (yaw span < 359 deg in the turret limits record,
 *    turret_yaw_limits) are clamped to their sector and never wrap. */
#define PREAIM_RATE 12.0f       /* deg/s, idle */
#define PREAIM_IDLE_MS 2000
#define PREAIM_SWITCH_MS 3000   /* gaze-source hysteresis */
#define PREAIM_HIT_NOW 1        /* a fresh hit bearing overrides the hysteresis */
#define PREAIM_SHIFT_MIN_MS 6000
#define PREAIM_SHIFT_MAX_MS 10000
#define PREAIM_SHIFT_DEG 20.0f
#define PREAIM_THREAT_DEG 6.0f
#define PREAIM_START_DEG 3.0f   /* start turning again when this far off the gaze */
#define PREAIM_EVAL_MS 200      /* gaze source re-evaluated at 5 Hz */
static int yaw_votes;
static uint64_t turret_log_next[MAX_SEATS];
static FILE *turret_csv;
static float last_yaw[MAX_SEATS];
static uint64_t last_yaw_ms[MAX_SEATS];
#define B5_GUNNER_GAP_MS 400    /* the gunner must have had no target this long */
#define PROBE_MS 100
#define PROBE_MAX_ROWS 6000
#define SPEED_YAW_DEFAULT 15.0f /* the BLK default when the weapon-AI params are missing */
static volatile long aim_calls[MAX_SEATS], gate_blocks[MAX_SEATS];
static float stab_sum[MAX_SEATS];
static int probe_seat_now, probe_rows;
static u8 *probe_unit;
static uint64_t probe_next;
static FILE *probe_csv;
typedef struct {
    uint64_t born_ms;           /* the life this state belongs to */
    int key;                    /* current source: kind * 100 + sub, 0 none */
    float brg;                  /* its world bearing, deg */
    int cand_key;               /* last evaluation */
    float cand_brg;
    int pend_key;               /* a different source waiting out the hysteresis */
    uint64_t pend_ms, eval_next;
    float off;                  /* gaze shift around brg, deg */
    uint64_t off_next;
    int moving;                 /* turning towards the gaze (0: holding, not writing) */
    uint64_t busy_ms;           /* last time the gunner had a target */
    float hull_prev;            /* hull world yaw at the previous tick (A1) */
} gaze_t;
static gaze_t gaze[MAX_SEATS];
/* Turret work of one tick; returns what it did (turret.csv turret_mode) and the
 * turret angle change it wrote in *applied. */
static const char *preaim_body(void *host, u8 *unit, lane_seat_t *s, int seat, uint64_t ms, u8 *turrets, u8 *rec,
                               float *applied)
{
    float raw = gh_turret_yaw(rec);
    float cur = wrap180(raw);
    float hull = atan2f(s->fz, s->fx) * 57.29578f;
    unsigned slot = (unsigned)seat & 63;
    const seen_t *seen = &seen_by[slot];
    gaze_t *g = &gaze[seat];
    int fresh = 0;
    if (g->born_ms != s->born_ms) {     /* new life */
        memset(g, 0, sizeof *g);
        g->born_ms = s->born_ms;
        g->pend_key = -1;
        fresh = 1;
    }
    float dt = !fresh && last_yaw_ms[seat] && ms > last_yaw_ms[seat] ? (float)(ms - last_yaw_ms[seat]) / 1000.0f : 0.0f;
    if (dt > 0.5f)
        dt = 0.0f;                      /* not ticked for a while: no rates from this gap */
    float native = dt > 0 ? wrap180(cur - last_yaw[seat]) : 0.0f;  /* the turret's own move since the last tick */
    float rate = dt > 0 ? fabsf(native) / dt : 999.0f;
    float dhull = dt > 0 ? wrap180(hull - g->hull_prev) : 0.0f;
    if (dt > 0)
        s->hull_rate = 0.7f * s->hull_rate + 0.3f * fabsf(dhull) / dt;
    g->hull_prev = hull;
    last_yaw[seat] = cur;
    last_yaw_ms[seat] = ms;
    float speed_yaw = SPEED_YAW_DEFAULT, sp_pitch = 0, cone = 0;
    if (!weapon_ai_read(unit, 0, &speed_yaw, &sp_pitch, &cone) || !(speed_yaw > 1.0f && speed_yaw < 200.0f))
        speed_yaw = SPEED_YAW_DEFAULT;
    float tlo = -180.0f, thi = 180.0f;
    int tlim = turret_yaw_limits(turrets, 0, &tlo, &thi);
    if (s->gt_slot >= 0)
        g->busy_ms = ms;
    else
        emu_pitch(unit, seat, turrets, rec, 0.0f, 0.0f, 0);    /* no target: drop the elevation reference */
    int seeing = ms - seen->ms <= PREAIM_IDLE_MS
                 && (!cmd_is_post(s) || dist2d(s->x, s->z, seen->x, seen->z) < CMD_POST_SELF_DEFENSE_M
                     || cmd_post_in_arc(s, seen->x, seen->z));
    int idle = !seeing && s->gt_slot < 0 && (!g->busy_ms || ms - g->busy_ms > PREAIM_IDLE_MS);

    /* Learn the sign from gunners that are on target. */
    if (seeing && yaw_sign == 0 && rate < 3.0f) {
        float rel = wrap180(atan2f(seen->z - s->z, seen->x - s->x) * 57.29578f - hull);
        float ep = fabsf(wrap180(cur - rel)), em = fabsf(wrap180(cur + rel));
        if (ep < 8.0f && em > 20.0f)
            ++yaw_votes;
        else if (em < 8.0f && ep > 20.0f)
            --yaw_votes;
        if (yaw_votes >= 6 || yaw_votes <= -6) {
            yaw_sign = yaw_votes > 0 ? 1 : -1;
            event(host, "preaim: turret yaw sign learnt: %+d", yaw_sign);
        }
    }

    /* A1: the gunner has a target -> stabilise against the hull's turn.
     * 2026-11-11: only an emulation of the vehicle's own horizontal
     * stabiliser now: none without one in the data (cls flag stab_h), none
     * while the engine stabilises it itself (its horizontal flag, see stab_native_on),
     * none over its horizontal speed limit. */
    if (s->gt_slot >= 0) {
        g->moving = 0;
        g->pend_key = -1;
        g->eval_next = 0;
        /* stab2: emulation only where the engine does not stabilise
         * (stab_emu_flags: the gun's own stab object, its limits, the drive). */
        int ef = stab_emu_flags(unit);
        emu_pitch(unit, seat, turrets, rec, dt, sp_pitch, (ef & 2) != 0);
        const char *vmode = (ef & 2) ? "stab_v" : "gunner";
        if (!cfg.stab || !yaw_sign || !(dt > 0) || dt > STAB_MAX_DT || fabsf(dhull) < 0.02f)
            return (stab_flags(unit) & 1) ? "native" : vmode;
        if (stab_flags(unit) & 1)
            return "native";            /* the engine does it */
        if (!(ef & 1))
            return vmode;               /* no horizontal stabiliser, over its limit, or emulation off */
        emu_t *em = emu_get(unit, seat);
        float comp = -(float)yaw_sign * dhull * (em ? em->om_h : 1.0f);  /* keeps hull + sign * A0 fixed in the world */
        if (em && em->spd_y > speed_yaw)
            speed_yaw = em->spd_y;
        float lim = speed_yaw * dt;
        float lo_s = -lim - native, hi_s = lim - native; /* |native + shift| <= speedYaw * dt */
        if (lo_s > 0.0f)
            lo_s = 0.0f;                                /* never push against the gunner's own move */
        if (hi_s < 0.0f)
            hi_s = 0.0f;
        float shift = comp < lo_s ? lo_s : comp > hi_s ? hi_s : comp;
        float next = turret_clamp(tlim, tlo, thi, cur + shift);
        gh_turret_set_yaw(rec, next);
        gh_turret_set_ext_yaw(rec, turret_clamp(tlim, tlo, thi, gh_turret_ext_yaw(rec) + comp));
        gh_turret_set_cmd_yaw(rec, turret_clamp(tlim, tlo, thi, gh_turret_cmd_yaw(rec) + comp));
        last_yaw[seat] = next;
        *applied = wrap180(next - cur);
        stab_sum[seat] += fabsf(*applied);
        return (ef & 2) ? "stab_hv" : "stab";
    }

    /* B5: own eyes on an enemy, the gunner not on it yet -> keep turning to it
     * at the vehicle's own traverse speed (the engaged contact first). */
    if (seeing && (!g->busy_ms || ms - g->busy_ms > B5_GUNNER_GAP_MS)) {
        g->moving = 0;
        g->pend_key = -1;
        g->eval_next = 0;
        if (!cfg.preaim || !yaw_sign)
            return "seen";
        float ex = seen->x, ez = seen->z, kx, kz, kage;
        if (s->mode == M_ENGAGE && s->eng >= 0 && contact_get(s->army, seat, s->eng, ms, &kx, &kz, &kage) == 2) {
            ex = kx;
            ez = kz;
        }
        float want = turret_clamp(tlim, tlo, thi, (float)yaw_sign * wrap180(atan2f(ez - s->z, ex - s->x) * 57.29578f - hull));
        float delta = tlim ? want - (cur < tlo ? tlo : cur > thi ? thi : cur) : wrap180(want - cur);
        float lim = speed_yaw * (dt > 0 ? dt : 0.04f) - fabsf(native);
        if (lim <= 0.0f || fabsf(delta) < 0.3f)
            return "b5";
        float next = turret_clamp(tlim, tlo, thi, cur + (fabsf(delta) <= lim ? delta : delta > 0 ? lim : -lim));
        gh_turret_set_ext_yaw(rec, next);
        gh_turret_set_yaw(rec, next);
        gh_turret_set_cmd_yaw(rec, next);
        last_yaw[seat] = next;
        *applied = wrap180(next - cur);
        return "b5";
    }
    if (!idle || !cfg.preaim) {
        /* The gunner owns the turret. Afterwards start from wherever it left it. */
        g->moving = 0;
        g->pend_key = -1;
        g->eval_next = 0;
        return "gunner";
    }

    /* 1. Gaze source, with hysteresis. */
    if (ms >= g->eval_next) {
        g->eval_next = ms + PREAIM_EVAL_MS;
        g->cand_brg = hull;
        g->cand_key = gaze_source(host, s, seat, ms, &g->cand_brg);
        int switch_now = 0;
        if (g->cand_key == g->key) {
            g->brg = g->cand_brg;           /* the source itself may move */
            g->pend_key = -1;
        } else if (g->key == 0 || (PREAIM_HIT_NOW && g->cand_key / 100 == GZ_HIT)) {
            switch_now = 1;
        } else if (g->pend_key != g->cand_key) {
            g->pend_key = g->cand_key;
            g->pend_ms = ms;
        } else if (ms - g->pend_ms >= PREAIM_SWITCH_MS) {
            switch_now = 1;
        }
        if (switch_now) {
            g->key = g->cand_key;
            g->brg = g->cand_brg;
            g->pend_key = -1;
            g->off = 0.0f;                  /* look straight at the new source first */
            g->off_next = ms + PREAIM_SHIFT_MIN_MS
                          + (uint64_t)(gaze_rand() * (float)(PREAIM_SHIFT_MAX_MS - PREAIM_SHIFT_MIN_MS));
        }
    }
    int kind = g->key / 100;
    if (kind == GZ_NONE)
        return "idle";

    /* 2. Every 6-10 s shift the gaze point a little. */
    if (kind != GZ_FWD && ms >= g->off_next) {
        float lim = kind >= GZ_POST ? PREAIM_THREAT_DEG : PREAIM_SHIFT_DEG;
        float off = g->off + (gaze_rand() * 2.0f - 1.0f) * PREAIM_SHIFT_DEG;
        g->off = off > lim ? lim : off < -lim ? -lim : off;
        g->off_next = ms + PREAIM_SHIFT_MIN_MS
                      + (uint64_t)(gaze_rand() * (float)(PREAIM_SHIFT_MAX_MS - PREAIM_SHIFT_MIN_MS));
    }

    /* 3. Turret angle for it (hull frame, the turret's own sign). */
    float want = kind == GZ_FWD ? 0.0f : (float)yaw_sign * wrap180(g->brg + g->off - hull);
    float delta;
    if (tlim) {
        want = want < tlo ? tlo : want > thi ? thi : want;
        float c = cur < tlo ? tlo : cur > thi ? thi : cur;
        delta = want - c;                   /* inside the sector: never through the stop */
    } else {
        delta = wrap180(want - cur);
    }

    /* 4. Turn at <= PREAIM_RATE; once there, hold without writing. */
    if (!g->moving) {
        if (fabsf(delta) < PREAIM_START_DEG)
            return "idle";
        g->moving = 1;
    }
    float step = PREAIM_RATE * (dt > 0 && dt < 0.5f ? dt : 0.04f);
    float next;
    if (fabsf(delta) <= step) {
        next = cur + delta;
        g->moving = 0;
    } else {
        next = cur + (delta > 0 ? step : -step);
    }
    next = wrap180(next);
    gh_turret_set_ext_yaw(rec, next);
    gh_turret_set_yaw(rec, next);
    gh_turret_set_cmd_yaw(rec, next);
    last_yaw[seat] = next;
    *applied = wrap180(next - cur);
    return "gaze";
}
static void preaim(void *host, u8 *unit, lane_seat_t *s, int seat, uint64_t ms)
{
    if (!cfg.preaim && !cfg.stab)
        return;
    u8 *turrets = gh_unit_turrets(unit);
    u8 *rec = turrets ? gh_turrets_records(turrets) : NULL;
    if (!rec)
        return;
    float raw = gh_turret_yaw(rec);
    if (!isfinite(raw))
        return;
    float f80 = gh_turret_ext_yaw(rec), fd8 = gh_turret_cmd_yaw(rec);
    float hull = atan2f(s->fz, s->fx) * 57.29578f, applied = 0.0f;
    const char *mode = preaim_body(host, unit, s, seat, ms, turrets, rec, &applied);
    unsigned slot = (unsigned)seat & 63;

    if (turret_csv == NULL && cfg.log_dir[0])
        turret_csv = open_csv("turret.csv", "t,seat,ext_yaw,yaw,cmd_yaw,ext_pitch,pitch,cmd_pitch,hull_deg,seen_age_s,yaw_sign,"
                                            "aim_calls,gate_blocks,stab_deg,turret_mode,hull_rate,"
                                            "stab_h_on,stab_v_on,cls_stab_h_kmh,cls_stab_v_kmh,speed_mps,"
                                            "emu_h,emu_v,emu_v_deg,emu_v_ticks");
    if (turret_csv && ms >= turret_log_next[seat]) {
        int sf = stab_flags(unit), ef = stab_emu_flags(unit);
        emu_t *em = emu_get(unit, seat);
        const vcls_t *vc = active_cls(seat, unit);
        turret_log_next[seat] = ms + 1000;
        fprintf(turret_csv, "%.1f,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%ld,%ld,%.1f,%s,%.1f,%d,%d,%.0f,%.0f,%.1f,%d,%d,%.2f,%ld\n",
                (double)(ms - t0) / 1000.0, seat, f80, raw, fd8, gh_turret_ext_pitch(rec), gh_turret_pitch(rec),
                gh_turret_cmd_pitch(rec), hull, (double)(ms - seen_by[slot].ms) / 1000.0, yaw_sign,
                gh_atomic_swap(&aim_calls[seat], 0), gh_atomic_swap(&gate_blocks[seat], 0),
                stab_sum[seat], mode, s->hull_rate, sf & 1, (sf >> 1) & 1,
                vc->ok ? vc->stab_h_kmh : -2.0f, vc->ok ? vc->stab_v_kmh : -2.0f,
                sqrtf(s->vx * s->vx + s->vz * s->vz), ef & 1, (ef >> 1) & 1, em ? (double)em->v_sum : 0.0,
                em ? em->v_ticks : 0L);
        stab_sum[seat] = 0.0f;
        if (em) {
            em->v_sum = 0.0f;
            em->v_ticks = 0;
        }
        fflush(turret_csv);
    }

    /* A7: one seat at 10 Hz. */
    if (probe_rows < PROBE_MAX_ROWS && cfg.log_dir[0]) {
        if (probe_seat_now && lseat[probe_seat_now].unit != probe_unit)
            probe_seat_now = 0;     /* that life is over: pick again */
        if (!probe_seat_now && (cfg.probe_seat > 0 ? seat == cfg.probe_seat : s->gt_slot >= 0)) {
            probe_seat_now = seat;
            probe_unit = unit;
            event(host, "probe: 10 Hz turret probe on seat %d", seat);
        }
        if (seat == probe_seat_now && ms >= probe_next) {
            probe_next = ms + PROBE_MS;
            if (!probe_csv)
                probe_csv = open_csv("turret10.csv", "t,seat,yaw_before,yaw,cmd_yaw_before,cmd_yaw,ext_yaw,hull_deg,hull_rate,"
                                                     "gunner_slot,mode,applied_deg,gun_err_deg,speed_mps");
            if (probe_csv) {
                char e[24] = "";
                int gt = s->gt_slot;
                if (yaw_sign && gt >= 0 && slot_alive[gt] && slot_unit[gt]) {
                    const float *p = gh_unit_pos(slot_unit[gt]);
                    snprintf(e, sizeof e, "%.2f", wrap180(hull + (float)yaw_sign * gh_turret_yaw(rec)
                                                          - atan2f(p[2] - s->z, p[0] - s->x) * 57.29578f));
                }
                ++probe_rows;
                fprintf(probe_csv, "%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%s,%.2f,%s,%.1f\n",
                        (double)(ms - t0) / 1000.0, seat, raw, gh_turret_yaw(rec), fd8, gh_turret_cmd_yaw(rec),
                        gh_turret_ext_yaw(rec), hull, s->hull_rate, gt, mode, applied, e,
                        sqrtf(s->vx * s->vx + s->vz * s->vz));
                fflush(probe_csv);
            }
        }
    }
}
/* ---- Native gun stabiliser for bot tanks (2026-11-11) -------------------
 * The engine stabilises a gun only when the gun has a stabiliser object (the
 * gunStabilizer data of the vehicle: horizontal / vertical axis, free mode,
 * forced, omega multipliers, speed limits) and the tank's "stabiliser allowed"
 * test passes; that test reads a byte of the vehicle controls that only the
 * network reader writes (from a human client), so bots never get the
 * stabiliser. Per axis it then also needs: no broken stabiliser part, speed
 * under the limit, crew multiplier > 0; vertical: the vertical drive not
 * blocked. The result is stored in the turret record (the engine-stabiliser
 * flags) and used by the turret drive.
 * So we write the allowed byte = 1 every tick for every bot tank (a tank
 * without gunStabilizer has no stab object: nothing changes) after checking
 * that the game code is what was read statically (gh_stabiliser_code_matches),
 * and read the engine-stabiliser flags back as "the engine stabilises this gun
 * now". WT_GROUND_STAB_NATIVE=0: no write.
 * 2026-11-11: that alone did nothing (the flags stayed 0 in two test battles): after
 * the allowed byte, the test returns "not a native bot" for units without the
 * exemption flag, i.e. never for native bots. The launcher's stabiliser
 * patch removes that bot test (gh_stabiliser_patch_present). */
static int stabn_state;         /* 0 unchecked, 1 on, 2 off */
static int stabn_live;          /* stab2: both host patches present, the engine stabilises bots */
/* s50: the guarded block of stab_native_on, run under the host's fault guard (gh_guard, ground_host.h). */
typedef struct { void *host; u8 *unit; } stab_native_on_ctx_t;
static void stab_native_on_run(void *ctx)
{
    stab_native_on_ctx_t *const context = ctx;
    void *const host = context->host;
    u8 *const unit = context->unit;
    {
        if (!stabn_state) {
            int ok = gh_stabiliser_code_matches(host);
            stabn_state = ok ? 1 : 2;
            event(host, ok ? "stab: native gun stabiliser switched on for bot tanks (stabiliser-allowed byte = 1)"
                            : "stab: off (the game code the stabiliser write relies on is not what was read statically)");
            if (!ok)
                goto leave;
            /* stab2: both host patches (the launcher's stabiliser patch: the two
             * bot-test masks set to 0) -> the engine decides per axis
             * and the DLL emulation stays out. */
            stabn_live = gh_stabiliser_patch_present(host);
            event(host, stabn_live ? "stab: host patches present: engine stabiliser for bots, DLL emulation idle"
                                    : "stab: host patches missing (mask a %02x, mask b %02x): DLL emulation stabilises",
                  gh_stab_patch_mask_a(host), gh_stab_patch_mask_b(host));
        }
        u8 *controls = gh_unit_controls(unit);
        if (controls && gh_controls_stab_allowed(controls) != 1)
            gh_controls_set_stab_allowed(controls, 1);
    }
leave:;
}
static void stab_native_on(void *host, u8 *unit)
{
    if (!cfg.stab_native || stabn_state == 2)
        return;
    stab_native_on_ctx_t context = {host, unit};
    if (gh_guard(stab_native_on_run, &context)) {
        stabn_state = 2;
    }
}
/* s50: the guarded block of stab_flags, run under the host's fault guard (gh_guard, ground_host.h). */
typedef struct { u8 *unit; int f; } stab_flags_ctx_t;
static void stab_flags_read(void *ctx)
{
    stab_flags_ctx_t *const context = ctx;
    u8 *const unit = context->unit;
    int f = context->f;
    {
        u8 *turrets = gh_unit_turrets(unit);
        u8 *rec = turrets && gh_turrets_count(turrets) ? gh_turrets_records(turrets) : NULL;
        if (rec)
            f = (gh_turret_engine_stab_h(rec) == 1 ? 1 : 0) | (gh_turret_engine_stab_v(rec) == 1 ? 2 : 0);
    }
    context->f = f;
}
/* What the engine's stabiliser does for the main gun right now: bit 0
 * horizontal, bit 1 vertical (turret record 0, its engine-stabiliser flags). */
static int stab_flags(u8 *unit)
{
    int f = 0;
    stab_flags_ctx_t context = {unit, f};
    const int faulted = gh_guard(stab_flags_read, &context);
    f = context.f;
    if (faulted) {
        f = 0;
    }
    return f;
}
/* s50: the guarded block of stab_diag, run under the host's fault guard (gh_guard, ground_host.h). */
typedef struct { u8 *unit; char *out; size_t n; } stab_diag_ctx_t;
static void stab_diag_read(void *ctx)
{
    stab_diag_ctx_t *const context = ctx;
    u8 *const unit = context->unit;
    char *const out = context->out;
    const size_t n = context->n;
    {
        u8 *controls = gh_unit_controls(unit), *pl = gh_unit_player(unit);
        u8 *turrets = gh_unit_turrets(unit);
        u8 *rec = turrets && gh_turrets_count(turrets) ? gh_turrets_records(turrets) : NULL;
        u8 *gun = rec ? gh_turret_gun(rec) : NULL;
        u8 *props = gun ? gh_gun_props(gun) : NULL;
        u8 *so = props ? gh_gun_stab_object(props) : NULL;
        int len = snprintf(out, n, "sim_mode %02x stab_exempt %d player_flags %02x stab_allowed %d kind %d | drive_intact %d state_flag_a %d vertical_blocked %d",
                           gh_unit_sim_mode(unit), gh_unit_stab_exempt(unit) ? 1 : 0, pl ? gh_player_flags(pl) : 0xff,
                           controls ? gh_controls_stab_allowed(controls) : -1, (int)gh_unit_kind(unit),
                           rec ? gh_turret_drive_intact(rec) : -1, rec ? gh_turret_state_flag_a(rec) : -1, rec ? gh_turret_vertical_blocked(rec) : -1);
        if (so && len > 0 && (size_t)len < n)
            snprintf(out + len, n - (size_t)len, " | obj h %d v %d free %d force %d omega %.2f/%.2f lim %.1f/%.1f m/s spd %.1f/%.1f",
                     gh_stab_axis_h(so), gh_stab_axis_v(so), gh_stab_free_mode(so), gh_stab_forced(so), (double)gh_stab_omega_h(so), (double)gh_stab_omega_v(so),
                     (double)gh_stab_limit_h(so), (double)gh_stab_limit_v(so), (double)gh_stab_speed_yaw(so),
                     (double)gh_stab_speed_pitch(so));
        else if (len > 0 && (size_t)len < n)
            snprintf(out + len, n - (size_t)len, " | no stab object");
        float plo = 0.0f, phi = 0.0f;
        len = (int)strlen(out);
        if (turrets && turret_pitch_limits(turrets, 0.0f, &plo, &phi) && (size_t)len < n)
            snprintf(out + len, n - (size_t)len, " | pitch limits %.1f..%.1f (pitch %.2f)", (double)plo, (double)phi,
                     rec ? (double)gh_turret_pitch(rec) : 0.0);
    }
}
static void stab_diag(u8 *unit, char *out, size_t n)
{
    snprintf(out, n, "?");
    stab_diag_ctx_t context = {unit, out, n};
    if (gh_guard(stab_diag_read, &context)) {
        snprintf(out, n, "fault");
    }
}
/* s50: the guarded block of stab_emu_flags, run under the host's fault guard (gh_guard, ground_host.h). */
typedef struct { u8 *unit; emu_t *e; int nf; float v; int f; } stab_emu_flags_ctx_t;
static void stab_emu_flags_read(void *ctx)
{
    stab_emu_flags_ctx_t *const context = ctx;
    u8 *const unit = context->unit;
    emu_t *const e = context->e;
    const int nf = context->nf;
    const float v = context->v;
    int f = context->f;
    {
        u8 *turrets = gh_unit_turrets(unit);
        u8 *rec = turrets && gh_turrets_count(turrets) ? gh_turrets_records(turrets) : NULL;
        if (rec && gh_turret_drive_intact(rec)) {
            if (cfg.stab && e->has_h && e->om_h > 0.0f && !(nf & 1) && (e->lim_h < 0.0f || v < e->lim_h))
                f |= 1;
            if (e->has_v && e->om_v > 0.0f && !(nf & 2) && !gh_turret_vertical_blocked(rec) && (e->lim_v < 0.0f || v < e->lim_v))
                f |= 2;
        }
    }
    context->f = f;
}
/* Which axes the emulation stabilises for this bot now: bit 0 horizontal,
 * bit 1 vertical (never an axis the engine already stabilises). */
static int stab_emu_flags(u8 *unit)
{
    if (!cfg.stab_emu || stabn_live || !unit || !yaw_sign)
        return 0;
    int seat = bot_seat(unit);
    if (seat < 0 || seat >= MAX_SEATS || lseat[seat].unit != unit)
        return 0;
    emu_t *e = emu_get(unit, seat);
    if (!e || !(e->has_h || e->has_v))
        return 0;
    int f = 0, nf = stab_flags(unit);
    float v = sqrtf(lseat[seat].vx * lseat[seat].vx + lseat[seat].vz * lseat[seat].vz);
    stab_emu_flags_ctx_t context = {unit, e, nf, v, f};
    const int faulted = gh_guard(stab_emu_flags_read, &context);
    f = context.f;
    if (faulted) {
        f = 0;
    }
    return f;
}
/* ==== Weak-spot aiming for bot tank gunners (2026-11-10) ====================
 *
 * INTEGRATION (history): this block was pasted into ground_brain.c above the path
 * diagnosis entry; it uses cfg, t0, the config init, open_csv, event, bot_seat,
 * is_tank and ground_height from above it. The switches are read from the
 * process environment (the host inherits route.py's environment, like
 * WT_GROUND_AI_CONFIG):
 *     WT_GROUND_WEAKSPOT=0          off (the launcher's aim hook is not installed either)
 *     WT_GROUND_WEAKSPOT_SKILL=0    disables weak-part picks; used as the pick
 *                                 probability in the legacy no-penetration mode
 *     WT_GROUND_WEAKSPOT_SCATTER=0.5 share of the engine's own bot scatter that is kept
 * The host calls it from the launcher's aim hook.
 *
 * What the engine does (verified statically; the notes are in ground_host.c):
 *  - The gunner evaluator asks the target for its world point; the native
 *    tank point is near its model origin.
 *  - Then it adds the weapon-AI error, target lead and the shooter's bot
 *    scatter (gh_unit_aim_scatter: every 5 s x,z = +-1 m, y = 0..0.2 m,
 *    times the engine's scale against bots / against humans) and solves the
 *    ballistics for that point.
 *  So a bot aims at the ground-level centre of the hull. This entry moves the
 *  point the evaluator got onto a damage-model part.
 *
 * Damage-model parts (what the game's aim-node names are matched against):
 *   the unit's damage model -> its collision object -> the collision header
 *   (aim_coll); node count, nodes and node names (gh_coll_*; names only when
 *   the header has them);
 *   box = gh_node_box(coll, out, n): model space with the node's
 *          current transform (turret parts follow the turret); min > max when
 *          the node is gone. World = the unit matrix (gh_unit_frame, 3x3 + position).
 * Names are the usual *_dm parts: ammo_body_NN_dm, ammo_turret_NN_dm,
 * engine_dm, transmission_dm, driver_dm, gunner_dm, loader_dm, commander_dm,
 * cannon_breech_dm, gun_barrel_dm, track_l_dm, track_r_dm ...
 */
enum {
    AIM_AMMO, AIM_ENGINE, AIM_TRANS, AIM_DRIVER, AIM_GUNNER, AIM_LOADER, AIM_COMMANDER,
    AIM_BREECH, AIM_BARREL, AIM_TRACK, AIM_NCLS
};
enum { ASPECT_FRONT, ASPECT_SIDE, ASPECT_REAR };
#define AIM_FRONT_DEG 35.0f
#define AIM_REAR_DEG 145.0f
#define AIM_MAX_PARTS 128
#define AIM_MAX_NODES 4096
#define AIM_WAYS 4
#define AIM_HOLD_MS 800         /* a pick is kept 0.8-1.2 s (re-pick about once a second) */
#define AIM_BOX_MS 300          /* the picked part's box is re-read this often */
#define AIM_REBUILD_MS 30000
static const char *const aim_cls_name[AIM_NCLS] = {"ammo", "engine", "transmission", "driver", "gunner", "loader",
                                                    "commander", "breech", "barrel", "track"};
/* Pick weights per aspect: front = lower front plate area (driver, hull ammo
 * beside him) and the turret crew; side = ammunition first; rear = engine. */
static const float aim_weight[3][AIM_NCLS] = {
    /* ammo engine trans driver gunner loader commander breech barrel track */
    {3.0f, 0.0f, 0.0f, 3.0f, 2.0f, 1.0f, 1.0f, 2.0f, 2.0f, 1.0f},
    {5.0f, 2.0f, 1.0f, 1.0f, 2.0f, 2.0f, 1.0f, 0.5f, 1.0f, 2.0f},
    {3.0f, 4.0f, 2.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 2.0f},
};
typedef struct { uint16_t node; uint8_t cls; } aim_part_t;
typedef struct {
    u8 *unit, *hdr;
    uint64_t built_ms, life;
    int n, per_cls[AIM_NCLS];
    aim_part_t part[AIM_MAX_PARTS];
    /* review_5003: hull armour extent, model space (the unit box, gh_unit_bbox, includes
     * the barrel: x1 = 7.07 m in 5003). ext_kind 2 = from the body_* /
     * superstructure_* / hull_* armour nodes (height up to the turret_* nodes),
     * 1 = from the internal parts' boxes + 0.25 m, 0 = the unit box. */
    int ext_kind, n_hull, n_turret;
    float ext_lo[3], ext_hi[3];
} aim_target_t;
typedef struct {
    u8 *target;
    uint64_t until_ms, box_ms, target_life, shooter_life, assessed_ms;
    int aspect, weak, cls, node, opportunity;
    float half[3], gate_w, gate_h;   /* weak: 0 mass, 1 part (old pick), 2 penetration test */
    float centre[3], jit[3];    /* target model space */
    /* penetration pick (weak == 2), see pen_pick */
    float bearing, dist, score, pen, speed;
    int res, cands, tested, counts[6];
    int centred, cache;         /* A5 centre point; pick came from a shared job */
    float phit, sigma;          /* hit probability of the pick, gun error used (deg) */
    const char *why;            /* why the centre (A5) */
} aim_choice_t;
static aim_target_t aim_targets[64];
static aim_choice_t aim_choices[MAX_SEATS][AIM_WAYS];
static int aim_state;           /* 0 not read, 1 on, 2 off */
static volatile long aim_faults;
static uint64_t aim_log_ms[MAX_SEATS], aim_skip_ms[MAX_SEATS], aim_retry_ms[MAX_SEATS];
static FILE *aim_csv;
static float aim_rand(void)     /* 0..1 */
{
    aim_rng ^= aim_rng << 13;
    aim_rng ^= aim_rng >> 17;
    aim_rng ^= aim_rng << 5;
    return (float)(aim_rng >> 8) * (1.0f / 16777216.0f);
}
static float aim_env(const char *name, float fallback)
{
    char v[32];
    unsigned long n = gh_env(name, v, sizeof v);
    return n && n < sizeof v ? (float)atof(v) : fallback;
}
static int aim_prefix(const char *s, const char *p)
{
    while (*p)
        if (*s++ != *p++)
            return 0;
    return 1;
}
/* "<crew>_dm" or "<crew>_NN_dm"; keeps commander_panoramic_sight_dm etc. out. */
static int aim_crew(const char *s, const char *crew)
{
    size_t n = strlen(crew);
    if (!aim_prefix(s, crew) || s[n] != '_')
        return 0;
    s += n + 1;
    while (*s >= '0' && *s <= '9')
        ++s;
    if (s[-1] != '_' && *s == '_')
        ++s;
    return s[0] == 'd' && s[1] == 'm' && s[2] == 0;
}
static int aim_classify(const char *s)
{
    if (aim_prefix(s, "ammo")) return AIM_AMMO;
    if (aim_prefix(s, "engine")) return AIM_ENGINE;
    if (aim_prefix(s, "transmission")) return AIM_TRANS;
    if (aim_crew(s, "cannon_breech")) return AIM_BREECH;
    if (aim_crew(s, "gun_barrel")) return AIM_BARREL;
    if (aim_crew(s, "track_l") || aim_crew(s, "track_r")) return AIM_TRACK;
    if (aim_crew(s, "driver")) return AIM_DRIVER;
    if (aim_crew(s, "gunner")) return AIM_GUNNER;
    if (aim_crew(s, "loader")) return AIM_LOADER;
    if (aim_crew(s, "commander")) return AIM_COMMANDER;
    return -1;
}
/* Collision header of a unit's damage model, as the game itself reaches it. */
static u8 *aim_coll(u8 *unit, u8 **hdr)
{
    u8 *dm = gh_unit_damage_model(unit);
    u8 *coll = dm ? gh_dm_collision(dm) : NULL;
    *hdr = coll ? gh_collision_header(coll) : NULL;
    return *hdr ? coll : NULL;
}
/* Model-space centre and half size of node n; 0 when the node has no box. */
static int aim_node_box(void *host, u8 *coll, u8 *hdr, int n, float *centre, float *half)
{
    float b[6];
    if (n < 0 || (uint32_t)n >= gh_coll_count(hdr))
        return 0;
    gh_node_box(host, coll, b, n);   /* s45: as the game's own call sites pass it */
    for (int i = 0; i < 3; ++i) {
        if (!(b[i] <= b[i + 3]) || !(b[i] > -50.0f) || !(b[i + 3] < 50.0f))
            return 0;
        centre[i] = 0.5f * (b[i] + b[i + 3]);
        half[i] = 0.5f * (b[i + 3] - b[i]);
    }
    return 1;
}
static int aim_extent_valid(const float lo[3], const float hi[3])
{
    for (int a = 0; a < 3; ++a) {
        float span = hi[a] - lo[a];
        if (!(span > 0.5f && span < 25.0f && fabsf(lo[a] + hi[a]) < 15.0f))
            return 0;
    }
    return 1;
}
static int aim_part_group(int cls)
{
    if (cls == AIM_BARREL || cls == AIM_BREECH)
        return SHOT_FIRE;
    if (cls == AIM_TRACK || cls == AIM_ENGINE || cls == AIM_TRANS)
        return SHOT_MOBILITY;
    return SHOT_KILL;
}
static int aim_external(int cls)
{
    return cls == AIM_BARREL || cls == AIM_TRACK;
}
static aim_target_t *aim_parts(void *host, u8 *unit, uint64_t life, uint64_t ms)
{
    u8 *hdr;
    /* One entry per player slot (the first version hashed the unit pointer,
     * (unit >> 6) & 63, and two live tanks sharing an entry rebuilt it on every
     * call: 20800 "has N dm nodes" lines in one battle log). */
    u8 *player = gh_unit_player(unit);
    aim_target_t *t = &aim_targets[player ? gh_player_slot(player) & 63 : ((uintptr_t)unit >> 6) & 63];
    u8 *coll = aim_coll(unit, &hdr);
    if (!coll)
        return NULL;
    if (t->unit == unit && t->hdr == hdr && t->life == life && ms - t->built_ms < AIM_REBUILD_MS)
        return t;
    const int fresh = t->unit != unit || t->hdr != hdr || t->life != life;
    memset(t, 0, sizeof *t);
    t->unit = unit;
    t->hdr = hdr;
    t->life = life;
    t->built_ms = ms;
    t->ext_kind = -1;
    const uint32_t count = gh_coll_count(hdr);
    if (!gh_coll_has_names(hdr) || count > AIM_MAX_NODES)
        return t;
    const u8 *nodes = gh_coll_nodes(hdr);
    const char *names = gh_coll_names(hdr);
    float hlo[3] = {1e9f, 1e9f, 1e9f}, hhi[3] = {-1e9f, -1e9f, -1e9f}, ttop = -1e9f;
    float plo[3] = {1e9f, 1e9f, 1e9f}, phi[3] = {-1e9f, -1e9f, -1e9f};
    int nparts_box = 0;
    for (uint32_t n = 0; n < count; ++n) {
        const char *nm = gh_coll_node_name(names, nodes, n);
        size_t len = strlen(nm);
        float c[3], h[3];
        int hull_arm = len > 3 && !strcmp(nm + len - 3, "_dm")
                       && (aim_prefix(nm, "body_") || aim_prefix(nm, "superstructure_") || aim_prefix(nm, "hull_"));
        int turret_arm = len > 3 && !strcmp(nm + len - 3, "_dm") && aim_prefix(nm, "turret_");
        if ((hull_arm || turret_arm) && aim_node_box(host, coll, hdr, (int)n, c, h)) {
            for (int a = 0; a < 3 && hull_arm; ++a) {
                hlo[a] = fminf(hlo[a], c[a] - h[a]);
                hhi[a] = fmaxf(hhi[a], c[a] + h[a]);
            }
            t->n_hull += hull_arm;
            t->n_turret += turret_arm;
            if (turret_arm)
                ttop = fmaxf(ttop, c[1] + h[1]);
            continue;
        }
        if (len <= 3 || strcmp(nm + len - 3, "_dm"))
            continue;
        int cls = aim_classify(nm);
        if (cls < 0 || t->n >= AIM_MAX_PARTS || (cls == AIM_AMMO && t->per_cls[AIM_AMMO] >= AIM_MAX_PARTS - 24))
            continue;
        t->part[t->n++] = (aim_part_t){(uint16_t)n, (uint8_t)cls};
        ++t->per_cls[cls];
        if (!aim_external(cls) && aim_node_box(host, coll, hdr, (int)n, c, h)) {
            ++nparts_box;
            for (int a = 0; a < 3; ++a) {
                plo[a] = fminf(plo[a], c[a] - h[a]);
                phi[a] = fmaxf(phi[a], c[a] + h[a]);
            }
        }
    }
    if (t->n_hull && hhi[0] - hlo[0] > 1.5f && hhi[2] - hlo[2] > 1.0f) {
        if (ttop > hhi[1] && ttop < hhi[1] + 2.5f)
            hhi[1] = ttop;
        if (aim_extent_valid(hlo, hhi)) {
            t->ext_kind = 2;
            memcpy(t->ext_lo, hlo, sizeof hlo);
            memcpy(t->ext_hi, hhi, sizeof hhi);
        }
    }
    if (t->ext_kind < 0 && nparts_box >= 3) {
        for (int a = 0; a < 3; ++a) {
            plo[a] -= 0.25f;
            phi[a] += 0.25f;
        }
        if (aim_extent_valid(plo, phi)) {
            t->ext_kind = 1;
            memcpy(t->ext_lo, plo, sizeof plo);
            memcpy(t->ext_hi, phi, sizeof phi);
        }
    }
    static int fresh_logged;
    if (fresh && fresh_logged < 24) {
        const char *name = gh_unit_name(unit) ? (const char *)gh_unit_name(unit) : "?";
        ++fresh_logged;
        event(host, "aim: %s has %u dm nodes: ammo %d engine %d transmission %d driver %d gunner %d loader %d "
              "commander %d breech %d barrel %d track %d; armour hull %d turret %d -> extent kind %d",
              name, count, t->per_cls[AIM_AMMO], t->per_cls[AIM_ENGINE], t->per_cls[AIM_TRANS], t->per_cls[AIM_DRIVER],
              t->per_cls[AIM_GUNNER], t->per_cls[AIM_LOADER], t->per_cls[AIM_COMMANDER], t->per_cls[AIM_BREECH],
              t->per_cls[AIM_BARREL], t->per_cls[AIM_TRACK], t->n_hull, t->n_turret, t->ext_kind);
    }
    return t;
}
/* Fills c with a live part of a class the aspect asks for; 0 when the tank offers none. */
static int aim_pick(void *host, u8 *coll, aim_target_t *t, int aspect, float mid_x, aim_choice_t *c)
{
    float w[AIM_NCLS], total = 0.0f;
    for (int k = 0; k < AIM_NCLS; ++k) {
        w[k] = t->per_cls[k] ? aim_weight[aspect][k] : 0.0f;
        total += w[k];
    }
    for (int attempt = 0; attempt < 4 && total > 0.0f; ++attempt) {
        float r = aim_rand() * total;
        int cls = 0;
        while (cls < AIM_NCLS - 1 && (w[cls] <= 0.0f || r >= w[cls])) {
            r -= w[cls];
            ++cls;
        }
        if (w[cls] <= 0.0f)
            break;
        /* Some parts are gone or behind the hull centre (front: only ammunition
         * in the front half is behind the lower plate), so try a few. */
        for (int tries = 0; tries < 6; ++tries) {
            int want = (int)(aim_rand() * (float)t->per_cls[cls]), seen = 0;
            for (int i = 0; i < t->n; ++i) {
                if (t->part[i].cls != cls || seen++ != want)
                    continue;
                if (aim_node_box(host, coll, t->hdr, t->part[i].node, c->centre, c->half)
                    && !(aspect == ASPECT_FRONT && cls == AIM_AMMO && c->centre[0] < mid_x)) {
                    for (int a = 0; a < 3; ++a) {
                        float j = 0.35f * c->half[a];
                        c->jit[a] = (aim_rand() * 2.0f - 1.0f) * (j < 0.35f ? j : 0.35f);
                    }
                    c->cls = cls;
                    c->node = t->part[i].node;
                    return 1;
                }
                break;
            }
        }
        total -= w[cls];
        w[cls] = 0.0f;
    }
    return 0;
}
/* ---- Penetration-aware aim points (2026-11-10, armour pass) ----------------
 *
 * The game's own shell-vs-damage-model test, as the game's script layer sees it
 * (gh_shell_test; the notes are in ground_host.c): target entity id, shell id, test input;
 *   - the target unit is found from its entity id (gh_entity_unit, generation-checked;
 *     no unit for a stale or garbage id); it needs the unit's damage model;
 *   - the shell properties come from the game's shell registry (gh_shell_props);
 *     they hold the penetration-by-distance table (distance, 1/(next distance -
 *     distance), mm) that the game interpolates (aim_pen_at does the same);
 *   - it builds a local hit context (the unit, the shell kinetics, the
 *     penetration at the test distance), copies the game's damage-model settings
 *     (armour scale, breach factor, slope multiplier, the ricochet test
 *     thresholds, ...) and runs the game's complex critical-damage test
 *     (pass 1 and 2, the same armour solver as a real hit, all
 *     allocations local and freed: no damage is applied);
 *   - returns the result byte: 0 none (ray missed / no dm), 1 cancel, 2 touch
 *     (hit, nothing critical), 3 critical damage, 4 stop (no penetration),
 *     5 ricochet (the game's own result names). The client's crosshair
 *     indicator calls the same test per frame and paints 3 green,
 *     1/2 yellow, 4 red, 5 ricochet.
 * Input (gh_shell_test_t): speed m/s, distance m (penetration table), ray start,
 * unit direction, ray length, full (crit pass with crew / ammo checks: mode 1),
 * else the mode itself; a hit flag. The crosshair indicator uses mode 1 + flag 1
 * and a ray from aim point - R*dir with length 4R; we do the same.
 * The shell is the next round of the shooter's main gun (as the game's shot code
 * picks it): the current bullet set's rounds in the belt and the belt
 * (gh_set_rounds, gh_set_belt), belt position = shots fired from the set % count,
 * shell id = the bullet's id (or its base id when -1), muzzle speed = the bullet's
 * speed (as the crosshair indicator reads it; inferred).
 * The unit's entity id field is not known statically: the first evaluation asks
 * the host to find it in the shooter and target (gh_unit_entity_id_find). */
#define AIM_EVAL_GAP_MS 500     /* <= 2 evaluations per second per gunner */
#define AIM_RESEEK_DEG 8.0f     /* bearing change (target frame) that forces a new pick */
#define AIM_BUDGET_US_PER_MS 60.0 /* test time allowed: 6% of one core on average (WT_GROUND_PENAIM_BUDGET) */
#define AIM_BUDGET_CAP_US 12000.0
#define AIM_JITTER_M 0.3f
enum { CT_NONE, CT_CANCEL, CT_TOUCH, CT_CRIT, CT_STOP, CT_RICOCHET, CT_N };
typedef struct {
    int32_t id;
    float speed;
    const float *table;         /* [dist, k, mm] * n */
    uint32_t n;
} aim_shell_t;
typedef struct { float l[3]; int node, cls, res; float score; } aim_cand_t;
typedef struct { float c[3], h[3]; int node, cls; } aim_box_t;
/* What a part is worth when the shell reaches it (ammo detonates, crew and
 * breech disable, engine/transmission immobilise). */
static const float aim_value[AIM_NCLS] = {1.0f, 0.45f, 0.35f, 0.7f, 0.8f, 0.6f, 0.6f, 0.75f, 0.70f, 0.55f};
static const char *const aim_res_name[CT_N] = {"none", "cancel", "touch", "crit", "stop", "ricochet"};
static int aim_pen_state;       /* 0 untried, 1 on, 2 off */
static int aim_part_on;         /* s46: the old part pick (weak 1) allowed; WT_GROUND_PARTAIM=1, default off */
static int aim_test_mode = 1;
static int aim_eid_off;         /* 0 unknown, 1 found (s51: a state, not the field), -1 not found */
static int aim_eid_tries;
static volatile long aim_pen_faults;
static double aim_qpc_us, aim_budget_us = AIM_BUDGET_CAP_US, aim_budget_rate = AIM_BUDGET_US_PER_MS;
static uint64_t aim_budget_ms, aim_eval_ms[MAX_SEATS];
static struct {
    uint64_t next_ms;
    int picks[3], tests, budget_skips, no_shell, no_eid, res_hist[CT_N], pick_res[CT_N];
    int centre, jobs, cache_hits, pending, job_tests;
    double us, us_max, call_us, call_us_max;
} aim_stat;
static double aim_now_us(void)
{
    long long c;
    if (aim_qpc_us == 0.0) {
        long long f;
        gh_qpc_frequency(&f);
        aim_qpc_us = 1e6 / (double)f;
    }
    gh_qpc(&c);
    return (double)c * aim_qpc_us;
}
static u8 *aim_shell_props(void *host, int32_t id)
{
    return gh_shell_props(host, id);
}
/* The next round of the shooter's main gun (turret record 0). */
static int aim_shell(void *host, u8 *unit, aim_shell_t *sh)
{
    memset(sh, 0, sizeof *sh);
    u8 *turrets = gh_unit_turrets(unit);
    u8 *recs = turrets ? gh_turrets_records(turrets) : NULL;
    if (!recs || !gh_turrets_count(turrets))
        return 0;
    u8 *gun = gh_turret_gun(recs);
    if (!gun || gh_weapon_not_gun(host, gun))
        return 0;
    u8 *props = gh_gun_props(gun);
    uint32_t nsets = props ? gh_props_set_count(props) : 0, cur = gh_gun_current_set(gun);
    if (!props || cur >= nsets || nsets > 16)
        return 0;
    u8 *set = gh_props_set(props, cur);
    uint32_t count = gh_set_rounds(set);
    if (!count || count > 256)
        return 0;
    u8 **belt = gh_set_belt(set, count);
    u8 *b = belt ? gh_belt_round(belt, gh_set_shots(set) % count) : NULL;
    if (!b)
        return 0;
    sh->id = gh_bullet_shell_id(b);
    if (sh->id == -1)
        sh->id = gh_bullet_base_id(b);
    sh->speed = gh_bullet_speed(b);
    if (!(sh->speed > 50.0f && sh->speed < 3000.0f))
        sh->speed = 800.0f;     /* the test only uses it for the shell's energy */
    u8 *p = aim_shell_props(host, sh->id);
    if (!p)
        return 0;
    uint32_t n = gh_shell_pen_rows(p);
    if (n && n <= 64) {
        sh->table = gh_shell_pen_table(p);
        sh->n = sh->table ? n : 0;
    }
    return 1;
}
/* Penetration at distance d, the way the game interpolates the table; -1 without one. */
static float aim_pen_at(const aim_shell_t *sh, float d)
{
    const float *t = sh->table;
    uint32_t n = sh->n, i = 1;
    if (!t || !n)
        return -1.0f;
    if (n == 1 || !(d > t[0]))
        return t[2];
    if (d >= t[(n - 1) * 3])
        return t[(n - 1) * 3 + 2];
    while (i < n - 1 && t[i * 3] < d)
        ++i;
    const float *a = t + (i - 1) * 3, *b = t + i * 3;
    float k = a[1] > 0.0f && a[1] < 1e6f ? a[1] : (b[0] > a[0] ? 1.0f / (b[0] - a[0]) : 0.0f);
    return a[2] + (b[2] - a[2]) * (d - a[0]) * k;
}
/* The unit's entity id field, found once by the host (gh_unit_entity_id_find): 1 found, -1 none. */
static int aim_find_eid(void *host, u8 *a, u8 *b)
{
    return gh_unit_entity_id_find(host, a, b);
}
/* One native test along dir through world point p; -1 on a fault. */
static int aim_test(void *host, uint32_t eid, const aim_shell_t *sh, const float *p, const float *dir, float dist,
                    float radius)
{
    gh_shell_test_t in;
    memset(&in, 0, sizeof in);
    in.speed = sh->speed;
    in.dist = dist;
    for (int a = 0; a < 3; ++a) {
        in.pos[a] = p[a] - dir[a] * radius;
        in.dir[a] = dir[a];
    }
    in.len = 4.0f * radius;
    in.full = aim_test_mode == 1;
    in.mode = (uint8_t)aim_test_mode;
    in.flag = aim_test_mode == 1;
    int32_t id = sh->id;
    int r = -1;
    double t = aim_now_us();
    if (gh_shell_test(host, eid, id, &in, &r)) {
        r = -1;
        if (gh_atomic_inc(&aim_pen_faults) >= 3)
            aim_pen_state = 2;
    }
    t = aim_now_us() - t;
    aim_budget_us -= t;
    aim_stat.us += t;
    if (t > aim_stat.us_max)
        aim_stat.us_max = t;
    ++aim_stat.tests;
    if (r >= 0 && r < CT_N)
        ++aim_stat.res_hist[r];
    return r >= 0 && r < CT_N ? r : -1;
}
/* Entry distance into a model-space box, or FLT_MAX's practical equivalent. */
static float aim_box_entry(const float *centre, const float *half, const float *o, const float *d)
{
    float entry = 0.0f, exit_dist = 1e9f;
    for (int a = 0; a < 3; ++a) {
        const float lo = centre[a] - half[a] - o[a], hi = centre[a] + half[a] - o[a];
        if (fabsf(d[a]) < 1e-6f) {
            if (lo > 0.0f || hi < 0.0f)
                return 1e9f;
        } else {
            const float x = lo / d[a], y = hi / d[a];
            entry = fmaxf(entry, fminf(x, y));
            exit_dist = fminf(exit_dist, fmaxf(x, y));
            if (exit_dist < entry)
                return 1e9f;
        }
    }
    return entry;
}
/* First valuable part box the line meets (model space). */
static int aim_line_part(const aim_box_t *box, int nbox, const float *o, const float *d, int *node, float *entry)
{
    int cls = -1;
    *entry = 1e9f;
    for (int i = 0; i < nbox; ++i) {
        const float n = aim_box_entry(box[i].c, box[i].h, o, d);
        if (n < *entry) {
            *entry = n;
            cls = box[i].cls;
            *node = box[i].node;
        }
    }
    return cls;
}
/* ---- Penetration picks, review_5003 (2026-11-10) ---------------------------
 * 5003 (h5.py): a pick took 19.6 native tests x ~340 us = 6.7 ms in one tick
 * (single tests up to 10-15 ms), the points sat a median 2.76 m from the unit
 * box centre (the box includes the barrel) and 73 % of them met no known part.
 * Now:
 *  - a pick is a job of at most PEN_MAX_CANDS candidates inside the middle
 *    60 % of the hull armour extent (aim_target_t.ext_*): up to PEN_MAX_PARTS
 *    valuable part centres (aspect weight x value, nearest box first) and a grid
 *    on the armour faces the shooter sees;
 *  - at most PEN_TESTS_PER_CALL native tests per aim call; the job goes on in
 *    the next calls (the gunner keeps its previous point meanwhile, or gets an
 *    interim one); then the best PEN_REFINE points are tested once more
 *    AIM_JITTER_M to the side and keep the mean;
 *  - jobs are cached PEN_CACHE_MS by (target, PEN_ASPECT_DEG bearing bucket in
 *    the target frame, PEN_RANGE_M range bucket, shell id) and shared by every
 *    shooter with that key; a seat starts at most one job per AIM_EVAL_GAP_MS;
 *  - each shooter scores the tested points itself: value x the probability
 *    that a round aimed there lands on the armour silhouette, for a Gaussian
 *    miss with a range-dependent angular spread. The current retained engine
 *    scatter reduces the available margins and a scene ray checks visibility
 *    from this muzzle;
 *  - "0 table rows" is no read failure: the game computes the penetration
 *    from speed and shell data when the shell has no table (APFSDS), so pen_mm is
 *    logged as "formula"; the effective-armour bisection (2 tests a pick, all
 *    -1 for those shells) is gone.
 * A5 limits new penetration picks during unstable motion. A fully stabilised
 * tank can keep evaluating while driving within the wider motion band. */
#define PEN_MAX_CANDS 20
#define PEN_MAX_PARTS 13
#define PEN_TESTS_PER_CALL 6
#define PEN_REFINE 2
#define PEN_CACHE_N 32
#define PEN_CACHE_MS 4000
#define PEN_ASPECT_DEG 15.0f
#define PEN_RANGE_M 150.0f
#define PEN_MAX_BOX 48
#define PEN_PENDING_MS 120
#define PEN_SIGMA0_DEG 0.3f
#define A5_SPEED 1.0f
#define A5_HULL_RATE 3.0f
#define A5_STAB_SPEED 8.0f
#define A5_STAB_RATE 8.0f
typedef struct {
    u8 *target;
    uint64_t life;
    int32_t shell;
    int abucket, rbucket, state;    /* state: 0 free, 1 testing, 2 refining, 3 done */
    uint64_t made_ms;
    int n, next, nrefine, refine_next, tested, counts[CT_N], nbox;
    int refine_idx[PEN_REFINE];
    float pen, speed, radius, hull_c[3], hull_h[3];
    aim_cand_t cand[PEN_MAX_CANDS];
    aim_box_t box[PEN_MAX_BOX];
} pen_job_t;
static pen_job_t pen_jobs[PEN_CACHE_N];
static int aim_a5 = 1;
static int gate_armed;          /* valid native aim and applied angles have been observed */
/* World point of model point l and the unit direction to it from the muzzle. */
static int pen_line(const float *pos, const float *tm, const float *muzzle, const float *l, float *w, float *dir,
                    float *len)
{
    float d2 = 0.0f;
    for (int a = 0; a < 3; ++a) {
        w[a] = pos[a] + tm[a] * l[0] + tm[3 + a] * l[1] + tm[6 + a] * l[2];
        dir[a] = w[a] - muzzle[a];
        d2 += dir[a] * dir[a];
    }
    *len = sqrtf(d2);
    if (!(*len > 1.0f))
        return 0;
    for (int a = 0; a < 3; ++a)
        dir[a] /= *len;
    return 1;
}
/* One native test of model point l for candidate cd; *score from what the
 * shell meets first along the line. Returns the result, -1 on a fault. */
static int pen_test_one(void *host, uint32_t eid, const aim_shell_t *sh, pen_job_t *j, aim_cand_t *cd, const float *l,
                        const float *pos, const float *tm, const float *muzzle, float dist, float *score)
{
    float w[3], dir[3], len;
    *score = 0.0f;
    if (!pen_line(pos, tm, muzzle, l, w, dir, &len))
        return CT_NONE;
    double t = aim_now_us();
    int r = aim_test(host, eid, sh, w, dir, dist, j->radius);
    aim_stat.call_us += aim_now_us() - t;
    if (r < 0)
        return -1;
    ++j->tested;
    ++j->counts[r];
    float dl[3], ol[3];
    for (int k = 0; k < 3; ++k) {
        dl[k] = dir[0] * tm[k * 3] + dir[1] * tm[k * 3 + 1] + dir[2] * tm[k * 3 + 2];
        ol[k] = l[k] - dl[k] * j->radius;
    }
    int node = -1;
    float entry;
    const int cls = aim_line_part(j->box, j->nbox, ol, dl, &node, &entry);
    if (cd->node < 0)
        cd->cls = cls;
    if (cd->node >= 0 && cd->node != node)
        return r;       /* another part lies between this ray and the intended part */
    if (r == CT_CRIT) {
        *score = 0.4f + 0.6f * (cls >= 0 ? aim_value[cls] : 0.25f);
    } else if (r == CT_TOUCH && node >= 0 && node == cd->node && aim_external(cls)
               && entry + 0.10f < aim_box_entry(j->hull_c, j->hull_h, ol, dl)) {
        /* The native test met the target. The intended external part's box
         * lies ahead of the hull extent on this ray. */
        *score = 0.6f * aim_value[cls];
    }
    return r;
}
/* The job of this key, or a fresh one (*fresh = 1, state 1, to be built). */
static pen_job_t *pen_job_get(u8 *target, uint64_t life, int32_t shell, int ab, int rb, uint64_t ms, int *fresh)
{
    pen_job_t *free_j = NULL, *oldest = &pen_jobs[0];
    *fresh = 0;
    for (int i = 0; i < PEN_CACHE_N; ++i) {
        pen_job_t *j = &pen_jobs[i];
        if (j->state && ms - j->made_ms > PEN_CACHE_MS)
            j->state = 0;
        if (j->state && j->target == target && j->life == life && j->shell == shell && j->abucket == ab && j->rbucket == rb)
            return j;
        if (!j->state && !free_j)
            free_j = j;
        if (j->made_ms < oldest->made_ms)
            oldest = j;
    }
    pen_job_t *j = free_j ? free_j : oldest;
    memset(j, 0, sizeof *j);
    j->target = target;
    j->life = life;
    j->shell = shell;
    j->abucket = ab;
    j->rbucket = rb;
    j->made_ms = ms;
    j->state = 1;
    *fresh = 1;
    return j;
}
/* Candidates of a fresh job (model space; ml = the muzzle in the target frame). */
static void pen_build(void *host, u8 *coll, const aim_target_t *t, pen_job_t *j, const float *ml, int aspect)
{
    int per[AIM_NCLS] = {0};
    for (int i = 0; i < t->n && j->nbox < PEN_MAX_BOX; ++i) {
        int cls = t->part[i].cls;
        if (per[cls] >= (cls == AIM_AMMO ? 12 : 4))
            continue;
        aim_box_t *b = &j->box[j->nbox];
        if (aim_node_box(host, coll, t->hdr, t->part[i].node, b->c, b->h)) {
            b->node = t->part[i].node;
            b->cls = cls;
            ++per[cls];
            ++j->nbox;
        }
    }
    float lo[3], hi[3], mlo[3], mhi[3], mid[3];
    for (int a = 0; a < 3; ++a) {
        float span = t->ext_hi[a] - t->ext_lo[a];
        lo[a] = t->ext_lo[a];
        hi[a] = t->ext_hi[a];
        mlo[a] = lo[a] + 0.2f * span;   /* the middle 60 % */
        mhi[a] = hi[a] - 0.2f * span;
        mid[a] = 0.5f * (lo[a] + hi[a]);
        j->hull_c[a] = mid[a];
        j->hull_h[a] = 0.5f * span;
    }
    /* Valuable parts: classes by aspect weight x value, nearest box inside the middle. */
    int done[AIM_NCLS] = {0};
    for (int np = 0; np < PEN_MAX_PARTS && j->n < PEN_MAX_CANDS;) {
        int cls = -1;
        float bw = 0.0f;
        for (int k = 0; k < AIM_NCLS; ++k)
            if (!done[k] && t->per_cls[k] && (0.1f + aim_weight[aspect][k]) * aim_value[k] > bw) {
                bw = (0.1f + aim_weight[aspect][k]) * aim_value[k];
                cls = k;
            }
        if (cls < 0)
            break;
        done[cls] = 1;
        const int copies = cls == AIM_AMMO || aim_external(cls) ? 2 : 1;
        for (int k = 0; k < copies && np < PEN_MAX_PARTS; ++k) {
            int best = -1;
            float bd = 1e9f;
            for (int i = 0; i < j->nbox; ++i) {
                const aim_box_t *b = &j->box[i];
                int used = 0, inside = 1;
                for (int q = 0; q < j->n; ++q)
                    used |= j->cand[q].node == b->node;
                for (int a = 0; a < 3; ++a)
                    inside &= b->c[a] >= mlo[a] && b->c[a] <= mhi[a];
                float dd = 0.0f;
                for (int a = 0; a < 3; ++a)
                    dd += (b->c[a] - ml[a]) * (b->c[a] - ml[a]);
                if (b->cls == cls && !used && (inside || aim_part_group(cls) != SHOT_KILL) && dd < bd) {
                    bd = dd;
                    best = i;
                }
            }
            if (best < 0)
                break;
            const aim_box_t *b = &j->box[best];
            j->cand[j->n++] = (aim_cand_t){{b->c[0], b->c[1], b->c[2]}, b->node, cls, -1, 0.0f};
            ++np;
        }
    }
    /* Armour faces the muzzle sees: a grid inside the middle 60 %. */
    int front = ml[0] > hi[0] || ml[0] < lo[0], side = ml[2] > hi[2] || ml[2] < lo[2], both = front && side;
    float ys[2] = {mlo[1] + 0.25f * (mhi[1] - mlo[1]), mlo[1] + 0.75f * (mhi[1] - mlo[1])};
    if (front) {
        float x = ml[0] > hi[0] ? hi[0] : lo[0];
        float zs[3] = {mlo[2], both ? mhi[2] : mid[2], mhi[2]};
        for (int iy = 0; iy < 2; ++iy)
            for (int iz = 0; iz < (both ? 2 : 3) && j->n < PEN_MAX_CANDS; ++iz)
                j->cand[j->n++] = (aim_cand_t){{x, ys[iy], zs[iz]}, -1, -1, -1, 0.0f};
    }
    if (side) {
        float z = ml[2] > hi[2] ? hi[2] : lo[2];
        float xs[3] = {mlo[0], both ? mhi[0] : mid[0], mhi[0]};
        for (int iy = 0; iy < 2; ++iy)
            for (int ix = 0; ix < (both ? 2 : 3) && j->n < PEN_MAX_CANDS; ++ix)
                j->cand[j->n++] = (aim_cand_t){{xs[ix], ys[iy], z}, -1, -1, -1, 0.0f};
    }
    if (ml[1] > hi[1] + 1.5f)           /* from above: the roof */
        for (int ix = 0; ix < 2 && j->n < PEN_MAX_CANDS; ++ix)
            j->cand[j->n++] = (aim_cand_t){{ix ? mhi[0] : mlo[0], hi[1], mid[2]}, -1, -1, -1, 0.0f};
}
/* Up to PEN_TESTS_PER_CALL tests of job j; -1 on a fault. */
static int pen_step(void *host, uint32_t eid, const aim_shell_t *sh, pen_job_t *j, const float *pos, const float *tm,
                    const float *muzzle, const float *ml, float dist)
{
    int left = PEN_TESTS_PER_CALL;
    while (left > 0 && j->state == 1 && j->next < j->n) {
        if (aim_budget_us <= 0.0) {
            ++aim_stat.budget_skips;
            return 0;
        }
        aim_cand_t *cd = &j->cand[j->next++];
        float sc;
        int r = pen_test_one(host, eid, sh, j, cd, cd->l, pos, tm, muzzle, dist, &sc);
        if (r < 0)
            return -1;
        cd->res = r;
        cd->score = sc;
        --left;
    }
    if (j->state == 1 && j->next >= j->n) {
        j->nrefine = 0;
        for (int k = 0; k < PEN_REFINE; ++k) {
            int bi = -1;
            for (int i = 0; i < j->n; ++i) {
                int taken = 0;
                for (int q = 0; q < j->nrefine; ++q)
                    taken |= j->refine_idx[q] == i;
                if (!taken && j->cand[i].score > 0.0f && (bi < 0 || j->cand[i].score > j->cand[bi].score))
                    bi = i;
            }
            if (bi >= 0)
                j->refine_idx[j->nrefine++] = bi;
        }
        j->refine_next = 0;
        j->state = j->nrefine ? 2 : 3;
    }
    while (left > 0 && j->state == 2) {
        if (aim_budget_us <= 0.0) {
            ++aim_stat.budget_skips;
            return 0;
        }
        aim_cand_t *cd = &j->cand[j->refine_idx[j->refine_next]];
        /* the engine's scatter moves the hit a few decimetres: sideways across the line of fire */
        float l[3] = {cd->l[0], cd->l[1], cd->l[2]};
        float sx = -(ml[2] - l[2]), sz = ml[0] - l[0], sl = sqrtf(sx * sx + sz * sz);
        if (sl > 0.1f) {
            l[0] += AIM_JITTER_M * sx / sl;
            l[2] += AIM_JITTER_M * sz / sl;
        }
        aim_cand_t tmp = *cd;
        float sc;
        if (pen_test_one(host, eid, sh, j, &tmp, l, pos, tm, muzzle, dist, &sc) < 0)
            return -1;
        cd->score = 0.5f * (cd->score + sc);
        --left;
        if (++j->refine_next >= j->nrefine)
            j->state = 3;
    }
    return 0;
}
static float aim_lateral(const float *v, float bearing)
{
    const float a = bearing / 57.29578f;
    return -sinf(a) * v[0] + cosf(a) * v[2];
}
/* Conservative central margins of a model-space box in the shot plane. */
static void aim_project(const float *half, const float *offset, float bearing, float *w, float *h)
{
    const float a = bearing / 57.29578f;
    *w = fmaxf(0.0f, fabsf(sinf(a)) * half[0] + fabsf(cosf(a)) * half[2] - fabsf(aim_lateral(offset, bearing)));
    *h = fmaxf(0.0f, half[1] - fabsf(offset[1]));
}
/* The evaluator adds this current world offset after the hook. Retaining
 * scatter consumes the corresponding part of a box's usable margins. */
static int aim_local_scatter(u8 *shooter, const float *tm, float *local)
{
    const float *sc = gh_unit_aim_scatter(shooter);
    if (!(fabsf(sc[0]) < 20.0f && fabsf(sc[1]) < 20.0f && fabsf(sc[2]) < 20.0f))
        return 0;
    for (int a = 0; a < 3; ++a)
        local[a] = aim_scatter * (tm[a * 3] * sc[0] + tm[a * 3 + 1] * sc[1] + tm[a * 3 + 2] * sc[2]);
    return 1;
}
/* s50: the guarded block of aim_scene_line, run under the host's fault guard (gh_guard, ground_host.h). */
typedef struct { void *host; const float *muzzle, *dir; float len, tt; int kind, clear; } aim_scene_line_ctx_t;
static void aim_scene_line_run(void *ctx)
{
    aim_scene_line_ctx_t *const context = ctx;
    void *const host = context->host;
    const float *const muzzle = context->muzzle;
    const float *const dir = context->dir;
    const float len = context->len;
    float tt = context->tt;
    int kind = context->kind;
    int clear = context->clear;
    {
        const u8 hit = gh_ray_cast(host, muzzle, dir, &tt, GH_RAY_GROUND_AND_OBJECTS, &kind);
        clear = !hit || tt >= len - 1.0f;
    }
    context->clear = clear;
}
/* A fresh line to the candidate, 1 clear, 0 obstructed, -1 unavailable. */
static int aim_scene_line(void *host, const float *muzzle, const float *dir, float len)
{
    float tt = len - 0.5f;
    int kind = 0;
    int clear = 1;
    aim_scene_line_ctx_t context = {host, muzzle, dir, len, tt, kind, clear};
    const int faulted = gh_guard(aim_scene_line_run, &context);
    clear = context.clear;
    if (faulted) {
        clear = -1;
    }
    return clear;
}
/* Read the live part box before using cached penetration results. A changed
 * box needs a fresh job; the old ray cannot establish an opportunity there. */
static int pen_part_current(void *host, u8 *coll, u8 *hdr, const aim_cand_t *p, float *half)
{
    float centre[3];
    if (!aim_node_box(host, coll, hdr, p->node, centre, half))
        return 0;
    for (int a = 0; a < 3; ++a)
        if (fabsf(p->l[a] - centre[a]) > fmaxf(0.035f, 0.35f * half[a]))
            return 0;
    return 1;
}
/* A finished job must offer a useful kill, a reliable firepower shot, or a
 * reliable mobility shot. Each is rescored in this gunner's current geometry. */
static int pen_select(void *host, u8 *coll, pen_job_t *j, const aim_target_t *t, aim_choice_t *c, const float *pos,
                      const float *tm, const float *muzzle, const float *scatter, float deg, float dist, float sigma_deg)
{
    const float sm = fmaxf(0.05f, dist * tanf(sigma_deg / 57.29578f));
    const float scatter_w = fabsf(aim_lateral(scatter, deg)), scatter_h = fabsf(scatter[1]);
    const float k = 1.0f / (1.41421356f * sm);
    int pick[SHOT_NONE] = {-1, -1, -1, -1};
    float score[SHOT_NONE] = {0}, chance[SHOT_NONE] = {0}, width[SHOT_NONE] = {0}, height[SHOT_NONE] = {0};
    int changed = 0, unavailable = 0;
    for (int i = 0; i < j->n; ++i) {
        const aim_cand_t *cd = &j->cand[i];
        if (!(cd->score > 0.0f))
            continue;
        const int group = aim_part_group(cd->cls);
        if (group != SHOT_KILL && cd->node < 0)
            continue;
        float hw, hh;
        if (cd->node >= 0) {
            float half[3], off[3] = {0};
            if (!pen_part_current(host, coll, t->hdr, cd, half)) {
                changed = 1;
                continue;
            }
            aim_project(half, off, deg, &hw, &hh);
        } else {
            float off[3];
            for (int a = 0; a < 3; ++a)
                off[a] = cd->l[a] - j->hull_c[a];
            aim_project(j->hull_h, off, deg, &hw, &hh);
        }
        if (!(hw - scatter_w >= 0.04f && hh - scatter_h >= 0.04f))
            continue;
        const float phit = erff((hw - scatter_w) * k) * erff((hh - scatter_h) * k);
        const float utility = cd->score * phit;
        const float min_chance = group == SHOT_KILL ? 0.20f : 0.17f;
        const float min_utility = group == SHOT_KILL ? 0.23f : 0.065f;
        if (!(phit >= min_chance && utility >= min_utility && utility > score[group]))
            continue;
        float w[3], dir[3], len;
        if (!pen_line(pos, tm, muzzle, cd->l, w, dir, &len)) {
            unavailable = 1;
            continue;
        }
        const int clear = aim_scene_line(host, muzzle, dir, len);
        if (clear <= 0) {
            unavailable |= clear < 0;
            continue;
        }
        score[group] = utility;
        chance[group] = phit;
        width[group] = hw;
        height[group] = hh;
        pick[group] = i;
    }
    const int group = pick[SHOT_KILL] >= 0 ? SHOT_KILL : pick[SHOT_FIRE] >= 0 ? SHOT_FIRE : SHOT_MOBILITY;
    if (pick[group] < 0) {
        if (changed)
            j->state = 0;
        if (!changed && !unavailable && j->n && j->tested >= j->n)
            c->opportunity = SHOT_NONE;
        return 0;
    }
    const aim_cand_t *p = &j->cand[pick[group]];
    memcpy(c->centre, p->l, sizeof c->centre);
    c->node = p->node;
    c->cls = p->cls;
    c->res = p->res;
    c->score = score[group];
    c->phit = chance[group];
    c->gate_w = width[group];
    c->gate_h = height[group];
    c->opportunity = group;
    c->sigma = sigma_deg;
    c->cands = j->n;
    c->tested = j->tested;
    memcpy(c->counts, j->counts, sizeof c->counts);
    c->pen = j->pen;
    c->speed = j->speed;
    return 1;
}
/* Penetration pick for this shooter: 1 = c filled (weak 2), 0 = fall back now,
 * -1 = a job is still running (try again in PEN_PENDING_MS). */
static int pen_pick(void *host, u8 *shooter, u8 *target, int seat, u8 *coll, aim_target_t *t, aim_choice_t *c,
                    float dist, float deg, int aspect, uint64_t ms)
{
    aim_shell_t sh;
    if (!aim_shell(host, shooter, &sh)) {
        ++aim_stat.no_shell;
        return 0;
    }
    static int shell_logged;
    if (shell_logged < 12) {    /* a sample to check the belt reading against the loadout lines */
        static u8 *last_shooter[MAX_SEATS];
        if (last_shooter[seat] != shooter) {
            last_shooter[seat] = shooter;
            ++shell_logged;
            if (sh.n)
                event(host, "aim: seat %d next shell id %d speed %.0f m/s, penetration %.0f mm at 0 m, %.0f at 1000 m "
                      "(%u table rows)", seat, sh.id, sh.speed, aim_pen_at(&sh, 0.0f), aim_pen_at(&sh, 1000.0f), sh.n);
            else
                event(host, "aim: seat %d next shell id %d speed %.0f m/s, penetration by formula (no table: "
                      "the game's own formula)", seat, sh.id, sh.speed);
        }
    }
    if (!aim_eid_off && aim_eid_tries++ < 6) {
        aim_eid_off = aim_find_eid(host, shooter, target);
        if (aim_eid_off > 0)
            event(host, "aim: unit eid found (the game maps it back for shooter and target)");
        else if (aim_eid_tries >= 6)
            event(host, "aim: no eid field found in the unit, penetration test off");
        else
            aim_eid_off = 0;
    }
    if (aim_eid_off <= 0) {
        ++aim_stat.no_eid;
        return 0;
    }
    uint32_t eid = gh_unit_entity_id(target);
    if (gh_entity_unit(host, eid) != target) {
        ++aim_stat.no_eid;
        return 0;
    }
    const float *tm = gh_unit_frame(target), *pos = gh_unit_pos(target);
    const float *stm = gh_unit_frame(shooter), *sp = gh_unit_pos(shooter);
    const float *sbb = gh_unit_bbox(shooter);
    float muzzle[3], ml[3], radius = 0.0f;
    float up = sbb[4] > 0.5f && sbb[4] < 6.0f ? 0.85f * sbb[4] : 2.0f;
    for (int a = 0; a < 3; ++a) {
        float size = t->ext_hi[a] - t->ext_lo[a];
        muzzle[a] = sp[a] + stm[3 + a] * up;
        radius += 0.25f * size * size;
    }
    radius = sqrtf(radius) + 0.5f;
    for (int r = 0; r < 3; ++r)
        ml[r] = (muzzle[0] - pos[0]) * tm[r * 3] + (muzzle[1] - pos[1]) * tm[r * 3 + 1] + (muzzle[2] - pos[2]) * tm[r * 3 + 2];

    int ab = (int)floorf((deg + 180.0f) / PEN_ASPECT_DEG), rb = (int)(dist / PEN_RANGE_M), fresh = 0;
    pen_job_t *j = pen_job_get(target, t->life, sh.id, ab, rb, ms, &fresh);
    if (fresh) {
        if (ms < aim_eval_ms[seat] + AIM_EVAL_GAP_MS) {
            j->state = 0;       /* this seat started one a moment ago */
            return -1;
        }
        aim_eval_ms[seat] = ms;
        j->pen = aim_pen_at(&sh, dist);
        j->speed = sh.speed;
        j->radius = radius;
        pen_build(host, coll, t, j, ml, aspect);
        ++aim_stat.jobs;
        if (!j->n)
            j->state = 3;
    } else if (j->state == 3) {
        ++aim_stat.cache_hits;
    }
    if (j->state < 3) {
        int before = j->tested;
        aim_stat.call_us = 0.0;
        if (pen_step(host, eid, &sh, j, pos, tm, muzzle, ml, dist) < 0) {
            j->state = 0;
            return 0;
        }
        aim_stat.job_tests += j->tested - before;
        if (aim_stat.call_us > aim_stat.call_us_max)
            aim_stat.call_us_max = aim_stat.call_us;
        if (j->state < 3)
            return aim_pen_state == 1 ? -1 : 0;
    }
    /* This shooter's gun error now (5003: < 1 deg hit 63 %, 5-15 deg 18 %). */
    float err = shooter_gun_err(shooter, pos);
    const float sigma = cfg.gate && gate_armed ? 0.035f
                       : PEN_SIGMA0_DEG + (err < 0.0f ? 1.5f : fminf(err, 8.0f));
    float scatter[3];
    if (!aim_local_scatter(shooter, tm, scatter))
        return 0;
    c->cache = !fresh;
    return pen_select(host, coll, j, t, c, pos, tm, muzzle, scatter, deg, dist, sigma);
}
/* The gate reads the geometric impact point for range and occlusion. Its
 * angular reference comes from the native turret command after ballistics. */
static void aim_note(u8 *shooter, u8 *target, int seat, int ts, const float *w, uint64_t ms,
                     float half_w, float half_h, int opportunity, int native, int node)
{
    aim_rec_t *r = &aim_rec[seat][ts];
    r->shooter = shooter;
    r->target = target;
    r->shooter_life = slot_life[seat];
    r->target_life = slot_life[ts];
    r->ms = ms;
    memcpy(r->point, w, sizeof r->point);
    memcpy(r->origin, gh_unit_pos(target), sizeof r->origin);
    r->half_w = half_w;
    r->half_h = half_h;
    r->opportunity = opportunity;
    r->native = native;
    r->node = node;
}
static int aim_point_sane(const float *p, const float *origin)
{
    float d2 = 0.0f;
    for (int a = 0; a < 3; ++a) {
        float d = p[a] - origin[a];
        d2 += d * d;
    }
    return d2 < 15.0f * 15.0f;
}
static int aim_frame_sane(const float *tm)
{
    for (int a = 0; a < 3; ++a) {
        const float *v = tm + a * 3;
        const float *w = tm + ((a + 1) % 3) * 3;
        const float len2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
        const float dot = v[0] * w[0] + v[1] * w[1] + v[2] * w[2];
        if (!(fabsf(len2 - 1.0f) < 0.08f && fabsf(dot) < 0.08f))
            return 0;
    }
    return 1;
}
static void aim_skip(void *host, int seat, int target, uint64_t ms, const char *why, const float *bb)
{
    if (ms < aim_skip_ms[seat])
        return;
    aim_skip_ms[seat] = ms + 8000;
    event(host, "aim: seat %d target %d native point (%s), unit box %.3g %.3g %.3g .. %.3g %.3g %.3g",
          seat, target, why, bb[0], bb[1], bb[2], bb[3], bb[4], bb[5]);
}
/* s50: the guarded block of wt_ground_aim, run under the host's fault guard (gh_guard, ground_host.h). */
typedef struct { u8 *shooter, *target; float *point; void *host; int seat, stage; } aim_entry_ctx_t;
static void aim_entry_run(void *ctx)
{
    aim_entry_ctx_t *const context = ctx;
    u8 *const shooter = context->shooter;
    u8 *const target = context->target;
    float *const point = context->point;
    void *const host = context->host;
    int seat = context->seat;
    int stage = context->stage;
    {
        if (!aim_state) {
            aim_state = aim_env("WT_GROUND_WEAKSPOT", 1.0f) > 0.0f ? 1 : 2;
            aim_skill = aim_env("WT_GROUND_WEAKSPOT_SKILL", 0.7f);
            aim_scatter = aim_env("WT_GROUND_WEAKSPOT_SCATTER", 0.5f);
            aim_scatter = aim_scatter < 0.0f ? 0.0f : aim_scatter > 1.0f ? 1.0f : aim_scatter;
            aim_rng ^= (uint32_t)gh_ms() | 1u;
            event(host, "aim: weak-spot aiming %s, skill %.2f, engine scatter kept %.2f (engine scale vs bots %.2f, "
                  "vs humans %.2f)", aim_state == 1 ? "on" : "off", aim_skill, aim_scatter,
                  gh_bot_scatter_vs_bot(host), gh_bot_scatter_vs_human(host));
            /* WT_GROUND_PENAIM_MODE (with the pick on): 1 = crit pass
             * like the crosshair indicator, 0 = the plain test the game's script layer uses. */
            /* s46: the penetration-test pick is off by default (WT_GROUND_PENAIM=1 turns it on); the old part pick
             * (weak 1, only while the penetration test is off) is off by default too (WT_GROUND_PARTAIM=1), so the
             * default aim uses the mass / centre picks only */
            aim_pen_state = aim_env("WT_GROUND_PENAIM", 0.0f) > 0.0f ? 1 : 2;
            aim_part_on = aim_env("WT_GROUND_PARTAIM", 0.0f) > 0.0f;
            event(host, "aim: penetration-test pick %s (WT_GROUND_PENAIM), part pick %s (WT_GROUND_PARTAIM)",
                  aim_pen_state == 1 ? "on" : "off", aim_part_on ? "on" : "off");
            aim_test_mode = aim_env("WT_GROUND_PENAIM_MODE", 1.0f) > 0.0f ? 1 : 0;
            aim_budget_rate = aim_env("WT_GROUND_PENAIM_BUDGET", (float)AIM_BUDGET_US_PER_MS);
            aim_a5 = aim_env("WT_GROUND_A5", 1.0f) > 0.0f;   /* review_5003 A5 */
            u8 *dms = gh_dm_settings(host);
            if (dms)
                event(host, "aim: penetration test %s (mode %d, budget %.0f us/s); dm settings "
                      "maxArmorEffectiveScale %.2f armorBreachK %.2f groundArmorSlopeAngleMult %.2f "
                      "ricochet test threshold %.2f..%.2f",
                      aim_pen_state == 1 ? "on" : "off", aim_test_mode, aim_budget_rate * 1000.0, gh_dm_max_armor_scale(dms),
                      gh_dm_armor_breach_k(dms),
                      gh_dm_slope_angle_mult(dms), gh_dm_ricochet_min(dms), gh_dm_ricochet_max(dms));
        }
        seat = context->seat = aim_state == 1 ? bot_seat(shooter) : -1;
        if (seat < 0 || !is_tank(shooter) || !is_tank(target))
            goto leave;
        gh_atomic_inc(&aim_calls[seat]);     /* A7 probe */
        uint64_t ms = gh_ms();
        const float *tm = gh_unit_frame(target), *pos = gh_unit_pos(target);
        const float *sp = gh_unit_pos(shooter), *bb = gh_unit_bbox(target);
        const u8 *player = gh_unit_player(target);
        const int ts = player ? gh_player_slot(player) & 63 : -1;
        if (ts < 0 || !slot_alive[ts] || slot_unit[ts] != target || !slot_alive[seat] || slot_unit[seat] != shooter) {
            aim_skip(host, seat, ts, ms, "unit identity unavailable", bb);
            goto leave;
        }
        const int native_valid = aim_point_sane(point, pos);
        if (native_valid)
            aim_note(shooter, target, seat, ts, point, ms, 0.65f, 0.40f, SHOT_UNKNOWN, 1, -1);
        else
            aim_rec[seat][ts].ms = 0;
        if (ms < aim_retry_ms[seat])
            goto leave;
        if (!aim_frame_sane(tm)) {
            aim_skip(host, seat, ts, ms, "target transform unavailable", bb);
            goto leave;
        }
        stage = context->stage = 1;
        /* Where the shooter is in the target's frame: 0 deg = dead ahead of its hull. */
        float d[3] = {sp[0] - pos[0], sp[1] - pos[1], sp[2] - pos[2]};
        float ahead = d[0] * tm[0] + d[1] * tm[1] + d[2] * tm[2];
        float right = d[0] * tm[6] + d[1] * tm[7] + d[2] * tm[8];
        float deg = atan2f(right, ahead) * 57.29578f, adeg = fabsf(deg);
        int aspect = adeg <= AIM_FRONT_DEG ? ASPECT_FRONT : adeg >= AIM_REAR_DEG ? ASPECT_REAR : ASPECT_SIDE;

        aim_choice_t *c = NULL, *oldest = &aim_choices[seat][0];
        for (int i = 0; i < AIM_WAYS; ++i) {
            aim_choice_t *e = &aim_choices[seat][i];
            if (e->target == target && e->target_life == slot_life[ts] && e->shooter_life == slot_life[seat])
                c = e;
            if (e->until_ms < oldest->until_ms)
                oldest = e;
        }
        u8 *hdr = NULL, *coll = aim_coll(target, &hdr);
        aim_target_t *t = coll ? aim_parts(host, target, slot_life[ts], ms) : NULL;
        const int dm_extent = t && t->ext_kind >= 0;
        const float *elo = dm_extent ? t->ext_lo : bb;
        const float *ehi = dm_extent ? t->ext_hi : bb + 3;
        if (!aim_extent_valid(elo, ehi)) {
            aim_skip(host, seat, ts, ms, "no validated damage-model or unit bounds", bb);
            goto leave;
        }
        /* A validated unit box is also usable when the parts exist but did
         * not establish their own hull extent. */
        if (t && !dm_extent) {
            memcpy(t->ext_lo, elo, sizeof t->ext_lo);
            memcpy(t->ext_hi, ehi, sizeof t->ext_hi);
            t->ext_kind = 0;
        }
        float esz[3], emid[3], eh[3];
        for (int a = 0; a < 3; ++a) {
            esz[a] = ehi[a] - elo[a];
            eh[a] = 0.5f * esz[a];
            emid[a] = elo[a] + eh[a];
        }
        float dist = hypotf(hypotf(d[0], d[1]), d[2]);
        if (!(dist > 1.0f && dist < 20000.0f) || !isfinite(deg)) {
            aim_skip(host, seat, ts, ms, "target range or bearing unavailable", bb);
            goto leave;
        }
        stage = context->stage = 2;
        if (ms > aim_budget_ms) {   /* refill the test-time budget */
            aim_budget_us += aim_budget_rate * (double)(ms - aim_budget_ms);
            if (aim_budget_us > AIM_BUDGET_CAP_US)
                aim_budget_us = AIM_BUDGET_CAP_US;
            aim_budget_ms = ms;
        }
        const int geometry_changed = !c || c->aspect != aspect || fabsf(wrap180(deg - c->bearing)) > AIM_RESEEK_DEG
                                     || fabsf(dist - c->dist) > fminf(15.0f, 0.10f * c->dist);
        int due = geometry_changed || ms >= c->until_ms;
        const lane_seat_t *ls = &lseat[seat];
        const float my_speed = ls->unit == shooter ? hypotf(ls->vx, ls->vz) : 0.0f;
        const float my_rate = ls->unit == shooter ? fabsf(ls->hull_rate) : 0.0f;
        const int stable = aim_a5 && (my_speed > A5_SPEED || my_rate > A5_HULL_RATE) && seat_movefire(shooter) == 2;
        const char *why = !aim_a5 ? NULL
                          : my_speed > (stable ? A5_STAB_SPEED : A5_SPEED) ? "moving"
                          : my_rate > (stable ? A5_STAB_RATE : A5_HULL_RATE) ? "turning" : NULL;
        if (c && (!!why != !!c->why))
            due = 1;
        if (!due && c->weak && c->node >= 0 && ms >= c->box_ms && coll && t && t->hdr == hdr) {
            float centre[3], half[3];
            c->box_ms = ms + AIM_BOX_MS;
            if (!aim_node_box(host, coll, hdr, c->node, centre, half)) {
                due = 1;
            } else if (c->weak == 2) {
                for (int a = 0; a < 3; ++a)
                    due |= fabsf(centre[a] - c->centre[a]) > fmaxf(0.035f, 0.35f * half[a]);
            } else {
                memcpy(c->centre, centre, sizeof centre);
                memcpy(c->half, half, sizeof half);
            }
            if (due)
                c->assessed_ms = 0;
        }

        int picked = 0;
        if (due) {
            if (!c)
                c = oldest;
            const int try_pen = !why && aim_skill > 0.0f && aim_pen_state == 1 && coll && t;
            aim_choice_t nc = {0};
            nc.node = nc.cls = nc.res = -1;
            nc.pen = -1.0f;
            nc.why = why;
            const int r = try_pen ? pen_pick(host, shooter, target, seat, coll, t, &nc, dist, deg, aspect, ms) : 0;
            if (r < 0)
                ++aim_stat.pending;
            if (r < 0 && !geometry_changed && c->weak == 2 && c->assessed_ms && ms - c->assessed_ms < 1400) {
                c->until_ms = ms + PEN_PENDING_MS;
            } else {
                picked = 1;
                nc.target = target;
                nc.target_life = slot_life[ts];
                nc.shooter_life = slot_life[seat];
                nc.aspect = aspect;
                nc.bearing = deg;
                nc.dist = dist;
                nc.assessed_ms = nc.opportunity != SHOT_UNKNOWN ? ms : 0;
                nc.until_ms = ms + (r < 0 ? PEN_PENDING_MS : AIM_HOLD_MS + (uint64_t)(aim_rand() * 400.0f));
                nc.box_ms = ms + AIM_BOX_MS;
                if (r > 0)
                    nc.weak = 2;
                else if (!why && aim_part_on && aim_pen_state == 2 && !aim_pen_faults && t && t->n && aim_rand() < aim_skill)
                    nc.weak = aim_pick(host, coll, t, aspect, emid[0], &nc);
                ++aim_stat.picks[nc.weak];
                if (nc.weak == 2 && nc.res >= 0 && nc.res < CT_N)
                    ++aim_stat.pick_res[nc.res];
                if (!nc.weak && why) {
                    nc.centred = 1;
                    ++aim_stat.centre;
                    nc.centre[0] = emid[0];
                    nc.centre[1] = elo[1] + 0.45f * esz[1];
                    nc.centre[2] = emid[2];
                    nc.jit[0] = (aim_rand() * 2.0f - 1.0f) * 0.05f * esz[0];
                    nc.jit[1] = (aim_rand() * 2.0f - 1.0f) * 0.04f * esz[1];
                    nc.jit[2] = (aim_rand() * 2.0f - 1.0f) * 0.06f * esz[2];
                } else if (!nc.weak) {
                    static const float fx[3] = {0.30f, -0.05f, -0.30f}, fy[3] = {0.35f, 0.42f, 0.50f};
                    nc.centre[0] = emid[0] + fx[aspect] * esz[0];
                    nc.centre[1] = elo[1] + fy[aspect] * esz[1];
                    nc.centre[2] = emid[2];
                    nc.jit[0] = (aim_rand() * 2.0f - 1.0f) * 0.10f * esz[0];
                    nc.jit[1] = (aim_rand() * 2.0f - 1.0f) * 0.08f * esz[1];
                    nc.jit[2] = (aim_rand() * 2.0f - 1.0f) * 0.15f * esz[2];
                }
                *c = nc;
            }
        }
        float l[3] = {c->centre[0] + c->jit[0], c->centre[1] + c->jit[1], c->centre[2] + c->jit[2]};
        float w[3], adjusted[3];
        const float *sc = gh_unit_aim_scatter(shooter);
        for (int a = 0; a < 3; ++a) {
            w[a] = pos[a] + tm[a] * l[0] + tm[3 + a] * l[1] + tm[6 + a] * l[2];
            /* Native adds this scatter after the hook. The gate uses the
             * unadjusted impact point and native's resulting turret command. */
            adjusted[a] = w[a];
            if (fabsf(sc[a]) < 20.0f)
                adjusted[a] -= (1.0f - aim_scatter) * sc[a];
        }
        if (!aim_point_sane(w, pos) || !aim_point_sane(adjusted, pos)) {
            aim_skip(host, seat, ts, ms, "computed point outside the target", bb);
            c->until_ms = 0;
            goto leave;
        }
        float scatter[3];
        if (!aim_local_scatter(shooter, tm, scatter)) {
            aim_skip(host, seat, ts, ms, "invalid engine aim offset", bb);
            goto leave;
        }
        float hw = c->gate_w, hh = c->gate_h;
        if (c->weak == 2) {
            hw -= fabsf(aim_lateral(scatter, deg));
            hh -= fabsf(scatter[1]);
        } else {
            float off[3];
            for (int a = 0; a < 3; ++a)
                off[a] = scatter[a] + (c->weak ? c->jit[a] : l[a] - emid[a]);
            aim_project(c->weak ? c->half : eh, off, deg, &hw, &hh);
        }
        if (!(hw > 0.0f && hh > 0.0f)) {
            aim_skip(host, seat, ts, ms, "no usable area around the point", bb);
            c->until_ms = 0;
            goto leave;
        }
        const float native_y = point[1];
        /* A completed miss of the opportunity criteria still has a geometric
         * reference for the gate. During pending work the native command can
         * track the fallback point while the evaluation continues. */
        memcpy(point, adjusted, sizeof adjusted);
        aim_note(shooter, target, seat, ts, w, ms, hw, hh, c->opportunity, 0, c->node);

        /* One aim.csv row per new pick (at most one per second per seat). */
        if (picked && ms >= aim_log_ms[seat]) {
            aim_log_ms[seat] = ms + 1000;
            if (!aim_csv)
                aim_csv = open_csv("aim.csv", "t,seat,target_slot,target,dist,aspect_deg,mode,part,node,lx,ly,lz,"
                                              "raise_m,origin_agl,cands,tested,crit,touch,stop,ricochet,none,result,"
                                              "score,pen_mm,eff_lo_mm,eff_hi_mm,shell_speed,box_x0,box_y0,box_z0,"
                                              "box_x1,box_y1,box_z1,cache,phit,sigma_deg,ext_kind,ext_x0,ext_y0,ext_z0,"
                                              "ext_x1,ext_y1,ext_z1,centre_why,own_speed,own_hull_rate,opportunity,half_w,half_h");
            if (aim_csv) {
                u8 *tp = gh_unit_player(target);
                const char *name = gh_unit_name(target) ? (const char *)gh_unit_name(target) : "?";
                static const char *const mode_name[3] = {"mass", "weak", "pen"};
                const char *part = c->weak && c->cls >= 0 && c->cls < AIM_NCLS ? aim_cls_name[c->cls]
                                   : c->weak == 2 ? "?"
                                   : c->centred ? "centre"
                                   : aspect == ASPECT_FRONT ? "front" : aspect == ASPECT_REAR ? "rear" : "side";
                char pen[16] = "";
                if (c->weak == 2 && c->pen >= 0.0f)
                    snprintf(pen, sizeof pen, "%.0f", c->pen);
                else if (c->weak == 2)
                    snprintf(pen, sizeof pen, "formula");
                fprintf(aim_csv, "%.1f,%d,%d,%s,%.0f,%.0f,%s,%s,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%s,%.2f,"
                                 "%s,,,%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
                                 "%s,%.1f,%.1f,%d,%.3f,%.3f\n",
                        (double)(ms - t0) / 1000.0, seat, tp ? gh_player_slot(tp) : -1, name, dist, deg,
                        c->centred ? "centre" : mode_name[c->weak], part, c->node, l[0], l[1], l[2], w[1] - native_y,
                        pos[1] - ground_height(host, pos[0], pos[1] + 5.0f, pos[2]), c->cands, c->tested,
                        c->counts[CT_CRIT], c->counts[CT_TOUCH] + c->counts[CT_CANCEL], c->counts[CT_STOP],
                        c->counts[CT_RICOCHET], c->counts[CT_NONE], c->res >= 0 && c->res < CT_N ? aim_res_name[c->res] : "",
                        c->weak == 2 ? c->score : 0.0f, pen, c->weak == 2 ? c->speed : 0.0f,
                        bb[0], bb[1], bb[2], bb[3], bb[4], bb[5], c->cache, c->phit, c->sigma, t ? t->ext_kind : -1,
                        elo[0], elo[1], elo[2], ehi[0], ehi[1], ehi[2], c->why ? c->why : "", my_speed, my_rate,
                        c->opportunity, hw, hh);
                fflush(aim_csv);
            }
        }
        if (ms >= aim_stat.next_ms) {   /* summary: one line per minute */
            if (aim_stat.next_ms && aim_stat.picks[0] + aim_stat.picks[1] + aim_stat.picks[2])
                event(host, "aim: last 60 s picks pen %d (crit %d touch %d) weak %d mass %d (centre %d); %d jobs, "
                      "%d cache hits, %d pending calls; %d tests (%.1f per job), %.0f us avg, %.0f us max, %.0f us max "
                      "per call; budget waits %d, no shell %d, no eid %d, faults %ld%s",
                      aim_stat.picks[2], aim_stat.pick_res[CT_CRIT], aim_stat.pick_res[CT_TOUCH], aim_stat.picks[1],
                      aim_stat.picks[0], aim_stat.centre, aim_stat.jobs, aim_stat.cache_hits, aim_stat.pending,
                      aim_stat.tests, aim_stat.jobs ? (double)aim_stat.job_tests / aim_stat.jobs : 0.0,
                      aim_stat.tests ? aim_stat.us / aim_stat.tests : 0.0, aim_stat.us_max, aim_stat.call_us_max,
                      aim_stat.budget_skips, aim_stat.no_shell, aim_stat.no_eid, aim_pen_faults,
                      aim_pen_state == 2 ? " (penetration test off)" : "");
            memset(&aim_stat, 0, sizeof aim_stat);
            aim_stat.next_ms = ms + 60000;
        }
    }
leave:;
}
/* The game's gunner evaluator: `point` is the aim point the target just
 * returned (world, 3 floats, in the evaluator's frame); it is used for the
 * ballistic solution of this gunner right after. */
void ground_aim(u8 *shooter, u8 *target, float *point, void *host)
{
    gh_once(GH_ONCE_INIT, init);
    if (aim_state == 2 || !cfg.loaded || !shooter || !target || !point || !host || shooter == target)
        return;
    gh_rw_write_lock(GH_RW_AIM);
    int seat = -1;
    int stage = 0;
    aim_entry_ctx_t context = {shooter, target, point, host, seat, stage};
    const int faulted = gh_guard(aim_entry_run, &context);
    seat = context.seat;
    stage = context.stage;
    if (faulted) {
        const long faults = gh_atomic_inc(&aim_faults);
        const uint64_t now = gh_ms();
        if (seat >= 1 && seat < MAX_SEATS) {
            aim_retry_ms[seat] = now + 5000;
            if (now >= aim_skip_ms[seat]) {
                aim_skip_ms[seat] = now + 8000;
                event(host, "aim: seat %d native fallback after fault in stage %d (fault %ld)", seat, stage, faults);
            }
        } else if (faults <= 3) {
            event(host, "aim: native fallback after initialisation fault %ld", faults);
        }
    }
    gh_rw_write_unlock(GH_RW_AIM);
}
/* Main-gun fire gate. The game adds lead/scatter and solves the
 * ballistics after the aim hook.
 * The turret record has the applied yaw/pitch (gh_turret_yaw / _pitch) and the
 * requested yaw/pitch (gh_turret_cmd_yaw / _pitch, the game's turret setter; turret.csv).
 * Both angles live in the same hull frame. The selected impact area
 * determines tolerances, while the native commands supply the firing line.
 * Holding past the request delay informs movement. Reload and movement
 * eligibility are checked independently before this precision gate. */
#define GATE_HALF_M 1.2f
#define GATE_HOLD_MS 2500
static int gate_state;              /* 0 unread, 1 on, 2 aim gate off */
static float gate_half_m = GATE_HALF_M;
static int gate_hold_ms = GATE_HOLD_MS;
static volatile long gate_faults;
static struct {
    uint64_t next_ms;
    int releases, holds, no_aim, fallback, blocked, reposition, move_holds, move_calls, band_holds, band_released;
    double wait_sum, err_sum;
} gate_stat;
#define GATE_BAND_MAX_MS 4000   /* stab2: a stabilised tank beyond its band waits this long for the stop ... */
#define GATE_BAND_ON_DEG 1.0f  /* ... then may fire on the move with the precision gate still active */
#define GATE_MOVE_MPS 1.0f      /* movefire: no vertical stabiliser -> hold every round above this speed */
static uint64_t gate_bad_cmd_ms[MAX_SEATS];
/* The engine angles are in the local hull frame. Apply the same frame to
 * both directions so a pitched or rolled hull cannot mix the two errors. */
static void gate_dir(u8 *unit, float yaw, float pitch, float out[3])
{
    const float *f = gh_unit_frame(unit), *u = gh_unit_up(unit);
    const float r[3] = {f[1] * u[2] - f[2] * u[1], f[2] * u[0] - f[0] * u[2], f[0] * u[1] - f[1] * u[0]};
    const float a = (float)yaw_sign * yaw / 57.29578f, p = pitch / 57.29578f;
    const float ca = cosf(a), sa = sinf(a), cp = cosf(p), sp = sinf(p);
    for (int i = 0; i < 3; ++i)
        out[i] = cp * (ca * f[i] + sa * r[i]) + sp * u[i];
}
static void gate_resume_reload(gate_seat_t *g, u8 *gun, float time)
{
    if (g->reload_held && gh_gun_reload_end(gun) < time)
        gh_gun_set_reload_end(gun, time);
    g->reload_held = 0;
}
/* s50: the guarded block of wt_ground_fire, run under the host's fault guard (gh_guard, ground_host.h). */
typedef struct { u8 *gun; void *host; float time; int hold; } gate_entry_ctx_t;
static void gate_entry_run(void *ctx)
{
    gate_entry_ctx_t *const context = ctx;
    u8 *const gun = context->gun;
    void *const host = context->host;
    const float time = context->time;
    int hold = context->hold;
    {
        if (!gate_state) {
            gate_state = 1;
            const float half = aim_env("WT_GROUND_GATE_HALF_M", GATE_HALF_M);
            const float delay = aim_env("WT_GROUND_GATE_HOLD_MS", (float)GATE_HOLD_MS);
            gate_half_m = half > 0.0f && half <= 5.0f ? half : GATE_HALF_M;
            gate_hold_ms = delay >= 250.0f && delay <= 60000.0f ? (int)delay : GATE_HOLD_MS;
            event(host, "gate: precision on (max half size %.2f m, reposition delay %d ms, movefire %d)",
                  gate_half_m, gate_hold_ms, cfg.movefire);
        }
        if (!(time >= gh_gun_reload_end(gun)) || gh_gun_state_flag_a(gun) || !gh_gun_state_flag_b(gun) || !(gh_gun_timer_a(gun) < 0.0f)
            || !(gh_gun_timer_b(gun) < 0.0f))
            goto leave;                    /* the native will not fire now */
        u8 *unit = gh_gun_unit(gun);
        int seat = unit ? bot_seat(unit) : -1;
        if (seat < 0 || !((cfg.brain_mask >> (seat - 1)) & 1) || !is_tank(unit))
            goto leave;
        u8 *turrets = gh_unit_turrets(unit);
        u8 *recs = turrets ? gh_turrets_records(turrets) : NULL;
        if (!recs || !gh_turrets_count(turrets) || gh_turret_gun(recs) != gun)
            goto leave;                    /* not the main gun */
        uint64_t ms = gh_ms();
        if (ms >= gate_stat.next_ms) {  /* summary: one line per minute */
            if (gate_stat.next_ms && gate_stat.releases + gate_stat.holds + gate_stat.move_holds)
                event(host, "gate: last 60 s %d releases (wait %.2f s avg, error %.2f deg avg), %d reposition requests, "
                      "%d holds, %d held calls, %d without a current aim, %d native aim fallbacks | movefire %d holds "
                      "in %d calls (beyond stabiliser band %d held calls, %d cleared movement hold after %d ms)",
                      gate_stat.releases, gate_stat.releases ? gate_stat.wait_sum / gate_stat.releases : 0.0,
                      gate_stat.releases ? gate_stat.err_sum / gate_stat.releases : 0.0, gate_stat.reposition,
                      gate_stat.holds, gate_stat.blocked, gate_stat.no_aim, gate_stat.fallback, gate_stat.move_holds,
                      gate_stat.move_calls, gate_stat.band_holds, gate_stat.band_released, GATE_BAND_MAX_MS);
            memset(&gate_stat, 0, sizeof gate_stat);
            gate_stat.next_ms = ms + 60000;
        }

        /* movefire (2026-11-11, design rule: without a vertical stabiliser do not fire
         * on the move): hold every round while the tank moves faster than
         * GATE_MOVE_MPS and the engine does not stabilise the gun vertically
         * (its vertical engine-stabiliser flag; also off over the stabiliser's speed limit). No forced
         * release: the fire stop (A3) brings it to a halt. Independent of the
         * aim self-check below. */
        gate_seat_t *gm = &gate_seat[seat];
        if (cfg.movefire && lseat[seat].unit == unit) {
            float v = sqrtf(lseat[seat].vx * lseat[seat].vx + lseat[seat].vz * lseat[seat].vz);
            ++gate_stat.move_calls;
            /* stab2 (8c46: with the native stabiliser 38 of 39 moving shots were
             * at 500-1500 m): stabilised tanks fire on the move only inside
             * their band (vertical only FSTOP_VONLY_M, both FSTOP_LONG_M, the
             * A3 rule); beyond it the round waits for the A3 stop, released
             * after GATE_BAND_MAX_MS of holding (they do have a stabiliser). */
            /* 6226: released after 4 s regardless, those shots had |yaw err| med
             * 2.1 deg (8c46 0.4): the release now also needs the turret on the
             * gunner's target (hull + sign * A0 within GATE_BAND_ON_DEG). */
            int mf = seat_movefire(unit);
            int over = 0;
            float band_d = 0.0f;
            if (mf > 0) {
                const lane_seat_t *ls = &lseat[seat];
                int tg = ls->gt_slot;
                if (tg >= 0 && tg < 64 && slot_alive[tg] && slot_unit[tg]) {
                    const float *tp = gh_unit_pos(slot_unit[tg]);
                    band_d = dist2d(ls->x, ls->z, tp[0], tp[2]);
                    over = band_d > (mf == 2 ? FSTOP_LONG_M : FSTOP_VONLY_M);
                    if (over && gm->move_unit == unit && gm->move_since && ms - gm->last_ms <= 500
                        && ms - gm->move_since > GATE_BAND_MAX_MS) {
                        int on = 1;
                        if (yaw_sign && band_d > 1.0f) {
                            float hull = atan2f(ls->fz, ls->fx) * 57.29578f;
                            float brg = atan2f(tp[2] - ls->z, tp[0] - ls->x) * 57.29578f;
                            on = fabsf(wrap180(hull + (float)yaw_sign * gh_turret_yaw(recs) - brg)) <= GATE_BAND_ON_DEG;
                        }
                        if (on) {
                            over = 0;
                            ++gate_stat.band_released;
                        }
                    }
                }
            }
            if (v > GATE_MOVE_MPS && (mf == 0 || over)) {
                if (over) {
                    static int band_logs;
                    ++gate_stat.band_holds;
                    if ((gm->move_unit != unit || !gm->move_since || ms - gm->last_ms > 500) && band_logs < 40) {
                        const lane_seat_t *ls = &lseat[seat];
                        ++band_logs;
                        event(host, "gate: seat %d stabiliser %s, target %.0f m beyond its band, %.1f m/s: round held "
                              "(A3 stopping %d, pusher %d, cool %d, cover %d, mode %d)", seat, mf == 2 ? "full" : "vertical",
                              band_d, v, ls->stopping, ls->pusher, ms < ls->stop_cool_until, ls->cov_state, ls->mode);
                    }
                }
                if (gm->move_unit != unit || !gm->move_since || ms - gm->last_ms > 500) {
                    gm->move_unit = unit;
                    gm->move_since = ms;
                    ++gate_stat.move_holds;
                }
                gm->last_ms = ms;
                gh_atomic_inc(&gate_blocks[seat]);
                hold = 1;
                goto leave;
            }
        }
        if (gm->move_unit == unit && gm->move_since) {
            /* Held by movefire, released now: restart the reload clock at the shot
             * (as the aim gate does below). */
            if (gh_gun_reload_end(gun) < time)
                gh_gun_set_reload_end(gun, time);
            gm->rel_move_wait = (float)(ms - gm->move_since) / 1000.0f;
            gm->move_since = 0;
        }
        if (gate_state == 2 || !slot_alive[seat] || slot_unit[seat] != unit)
            goto leave;
        const uint64_t sl = slot_life[seat];
        if (gm->unit != unit || gm->shooter_life != sl) {
            gm->unit = unit;
            gm->shooter_life = sl;
            gm->target = NULL;
            gm->eval_ms = gm->hold_since = gm->rel_ms = 0;
            gm->reload_held = 0;
        }
        const int ts = gunner_target(host, unit, gh_unit_army(unit));
        if (ts < 0) {
            gm->eval_ms = gm->hold_since = 0;
            gate_resume_reload(gm, gun, time);
            ++gate_stat.no_aim;
            goto leave;
        }
        u8 *target = slot_unit[ts];
        const uint64_t tl = slot_life[ts];
        const aim_rec_t *ar = &aim_rec[seat][ts];
        const int have_aim = ar->shooter == unit && ar->shooter_life == sl && ar->target == target
                             && ar->target_life == tl && ar->ms && ms - ar->ms <= GATE_AIM_MS;
        const int opportunity = have_aim ? ar->opportunity : SHOT_UNKNOWN;
        const int node = have_aim ? ar->node : -1;
        const char *src = !have_aim ? "gunner" : ar->native ? "native" : "aim";
        if (!have_aim || ar->native)
            ++gate_stat.fallback;

        const float *tp = gh_unit_pos(target);
        const float *up = gh_unit_pos(unit);
        const float *gp = gh_gun_position(gun);
        const float fallback_pos[3] = {up[0], up[1] + 1.5f, up[2]};
        if (!aim_point_sane(gp, up))
            gp = fallback_pos;
        float line[3];
        for (int a = 0; a < 3; ++a)
            line[a] = tp[a] - gp[a] + (have_aim ? ar->point[a] - ar->origin[a] : a == 1 ? 1.0f : 0.0f);
        const float d = hypotf(hypotf(line[0], line[2]), line[1]);
        if (!(d > 1.0f && d < 20000.0f)) {
            gm->eval_ms = gm->hold_since = 0;
            gate_resume_reload(gm, gun, time);
            goto leave;
        }
        const float raw_ay = gh_turret_yaw(recs), raw_ap = gh_turret_pitch(recs);
        const float raw_cy = gh_turret_cmd_yaw(recs), raw_cp = gh_turret_cmd_pitch(recs);
        const float ap = wrap180(raw_ap), cp = wrap180(raw_cp);
        const int command_valid = fabsf(raw_ay) < 720.0f && fabsf(raw_cy) < 720.0f && fabsf(raw_ap) < 720.0f
                                  && fabsf(raw_cp) < 720.0f && fabsf(ap) < 89.0f && fabsf(cp) < 89.0f;
        if (!command_valid && opportunity != SHOT_NONE) {
            gm->eval_ms = gm->hold_since = 0;
            gate_resume_reload(gm, gun, time);
            if (ms >= gate_bad_cmd_ms[seat]) {
                gate_bad_cmd_ms[seat] = ms + 8000;
                event(host, "gate: seat %d target %d turret angles unavailable, native fire", seat, ts);
            }
            goto leave;
        }
        if (command_valid)
            gate_armed = 1;
        const float ay = wrap180(raw_ay), cy = wrap180(raw_cy);
        float err = fabsf(wrap180(ay - cy)) * cosf(cp / 57.29578f);
        float perr = ap - cp;
        int command_off_target = 0;
        float cmd_dyaw = 0.0f, cmd_dpitch = 0.0f;  /* s45: record only (shots.csv), the two differences tested below */
        int cmd_dir_ok = 0;
        if (command_valid && yaw_sign && aim_frame_sane(gh_unit_frame(unit))) {
            float actual[3], commanded[3];
            gate_dir(unit, ay, ap, actual);
            gate_dir(unit, cy, cp, commanded);
            const float cmd_yaw = atan2f(commanded[2], commanded[0]);
            const float cmd_pitch = atan2f(commanded[1], hypotf(commanded[0], commanded[2]));
            err = fabsf(wrap180((atan2f(actual[2], actual[0]) - cmd_yaw) * 57.29578f)) * cosf(cmd_pitch);
            perr = (atan2f(actual[1], hypotf(actual[0], actual[2])) - cmd_pitch) * 57.29578f;
            /* A wide check catches a native command for another target or an
             * unreachable aim. The native trajectory determines exact pitch. */
            const float line_yaw = atan2f(line[2], line[0]), line_pitch = atan2f(line[1], hypotf(line[0], line[2]));
            command_off_target = fabsf(wrap180((cmd_yaw - line_yaw) * 57.29578f)) > 14.0f
                                 || fabsf((cmd_pitch - line_pitch) * 57.29578f) > 18.0f;
            cmd_dyaw = wrap180((cmd_yaw - line_yaw) * 57.29578f);
            cmd_dpitch = (cmd_pitch - line_pitch) * 57.29578f;
            cmd_dir_ok = 1;
        }
        const float half_w = have_aim ? ar->half_w : 0.65f, half_h = have_aim ? ar->half_h : 0.40f;
        const float tol = atanf(fminf(gate_half_m, 0.7f * half_w) / d) * 57.29578f;
        const float ptol = atanf(fminf(gate_half_m, 0.7f * half_h) / d) * 57.29578f;
        if (gm->target != target || gm->target_life != tl || gm->opportunity != opportunity || gm->node != node) {
            gm->target = target;
            gm->target_life = tl;
            gm->opportunity = opportunity;
            gm->node = node;
            gm->hold_since = gm->scene_ms = 0;
            gm->occluded = 0;
        } else if (gm->hold_since && ms - gm->eval_ms > 500) {
            gm->hold_since = 0;         /* native stopped attempting the shot */
        }
        gm->eval_ms = gm->last_ms = ms;
        gm->precision_blocked = command_valid && (!(err <= tol && fabsf(perr) <= ptol) || command_off_target);
        if (!gm->precision_blocked && opportunity > SHOT_UNKNOWN && opportunity < SHOT_NONE && d > 5.0f) {
            if (!gm->scene_ms || ms - gm->scene_ms >= 120) {
                for (int a = 0; a < 3; ++a)
                    line[a] /= d;
                gm->occluded = aim_scene_line(host, gp, line, d) == 0;
                gm->scene_ms = ms;
            }
        }
        if (gm->precision_blocked || gm->occluded || opportunity == SHOT_NONE) {
            if (!gm->hold_since) {
                gm->hold_since = ms;
                gm->reposition = 0;
                ++gate_stat.holds;
            }
            if (!gm->reposition && ms - gm->hold_since >= (uint64_t)gate_hold_ms) {
                gm->reposition = 1;
                ++gate_stat.reposition;
            }
            gm->reload_held = 1;
            gh_atomic_inc(&gate_blocks[seat]);
            ++gate_stat.blocked;
            hold = 1;
            goto leave;
        }

        /* Native advances reload from the reload end. A held shot restarts it at release. */
        gate_resume_reload(gm, gun, time);
        gm->rel_err = err;
        gm->rel_tol = tol;
        gm->rel_perr = perr;
        gm->rel_ptol = ptol;
        gm->rel_wait = gm->hold_since ? (float)(ms - gm->hold_since) / 1000.0f : 0.0f;
        gm->rel_src = src;
        gm->rel_ms = ms;
        gm->rel_target = target;
        gm->rel_dir_ok = cmd_dir_ok;    /* s45: record only */
        gm->rel_dyaw = cmd_dyaw;
        gm->rel_dpitch = cmd_dpitch;
        gm->rel_ts = ts;
        gm->rel_shooter_life = sl;
        gm->rel_target_life = tl;
        aim_shell_t sh;
        const float speed = aim_shell(host, unit, &sh) ? sh.speed : 600.0f;
        gm->rel_flight = (uint64_t)(d * 1000.0f / speed);
        gm->hold_since = 0;
        gm->reposition = 0;
        ++gate_stat.releases;
        gate_stat.wait_sum += gm->rel_wait;
        gate_stat.err_sum += err;
    }
leave:
    context->hold = hold;
}
int ground_fire(u8 *gun, void *host, float time)
{
    gh_once(GH_ONCE_INIT, init);
    if (!cfg.loaded || !cfg.gate || !gun || !host)
        return 0;
    int hold = 0;
    gh_rw_write_lock(GH_RW_AIM);
    gate_entry_ctx_t context = {gun, host, time, hold};
    const int faulted = gh_guard(gate_entry_run, &context);
    hold = context.hold;
    if (faulted) {
        hold = 0;
        if (gh_atomic_inc(&gate_faults) >= 3) {
            gate_armed = 0;
            gate_state = 2;
        }
    }
    gh_rw_write_unlock(GH_RW_AIM);
    return hold;
}

