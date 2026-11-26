/* ---- Bot tank physics tick rate ----------------------------------------------
 * The tank behaviour tree (and with it the game's own move order, wt_ground_plan and the tracker) runs only when
 * the unit's physics tick number changes (the physics object's tick against the bot's); bot tanks get a physics
 * timestep of 1/24 s, so everything the AI does is clocked at 24 Hz.
 * The game sets a unit's timestep through one of the unit's own functions (gh_set_unit_timestep): it stores it
 * in the unit's physics object (gh_physics_timestep) and recomputes the player connection's controls tick delta.
 * The engine already uses 1/48 s for some units.
 * Here: once per tree tick of a bot tank, if its timestep differs from 1/bot_phys_hz, call that same function
 * with the target (the game's own call pattern). bot_phys_hz 0 = never touch it.
 * A fault in the call turns the feature off for the rest of the process with one event line. */
#define PHYS_LOG_MAX 64

static int phys_rate_dead;
static unsigned phys_rate_sets, phys_rate_logged;

/* s49: the guarded block of phys_rate_tick, run under the host's fault guard (gh_guard, ground_host.h). */
typedef struct { void *host; u8 *unit; float want; } phys_rate_ctx_t;
static void phys_rate_run(void *ctx)
{
    phys_rate_ctx_t *const context = ctx;
    void *const host = context->host;
    u8 *const unit = context->unit;
    const float want = context->want;
    {
        u8 *phys = gh_unit_physics(unit);
        if (!phys)
            return;
        const float cur = gh_physics_timestep(phys);
        if (fabsf(cur - want) < 1e-6f)
            return;
        gh_set_unit_timestep(unit, want);
        const float now = gh_physics_timestep(phys);
        phys_rate_sets++;
        if (phys_rate_logged < PHYS_LOG_MAX) {
            phys_rate_logged++;
            event(host, "phys: unit %p timestep %.4f s (%.1f Hz) -> %.4f s (%.1f Hz) via the unit's timestep setter%s",
                  (void *)unit, cur, cur > 0.0f ? 1.0f / cur : 0.0f, now, now > 0.0f ? 1.0f / now : 0.0f,
                  gh_last_timestep_setter_is_base(host) ? " (base)" : " (override)");
        }
    }
}

static void phys_rate_tick(void *host, u8 *unit)
{
    if (phys_rate_dead || cfg.bot_phys_hz <= 0 || !unit)
        return;
    const float want = 1.0f / (float)cfg.bot_phys_hz;
    phys_rate_ctx_t context = {host, unit, want};
    if (gh_guard(phys_rate_run, &context)) {
        phys_rate_dead = 1;
        event(host, "phys: setting the bot physics timestep raised an exception; left at the game's value from now on");
    }
}
