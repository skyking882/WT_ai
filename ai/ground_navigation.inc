/* Goal arbitration and route progress. The host adapts native pointers separately. */
#define NAV_GOAL_VALID_MS 500
#define NAV_PORTAL_M 3.5f
#define NAV_LATERAL_M 4.5f


static void nav_propose(nav_state_t *nav, enum nav_source source, float x, float z,
                        enum nav_path_mode path, int move, int parked, int brake)
{
    nav_goal_t *goal = &nav->proposals[source];
    *goal = (nav_goal_t){nav->life_id, nav->task_id, nav->epoch_id, nav->sampled_ms + NAV_GOAL_VALID_MS,
                        nav->revision, nav->generation, x, z, path,
                        isfinite(x) && isfinite(z), move, parked, brake};
}

static const nav_goal_t *nav_choose(const nav_state_t *nav, uint64_t ms, int clearing, enum nav_source *source)
{
    const nav_goal_t *hold = NULL;
    enum nav_source hold_source = NAV_SOURCE_COUNT;
    for (int i = NAV_SOURCE_COUNT - 1; i >= NAV_LANE; --i) {
        const nav_goal_t *goal = &nav->proposals[i];
        if (!goal->valid || goal->epoch_id != nav->epoch_id || goal->life_id != nav->life_id
            || goal->task_id != nav->task_id || goal->revision != nav->revision
            || goal->generation != nav->generation || ms > goal->valid_until_ms)
            continue;
        if (clearing && (i == NAV_REACTION || i == NAV_FIGHT || i == NAV_RECOVERY))
            continue;
        if (clearing && !goal->move) {
            if (!hold) {
                hold = goal;
                hold_source = (enum nav_source)i;
            }
            continue;
        }
        *source = (enum nav_source)i;
        return goal;
    }
    *source = hold_source;
    return hold;
}

static void nav_tick_begin(nav_state_t *nav, uint64_t life, uint64_t ms)
{
    ++nav->epoch_id;
    nav->life_id = life;
    nav->sampled_ms = ms;
    memset(nav->proposals, 0, sizeof nav->proposals);
}

static float nav_leg_length(float x, float z, float tx, float tz)
{
    return hypotf(tx - x, tz - z);
}

typedef struct {
    float arc, lateral, forward, alignment;
    int valid;
} nav_projection_t;

static nav_projection_t nav_project(float x, float z, float fx, float fz, float ax, float az,
                                   float bx, float bz, float start_m)
{
    nav_projection_t p = {0};
    const float dx = bx - ax, dz = bz - az, length = nav_leg_length(ax, az, bx, bz);
    if (length < 0.1f)
        return p;
    p.forward = ((x - ax) * dx + (z - az) * dz) / length;
    p.lateral = fabsf((x - ax) * dz - (z - az) * dx) / length;
    p.alignment = (fx * dx + fz * dz) / length;
    p.arc = start_m + fmaxf(0.0f, fminf(length, p.forward));
    p.valid = p.lateral <= NAV_LATERAL_M && p.forward >= -NAV_PORTAL_M
              && p.forward <= length + NAV_PORTAL_M && p.alignment >= 0.35f;
    return p;
}
