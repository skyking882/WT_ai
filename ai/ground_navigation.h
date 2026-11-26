#ifndef GROUND_NAVIGATION_H
#define GROUND_NAVIGATION_H

#include <stdint.h>

/* A goal proposal belongs to this planning opportunity and unit/task/route. */
enum nav_source {
    NAV_LANE,
    NAV_ROUTE,
    NAV_FIGHT,
    NAV_REACTION,
    NAV_RECOVERY,
    NAV_SOURCE_COUNT
};

enum nav_geometry { NAV_UNKNOWN, NAV_CLEAR, NAV_BLOCKED };
enum nav_path_mode { NAV_NATIVE, NAV_REPATH_NOW, NAV_DIRECT, NAV_PARK };

typedef struct {
    uint64_t life_id, task_id, epoch_id, valid_until_ms;
    unsigned revision, generation;
    float x, z;
    enum nav_path_mode path;
    int valid, move, parked, route_brake;
} nav_goal_t;

typedef struct {
    uint64_t epoch_id, life_id, task_id, sampled_ms, feedback_ms, blocked_since_ms;
    uint64_t retry_after_ms, next_replan_ms, recovery_ms, reported_ms;
    unsigned revision, generation;
    int state, reason, reported_state, reported_reason, reported_route;
    unsigned reported_generation, reported_revision;
    int waiting, blocked, clearing, recovery_result, needs_rejoin;
    int route_id, entry, previous_segment, valid_projection, route_active, route_tactical;
    float origin_x, origin_z, actual_m, completed_m, arc[MAX_TWP];
    float prev_x, prev_z;
    uint64_t pose_ms, route_task_id;
    unsigned route_revision;
    enum nav_source applied_source;
    nav_goal_t proposals[NAV_SOURCE_COUNT];
} nav_state_t;

#endif
