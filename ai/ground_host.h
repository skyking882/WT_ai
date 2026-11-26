/* ground_host.h (s49, s50): the host layer, as the AI sees it. The AI (ground_brain.c and its .inc files, the fire
 * control of ground_fire.inc since s50, the command unit, the driving units) reaches the game and the operating system
 * only through these functions; ground_host.c implements them and is, with ground_host_private.h, the only code that
 * knows how to reach into this particular game. Each gh_* function gives one fact or does one action, named after
 * what it means (s50: never after a layout offset; an argument never carries an offset or a code address).
 * Game objects are opaque handles (unsigned char *, the AI's u8 *, and void *host for the game's context): the
 * AI stores, compares and hands them back; float / double pointers it gets back point at the game's own vectors.
 * Plain C only (no OS header), so every public file can include it. */
#ifndef GROUND_HOST_H
#define GROUND_HOST_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* ---- OS services -------------------------------------------------------------------------------------------- */
uint64_t gh_ms(void);                                   /* milliseconds since the system started */
int gh_qpc(long long *count);                           /* the performance counter now */
int gh_qpc_frequency(long long *frequency);             /* its counts per second */
unsigned long gh_env(const char *name, char *out, unsigned long size);   /* an environment variable */
enum { GH_LOCK_MAIN, GH_LOCK_KNOW, GH_LOCK_HUMAN_VIS, GH_LOCK_N };       /* lock: the tick, team knowledge, human_vis.csv */
void gh_lock_init(int id);
void gh_lock(int id);
void gh_unlock(int id);
enum { GH_RW_AIM };                                     /* scan identities and aim / gate state */
void gh_rw_write_lock(int id);
void gh_rw_write_unlock(int id);
void gh_rw_read_lock(int id);
void gh_rw_read_unlock(int id);
enum { GH_ONCE_INIT, GH_ONCE_HUMAN_VIS };               /* the config init, the human_vis lock */
void gh_once(int id, int (*fn)(void));                  /* fn runs once per process and id */
int gh_guard(void (*fn)(void *), void *ctx);            /* runs fn(ctx) under the fault guard: 1 when it faulted */
long gh_atomic_inc(volatile long *value);               /* the incremented value */
long gh_atomic_swap(volatile long *value, long to);     /* the old value */
long long gh_atomic_cas64(volatile long long *value, long long to, long long expected);   /* the old value */
void *gh_file_open(const char *path);                   /* a file opened for reading (check with gh_file_failed) */
int gh_file_failed(void *file);
int gh_file_size(void *file, long long *size);
void *gh_file_map(void *file);                          /* a read-only mapping of the file, NULL on failure */
void *gh_file_view(void *map);                          /* its bytes, NULL on failure */
int gh_file_unview(const void *view);
int gh_handle_close(void *handle);
int gh_file_replace(const char *from, const char *to);  /* rename from over to; nonzero on success */

/* ---- game: the log and the indivisible access sequences ------------------------------------------------------ */
void gh_log(void *host, const char *msg);               /* one line into the game's own log */

/* State of the gun in one turret record (gh_gun_state). */
typedef struct {
    int is_gun;                 /* record holds a gun object */
    int nsets, cur;
    int shots;                  /* sum of the per-set shot counters */
    int left;                   /* sum of rounds left */
    int alt;                    /* the gun's other shot count (used when not simulating as server) */
    float last_t;               /* time of the last shot (the game clock the gun was given) */
    float next_t;               /* reload end, same clock (the game's own fire call waits for it) */
} gun_state_t;

float gh_height_query(void *host, unsigned char *land, float x, float y, float z);
void gh_player_uid_scan(FILE *scan, const unsigned char *player);
uint64_t gh_gunner_target(void *host, unsigned char *unit);
unsigned gh_sight_lookup(void *host, void **tables, unsigned char *target, unsigned char *observer);
int gh_turret_yaw_limits(unsigned char *turrets, unsigned i, float *lo, float *hi);
int gh_gun_state(void *host, unsigned char *unit, unsigned idx, gun_state_t *g);
int gh_weapon_ai(unsigned char *unit, unsigned idx, float *yaw, float *pitch, float *cone);
int gh_turret_pitch_limits(unsigned char *turrets, float a0, float *lo, float *hi);
int32_t gh_gun_shell(void *host, unsigned char *unit, int back);
void gh_unit_velocity(const unsigned char *unit, float fallback_x, float fallback_z, float *x, float *z);
const float *gh_vphys_gear_ratios(const unsigned char *phys, unsigned *count);
int gh_vphys_drive_params(const unsigned char *phys, int *tracked, int *separate, int *boost, float *mass, float *brake);
int gh_riex_release(void *host, unsigned layer, int64_t *was, int32_t *pending);
int gh_ray(void *host, const float *p, const float *dir, float *t, int flags, int *kind, unsigned char *hit);
/* s50: the unit's "set physics timestep" function is called and remembered by the host (no code address here). */
void gh_set_unit_timestep(unsigned char *unit, float want);
int gh_last_timestep_setter_is_base(void *host);    /* whether it was the base implementation (log; s51: the host
                                                       may write its own detail line here) */

/* s50: the fire control's sequences. */
int gh_stabiliser_code_matches(void *host);     /* the game code the stabiliser write relies on is the one read statically */
int gh_stabiliser_patch_present(void *host);    /* the launcher's stabiliser patch for bot tanks is in place */
unsigned char *gh_shell_props(void *host, int32_t id);  /* the properties of a shell id (the game's shell registry) */
int gh_unit_entity_id_find(void *host, unsigned char *a, unsigned char *b);  /* finds the units' entity-id field: 1 found,
                                                                               -1 none (s51: no value handed over) */
uint32_t gh_unit_entity_id(const unsigned char *unit);  /* the unit's entity id (the field gh_unit_entity_id_find found) */
/* The input of the game's shell-vs-damage-model test (gh_shell_test copies it into the game's own input). */
typedef struct {
    float pos[3], dir[3];       /* ray start (world), unit direction */
    float len;                  /* ray length, m */
    float speed;                /* shell speed, m/s */
    float dist;                 /* distance for the penetration table, m */
    uint8_t mode, full, flag;   /* test mode; 1: the full pass with crew / ammo checks; the hit flag */
} gh_shell_test_t;
int gh_shell_test(void *host, uint32_t eid, int32_t id, const gh_shell_test_t *test, int *out);  /* 1 on a fault, else
                                                                                                   *out = the result */

/* s50: collision-ray flags (gh_ray, gh_ray_cast); the values are the host's. */
extern const int GH_RAY_LAND_MESH;              /* the land mesh */
extern const int GH_RAY_GROUND;                 /* land mesh and heightmap */
extern const int GH_RAY_STATIC_AND_OBJECTS;     /* static collision and placed objects (buildings, walls, trees) */
extern const int GH_RAY_GROUND_AND_OBJECTS;     /* ground, static collision and placed objects */

/* ---- game: single facts (one expression each; generated from tools_split_host_table.py) ---- */
/* ---- accessors begin ---- */
const float * gh_unit_pos(const unsigned char *unit);
float gh_unit_fwd_x(const unsigned char *unit);
float gh_unit_fwd_z(const unsigned char *unit);
const float * gh_unit_frame(const unsigned char *unit);
const float * gh_unit_up(const unsigned char *unit);
uint32_t gh_unit_not_active(const unsigned char *unit);
uint32_t gh_unit_not_present(const unsigned char *unit);
unsigned char * gh_unit_player(const unsigned char *unit);
unsigned char gh_unit_army(const unsigned char *unit);
unsigned char * gh_unit_name(const unsigned char *unit);
unsigned char * gh_unit_description(const unsigned char *unit);
unsigned char * gh_description_model(const unsigned char *description);
unsigned char * gh_unit_ai(const unsigned char *unit);
uint16_t gh_unit_kind(const unsigned char *unit);
unsigned char gh_unit_burning(const unsigned char *unit);
unsigned char * gh_unit_controls(const unsigned char *unit);
void gh_unit_set_los_interval(unsigned char *unit, int32_t value);
int32_t gh_unit_los_interval(const unsigned char *unit);
unsigned char * gh_unit_turrets(const unsigned char *unit);
unsigned char * gh_unit_physics(const unsigned char *unit);
float gh_physics_timestep(const unsigned char *phys);
unsigned char * gh_detector_target(const unsigned char *detector);
unsigned char gh_player_slot(const unsigned char *player);
int gh_player_bot(const unsigned char *player);
uint16_t gh_player_spawned(const unsigned char *player);
const unsigned char *const * gh_players(void *host);
const unsigned char * gh_player_at(const unsigned char *const *players, int seat);
int gh_hit_army(const unsigned char *record);
float gh_hit_damage(const unsigned char *record);
int gh_hit_parts(const unsigned char *record);
int gh_hit_kind(const unsigned char *record);
int gh_hit_count(const unsigned char *record);
unsigned char * gh_bot_target(const unsigned char *bot);
float gh_bot_hit_time(const unsigned char *bot);
unsigned char * gh_game(void *host);
unsigned char * gh_game_land(const unsigned char *game);
unsigned char ** gh_game_units(const unsigned char *game);
uint32_t gh_game_unit_count(const unsigned char *game);
unsigned char * gh_unit_at(unsigned char **units, uint32_t i);
unsigned char * gh_game_zones(const unsigned char *game);
unsigned char ** gh_zone_list(const unsigned char *zones);
uint32_t gh_zone_count(const unsigned char *zones);
unsigned char * gh_zone_at(unsigned char **zones, uint32_t i);
unsigned char * gh_zone_shape(const unsigned char *zone);
unsigned char gh_zone_owner(const unsigned char *zone);
uint16_t gh_zone_flags(const unsigned char *zone);
uint32_t gh_shape_type(const unsigned char *shape);
const float * gh_shape_centre(const unsigned char *shape, uint32_t type);
const float * gh_shape_extent(const unsigned char *shape);
void ** gh_game_sight_tables(const unsigned char *game);
void * gh_sight_table(void **tables);
float gh_height_pad(void *host);
const float * gh_ai_dest(const unsigned char *ai);
float gh_ai_dest_x(const unsigned char *ai);
float gh_ai_dest_y(const unsigned char *ai);
float gh_ai_dest_z(const unsigned char *ai);
void gh_ai_set_dest(unsigned char *ai, float x, float y, float z);
void gh_ai_set_hold(unsigned char *ai, float value);
void gh_ai_set_repath(unsigned char *ai, float value);
void gh_ai_clear_path(unsigned char *ai);
void gh_ai_set_zone(unsigned char *ai, int value);
const float * gh_ai_path(const unsigned char *ai);
unsigned gh_ai_path_count(const unsigned char *ai);
unsigned gh_ai_path_cursor(const unsigned char *ai);
void gh_node_set_arrival(unsigned char *node, float value);
float gh_node_arrival(const unsigned char *node);
uint32_t gh_path_points(const unsigned char *path);
const float * gh_path_from(const unsigned char *request);
const float * gh_path_to(const unsigned char *request);
unsigned char * gh_turrets_records(const unsigned char *turrets);
uint32_t gh_turrets_count(const unsigned char *turrets);
float gh_turret_yaw(const unsigned char *rec);
void gh_turret_set_pitch(unsigned char *rec, float value);
float gh_turret_pitch(const unsigned char *rec);
void gh_turret_set_ext_pitch(unsigned char *rec, float value);
float gh_turret_ext_pitch(const unsigned char *rec);
void gh_turret_set_cmd_pitch(unsigned char *rec, float value);
float gh_turret_cmd_pitch(const unsigned char *rec);
unsigned char * gh_turret_gun(const unsigned char *rec);
unsigned char * gh_gun_props(const unsigned char *gun);
unsigned char * gh_gun_stab_object(const unsigned char *props);
unsigned char gh_stab_axis_h(const unsigned char *so);
unsigned char gh_stab_axis_v(const unsigned char *so);
float gh_stab_omega_h(const unsigned char *so);
float gh_stab_omega_v(const unsigned char *so);
float gh_stab_limit_h(const unsigned char *so);
float gh_stab_limit_v(const unsigned char *so);
float gh_stab_speed_yaw(const unsigned char *so);
float gh_stab_speed_pitch(const unsigned char *so);
uint32_t gh_coll_has_names(const unsigned char *hdr);
uint32_t gh_coll_count(const unsigned char *hdr);
const unsigned char * gh_coll_nodes(const unsigned char *hdr);
const char * gh_coll_names(const unsigned char *hdr);
const char * gh_coll_node_name(const char *names, const unsigned char *nodes, uint32_t i);
void gh_drive_set_throttle(unsigned char *phys, unsigned char value);
void gh_drive_set_steer(unsigned char *phys, unsigned char value);
void gh_drive_release_brakes(unsigned char *controls);
unsigned char gh_drive_throttle(const unsigned char *controls);
unsigned char gh_drive_steer(const unsigned char *controls);
unsigned char gh_drive_brake_left(const unsigned char *controls);
unsigned char gh_drive_brake_right(const unsigned char *controls);
void gh_drive_write(unsigned char *controls, const void *src, size_t n);
void gh_drive_read(void *out, const unsigned char *phys, size_t n);
void gh_vphys_applied(void *out, const unsigned char *phys, size_t n);
uint32_t gh_vphys_tick(const unsigned char *phys);
uint8_t gh_vphys_gear(const unsigned char *phys);
uint8_t gh_vphys_gear_request(const unsigned char *phys);
float gh_vphys_mass(const unsigned char *phys);
float gh_vphys_max_brake(const unsigned char *phys);
float gh_vphys_wheel_radius(const unsigned char *phys);
int gh_vphys_neutral_gear(const unsigned char *phys);
float gh_vphys_rear_engage_speed(const unsigned char *phys);
float gh_vphys_min_shift_time(const unsigned char *phys);
uint8_t gh_vphys_tracked(const unsigned char *phys);
int gh_vphys_steer_type(const unsigned char *phys);
float gh_vphys_timestep(const unsigned char *phys);
const double * gh_vphys_velocity(const unsigned char *phys);
const double * gh_vphys_accel(const unsigned char *phys);
const double * gh_vphys_omega(const unsigned char *phys);
unsigned char gh_vphys_auto_gear(const unsigned char *phys);
unsigned char gh_vphys_input_flag_a(const unsigned char *phys);
unsigned char gh_vphys_input_flag_b(const unsigned char *phys);
unsigned char gh_vphys_input_flag_c(const unsigned char *phys);
void gh_turret_set_yaw(unsigned char *rec, float value);
void gh_turret_set_ext_yaw(unsigned char *rec, float value);
float gh_turret_ext_yaw(const unsigned char *rec);
void gh_turret_set_cmd_yaw(unsigned char *rec, float value);
float gh_turret_cmd_yaw(const unsigned char *rec);
unsigned char gh_turret_engine_stab_h(const unsigned char *rec);
unsigned char gh_turret_engine_stab_v(const unsigned char *rec);
unsigned char gh_turret_drive_intact(const unsigned char *rec);
unsigned char gh_turret_state_flag_a(const unsigned char *rec);
unsigned char gh_turret_vertical_blocked(const unsigned char *rec);
unsigned char gh_unit_sim_mode(const unsigned char *unit);
uint32_t gh_unit_stab_exempt(const unsigned char *unit);
unsigned char gh_player_flags(const unsigned char *player);
void gh_controls_set_stab_allowed(unsigned char *controls, unsigned char value);
unsigned char gh_controls_stab_allowed(const unsigned char *controls);
unsigned char gh_stab_free_mode(const unsigned char *so);
unsigned char gh_stab_forced(const unsigned char *so);
unsigned char gh_stab_patch_mask_a(void *host);
unsigned char gh_stab_patch_mask_b(void *host);
unsigned char * gh_unit_damage_model(const unsigned char *unit);
unsigned char * gh_dm_collision(const unsigned char *dm);
unsigned char * gh_collision_header(const unsigned char *coll);
void gh_node_box(void *host, unsigned char *coll, float *out, int n);
uint32_t gh_props_set_count(const unsigned char *props);
unsigned char gh_gun_current_set(const unsigned char *gun);
int gh_weapon_not_gun(void *host, const unsigned char *gun);
unsigned char * gh_props_set(unsigned char *props, uint32_t cur);
unsigned char ** gh_set_belt(unsigned char *set, uint32_t count);
uint32_t gh_set_rounds(const unsigned char *set);
uint16_t gh_set_shots(const unsigned char *set);
unsigned char * gh_belt_round(unsigned char **belt, uint32_t k);
int32_t gh_bullet_shell_id(const unsigned char *bullet);
int32_t gh_bullet_base_id(const unsigned char *bullet);
float gh_bullet_speed(const unsigned char *bullet);
uint32_t gh_shell_pen_rows(const unsigned char *props);
const float * gh_shell_pen_table(const unsigned char *props);
unsigned char * gh_entity_unit(void *host, uint32_t eid);
const float * gh_unit_aim_scatter(const unsigned char *unit);
const float * gh_unit_bbox(const unsigned char *unit);
unsigned char gh_ray_cast(void *host, const float *p, const float *dir, float *t, int flags, int *kind);
float gh_bot_scatter_vs_bot(void *host);
float gh_bot_scatter_vs_human(void *host);
unsigned char * gh_dm_settings(void *host);
float gh_dm_max_armor_scale(const unsigned char *dms);
float gh_dm_armor_breach_k(const unsigned char *dms);
float gh_dm_slope_angle_mult(const unsigned char *dms);
float gh_dm_ricochet_min(const unsigned char *dms);
float gh_dm_ricochet_max(const unsigned char *dms);
void gh_gun_set_reload_end(unsigned char *gun, float value);
float gh_gun_reload_end(const unsigned char *gun);
unsigned char gh_gun_state_flag_a(const unsigned char *gun);
unsigned char gh_gun_state_flag_b(const unsigned char *gun);
float gh_gun_timer_a(const unsigned char *gun);
float gh_gun_timer_b(const unsigned char *gun);
unsigned char * gh_gun_unit(const unsigned char *gun);
const float * gh_gun_position(const unsigned char *gun);
int gh_unit_is_tank(const unsigned char *unit);
int gh_unit_is_basic_tank(const unsigned char *unit);
int gh_zone_shape_is_area(uint32_t type);
int gh_uid_ai_slot_valid(int32_t uid);
int gh_ai_slot_of_uid(int32_t uid);
int gh_uid_is_human(int32_t uid);
int gh_uid_is_ai(int32_t uid);
int gh_ai_dest_height_none(float y);
int gh_ai_dest_height_valid(float y);
/* ---- accessors end ---- */

/* ---- the AI's entry points: the host's exported wt_ground_* functions call them with the same arguments ---- */
struct control_frame;
void ground_plan(unsigned char *unit, unsigned char *node, void *host, float dt);
void ground_hit(unsigned char *victim, uint64_t offender_uid, unsigned char *record, void *host);
int ground_visible(unsigned char *detector, unsigned char *observer, uint64_t flag, void *host);
void ground_path(unsigned char *unit, unsigned char *path, unsigned char *request, void *host);
int ground_los_interval(unsigned char *a, unsigned char *b, int native_ms, void *host);
void ground_control_begin(unsigned char *node, struct control_frame *frame);
int ground_control_after(unsigned char *node, void *host, struct control_frame *frame, unsigned native_status);
void ground_observe(void *host);
void ground_controls(unsigned char *unit, void *host, int phase);
void ground_aim(unsigned char *shooter, unsigned char *target, float *point, void *host);
int ground_fire(unsigned char *gun, void *host, float time);

#endif
