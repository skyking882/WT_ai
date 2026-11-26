/* host_stub/ground_host_stub.c: the skeleton of a host layer. The AI library reaches its environment only through
 * the functions and constants declared in ground_host.h; a host implements every one of them. Here each has its
 * one-sentence description and a trivial body (0 / NULL / nothing), so that the library links: build_public.cmd
 * links it against the library as the link test. Replace each body with the real access of your host.
 * The handles (unsigned char *, void *host) are opaque to the AI: it stores, compares and hands them back. */
#include "ground_host.h"

/* ---- collision-ray flags (gh_ray, gh_ray_cast): the host chooses the values ---- */
const int GH_RAY_LAND_MESH = 0;   /* the land mesh */
const int GH_RAY_GROUND = 0;   /* land mesh and heightmap */
const int GH_RAY_STATIC_AND_OBJECTS = 0;   /* static collision and placed objects (buildings, walls, trees) */
const int GH_RAY_GROUND_AND_OBJECTS = 0;   /* ground, static collision and placed objects */

/* ---- every gh_* function of ground_host.h ---- */
/* Milliseconds since the system started. */
uint64_t gh_ms(void)
{
    return 0;
}

/* The performance counter now. */
int gh_qpc(long long *count)
{
    (void)count;
    return 0;
}

/* The performance counter's counts per second. */
int gh_qpc_frequency(long long *frequency)
{
    (void)frequency;
    return 0;
}

/* The value of an environment variable of the process. */
unsigned long gh_env(const char *name, char *out, unsigned long size)
{
    (void)name;
    (void)out;
    (void)size;
    return 0;
}

/* Prepares one of the AI's locks. */
void gh_lock_init(int id)
{
    (void)id;
}

/* Takes one of the AI's locks. */
void gh_lock(int id)
{
    (void)id;
}

/* Releases one of the AI's locks. */
void gh_unlock(int id)
{
    (void)id;
}

/* Takes the aim / scan reader-writer lock for writing. */
void gh_rw_write_lock(int id)
{
    (void)id;
}

/* Releases it after writing. */
void gh_rw_write_unlock(int id)
{
    (void)id;
}

/* Takes the aim / scan reader-writer lock for reading. */
void gh_rw_read_lock(int id)
{
    (void)id;
}

/* Releases it after reading. */
void gh_rw_read_unlock(int id)
{
    (void)id;
}

/* Runs a function once per process (the config init, the human_vis lock). */
void gh_once(int id, int (*fn)(void))
{
    (void)id;
    (void)fn;
}

/* Runs a function under the fault guard and says whether it faulted. */
int gh_guard(void (*fn)(void *), void *ctx)
{
    (void)fn;
    (void)ctx;
    return 0;
}

/* Increments a counter atomically and returns the new value. */
long gh_atomic_inc(volatile long *value)
{
    (void)value;
    return 0;
}

/* Replaces a counter atomically and returns the old value. */
long gh_atomic_swap(volatile long *value, long to)
{
    (void)value;
    (void)to;
    return 0;
}

/* Replaces a 64-bit value if it still holds the expected one; returns the old value. */
long long gh_atomic_cas64(volatile long long *value, long long to, long long expected)
{
    (void)value;
    (void)to;
    (void)expected;
    return 0;
}

/* A file opened for reading. */
void *gh_file_open(const char *path)
{
    (void)path;
    return NULL;
}

/* Whether the open failed. */
int gh_file_failed(void *file)
{
    (void)file;
    return 0;
}

/* The file's size. */
int gh_file_size(void *file, long long *size)
{
    (void)file;
    (void)size;
    return 0;
}

/* A read-only mapping of the file. */
void *gh_file_map(void *file)
{
    (void)file;
    return NULL;
}

/* The mapped file's bytes. */
void *gh_file_view(void *map)
{
    (void)map;
    return NULL;
}

/* Gives a mapped view back. */
int gh_file_unview(const void *view)
{
    (void)view;
    return 0;
}

/* Closes a file or mapping handle. */
int gh_handle_close(void *handle)
{
    (void)handle;
    return 0;
}

/* Renames a file over another. */
int gh_file_replace(const char *from, const char *to)
{
    (void)from;
    (void)to;
    return 0;
}

/* Writes one line into the host's log. */
void gh_log(void *host, const char *msg)
{
    (void)host;
    (void)msg;
}

/* The terrain height under a point, as the game computes it for a capture destination. */
float gh_height_query(void *host, unsigned char *land, float x, float y, float z)
{
    (void)host;
    (void)land;
    (void)x;
    (void)y;
    (void)z;
    return 0;
}

/* Writes the player record's candidate AI-uid fields into the scan CSV (one-off probe). */
void gh_player_uid_scan(FILE *scan, const unsigned char *player)
{
    (void)scan;
    (void)player;
}

/* The unit the tank's gunner currently targets (0 none or unreadable). */
uint64_t gh_gunner_target(void *host, unsigned char *unit)
{
    (void)host;
    (void)unit;
    return 0;
}

/* The engine's line-of-sight table entry for a target and an observer (0 clear). */
unsigned gh_sight_lookup(void *host, void **tables, unsigned char *target, unsigned char *observer)
{
    (void)host;
    (void)tables;
    (void)target;
    (void)observer;
    return 0;
}

/* The yaw limits of a turret record, and whether its traverse is limited. */
int gh_turret_yaw_limits(unsigned char *turrets, unsigned i, float *lo, float *hi)
{
    (void)turrets;
    (void)i;
    (void)lo;
    (void)hi;
    return 0;
}

/* The state of the gun of a turret record (sets, shots, rounds left, reload times). */
int gh_gun_state(void *host, unsigned char *unit, unsigned idx, gun_state_t *g)
{
    (void)host;
    (void)unit;
    (void)idx;
    (void)g;
    return 0;
}

/* Yaw speed, pitch speed and fire cone of a weapon as the game's AI uses them. */
int gh_weapon_ai(unsigned char *unit, unsigned idx, float *yaw, float *pitch, float *cone)
{
    (void)unit;
    (void)idx;
    (void)yaw;
    (void)pitch;
    (void)cone;
    return 0;
}

/* The pitch limits of the main turret at a given yaw. */
int gh_turret_pitch_limits(unsigned char *turrets, float a0, float *lo, float *hi)
{
    (void)turrets;
    (void)a0;
    (void)lo;
    (void)hi;
    return 0;
}

/* The shell id of a round of the main gun's current belt. */
int32_t gh_gun_shell(void *host, unsigned char *unit, int back)
{
    (void)host;
    (void)unit;
    (void)back;
    return 0;
}

/* The unit's ground velocity from the game's own speed sources (the fallback where none). */
void gh_unit_velocity(const unsigned char *unit, float fallback_x, float fallback_z, float *x, float *z)
{
    (void)unit;
    (void)fallback_x;
    (void)fallback_z;
    (void)x;
    (void)z;
}

/* The vehicle's gear ratios and their count. */
const float *gh_vphys_gear_ratios(const unsigned char *phys, unsigned *count)
{
    (void)phys;
    (void)count;
    return NULL;
}

/* Tracked, separate brakes, steer boost, mass and maximum brake force of the vehicle. */
int gh_vphys_drive_params(const unsigned char *phys, int *tracked, int *separate, int *boost, float *mass, float *brake)
{
    (void)phys;
    (void)tracked;
    (void)separate;
    (void)boost;
    (void)mass;
    (void)brake;
    return 0;
}

/* Gives one scenery-object update layer no render owner and drops its pending queue; 1 when it did. */
int gh_riex_release(void *host, unsigned layer, int64_t *was, int32_t *pending)
{
    (void)host;
    (void)layer;
    (void)was;
    (void)pending;
    return 0;
}

/* Casts the game's collision ray (hit and distance; says whether it faulted). */
int gh_ray(void *host, const float *p, const float *dir, float *t, int flags, int *kind, unsigned char *hit)
{
    (void)host;
    (void)p;
    (void)dir;
    (void)t;
    (void)flags;
    (void)kind;
    (void)hit;
    return 0;
}

/* Calls the unit's own "set physics timestep" function with a timestep (s). */
void gh_set_unit_timestep(unsigned char *unit, float want)
{
    (void)unit;
    (void)want;
}

/* Whether the timestep function gh_set_unit_timestep last called is the base implementation (log only). */
int gh_last_timestep_setter_is_base(void *host)
{
    (void)host;
    return 0;
}

/* Whether the host can use its gun-stabiliser write for bot tanks (it checked the game it runs in). */
int gh_stabiliser_code_matches(void *host)
{
    (void)host;
    return 0;
}

/* Whether the host lets the engine stabilise bot tanks' guns itself. */
int gh_stabiliser_patch_present(void *host)
{
    (void)host;
    return 0;
}

/* The properties of a shell id (the game's shell registry). */
unsigned char *gh_shell_props(void *host, int32_t id)
{
    (void)host;
    (void)id;
    return NULL;
}

/* Finds, from two units, how the host gets a unit's entity id; 1 found, -1 none. */
int gh_unit_entity_id_find(void *host, unsigned char *a, unsigned char *b)
{
    (void)host;
    (void)a;
    (void)b;
    return 0;
}

/* The entity id of a unit (after gh_unit_entity_id_find found it). */
uint32_t gh_unit_entity_id(const unsigned char *unit)
{
    (void)unit;
    return 0;
}

/* Runs the game's shell-vs-damage-model test for an entity, a shell and a test input; 1 on a fault. */
int gh_shell_test(void *host, uint32_t eid, int32_t id, const gh_shell_test_t *test, int *out)
{
    (void)host;
    (void)eid;
    (void)id;
    (void)test;
    (void)out;
    return 0;
}

/* The unit's world position (x, y, z). */
const float * gh_unit_pos(const unsigned char *unit)
{
    (void)unit;
    return NULL;
}

/* X of the unit's forward vector. */
float gh_unit_fwd_x(const unsigned char *unit)
{
    (void)unit;
    return 0;
}

/* Z of the unit's forward vector. */
float gh_unit_fwd_z(const unsigned char *unit)
{
    (void)unit;
    return 0;
}

/* The unit's forward axis (its transform's first row). */
const float * gh_unit_frame(const unsigned char *unit)
{
    (void)unit;
    return NULL;
}

/* The unit's up axis. */
const float * gh_unit_up(const unsigned char *unit)
{
    (void)unit;
    return NULL;
}

/* Nonzero unless the unit is active (alive, in play). */
uint32_t gh_unit_not_active(const unsigned char *unit)
{
    (void)unit;
    return 0;
}

/* Nonzero when the unit is dead or removed. */
uint32_t gh_unit_not_present(const unsigned char *unit)
{
    (void)unit;
    return 0;
}

/* The player that drives the unit (NULL for scenery). */
unsigned char * gh_unit_player(const unsigned char *unit)
{
    (void)unit;
    return NULL;
}

/* The unit's army. */
unsigned char gh_unit_army(const unsigned char *unit)
{
    (void)unit;
    return 0;
}

/* The unit's name. */
unsigned char * gh_unit_name(const unsigned char *unit)
{
    (void)unit;
    return NULL;
}

/* The unit's vehicle description. */
unsigned char * gh_unit_description(const unsigned char *unit)
{
    (void)unit;
    return NULL;
}

/* The model path of a vehicle description. */
unsigned char * gh_description_model(const unsigned char *description)
{
    (void)description;
    return NULL;
}

/* The tank's native bot (its navigation state). */
unsigned char * gh_unit_ai(const unsigned char *unit)
{
    (void)unit;
    return NULL;
}

/* The unit's kind code. */
uint16_t gh_unit_kind(const unsigned char *unit)
{
    (void)unit;
    return 0;
}

/* Whether the unit is on fire. */
unsigned char gh_unit_burning(const unsigned char *unit)
{
    (void)unit;
    return 0;
}

/* The unit's vehicle physics / control block. */
unsigned char * gh_unit_controls(const unsigned char *unit)
{
    (void)unit;
    return NULL;
}

/* Sets the unit's line-of-sight re-trace interval (ms). */
void gh_unit_set_los_interval(unsigned char *unit, int32_t value)
{
    (void)unit;
    (void)value;
}

/* The unit's line-of-sight re-trace interval (ms). */
int32_t gh_unit_los_interval(const unsigned char *unit)
{
    (void)unit;
    return 0;
}

/* The unit's turret block. */
unsigned char * gh_unit_turrets(const unsigned char *unit)
{
    (void)unit;
    return NULL;
}

/* The unit's physics object. */
unsigned char * gh_unit_physics(const unsigned char *unit)
{
    (void)unit;
    return NULL;
}

/* The physics object's timestep (s). */
float gh_physics_timestep(const unsigned char *phys)
{
    (void)phys;
    return 0;
}

/* The unit a visibility detector asks about. */
unsigned char * gh_detector_target(const unsigned char *detector)
{
    (void)detector;
    return NULL;
}

/* The player's slot. */
unsigned char gh_player_slot(const unsigned char *player)
{
    (void)player;
    return 0;
}

/* Nonzero when the player is a native bot. */
int gh_player_bot(const unsigned char *player)
{
    (void)player;
    return 0;
}

/* Nonzero once the player has used a life. */
uint16_t gh_player_spawned(const unsigned char *player)
{
    (void)player;
    return 0;
}

/* The table of players by slot. */
const unsigned char *const * gh_players(void *host)
{
    (void)host;
    return NULL;
}

/* The player in a slot of that table. */
const unsigned char * gh_player_at(const unsigned char *const *players, int seat)
{
    (void)players;
    (void)seat;
    return NULL;
}

/* The offender's army of a hit record. */
int gh_hit_army(const unsigned char *record)
{
    (void)record;
    return 0;
}

/* The damage of a hit record. */
float gh_hit_damage(const unsigned char *record)
{
    (void)record;
    return 0;
}

/* The critical-parts field of a hit record. */
int gh_hit_parts(const unsigned char *record)
{
    (void)record;
    return 0;
}

/* The kind field of a hit record. */
int gh_hit_kind(const unsigned char *record)
{
    (void)record;
    return 0;
}

/* The hit count of a hit record. */
int gh_hit_count(const unsigned char *record)
{
    (void)record;
    return 0;
}

/* The bot's current enemy target (probe). */
unsigned char * gh_bot_target(const unsigned char *bot)
{
    (void)bot;
    return NULL;
}

/* When the bot was last hit (probe). */
float gh_bot_hit_time(const unsigned char *bot)
{
    (void)bot;
    return 0;
}

/* The game object. */
unsigned char * gh_game(void *host)
{
    (void)host;
    return NULL;
}

/* The game's landscape (for heights). */
unsigned char * gh_game_land(const unsigned char *game)
{
    (void)game;
    return NULL;
}

/* The game's list of units. */
unsigned char ** gh_game_units(const unsigned char *game)
{
    (void)game;
    return NULL;
}

/* Its length. */
uint32_t gh_game_unit_count(const unsigned char *game)
{
    (void)game;
    return 0;
}

/* One unit of that list. */
unsigned char * gh_unit_at(unsigned char **units, uint32_t i)
{
    (void)units;
    (void)i;
    return NULL;
}

/* The game's capture-zone owner. */
unsigned char * gh_game_zones(const unsigned char *game)
{
    (void)game;
    return NULL;
}

/* Its list of capture zones. */
unsigned char ** gh_zone_list(const unsigned char *zones)
{
    (void)zones;
    return NULL;
}

/* Its length. */
uint32_t gh_zone_count(const unsigned char *zones)
{
    (void)zones;
    return 0;
}

/* One zone of that list. */
unsigned char * gh_zone_at(unsigned char **zones, uint32_t i)
{
    (void)zones;
    (void)i;
    return NULL;
}

/* The zone's shape. */
unsigned char * gh_zone_shape(const unsigned char *zone)
{
    (void)zone;
    return NULL;
}

/* The zone's owner army (0 neutral). */
unsigned char gh_zone_owner(const unsigned char *zone)
{
    (void)zone;
    return 0;
}

/* The zone's flag word. */
uint16_t gh_zone_flags(const unsigned char *zone)
{
    (void)zone;
    return 0;
}

/* The shape's type. */
uint32_t gh_shape_type(const unsigned char *shape)
{
    (void)shape;
    return 0;
}

/* The shape's stored centre point (type dependent). */
const float * gh_shape_centre(const unsigned char *shape, uint32_t type)
{
    (void)shape;
    (void)type;
    return NULL;
}

/* The shape's extent vector. */
const float * gh_shape_extent(const unsigned char *shape)
{
    (void)shape;
    return NULL;
}

/* The engine's pairwise line-of-sight tables. */
void ** gh_game_sight_tables(const unsigned char *game)
{
    (void)game;
    return NULL;
}

/* The terrain table of those. */
void * gh_sight_table(void **tables)
{
    (void)tables;
    return NULL;
}

/* The height added to a capture destination. */
float gh_height_pad(void *host)
{
    (void)host;
    return 0;
}

/* The bot's move destination (x, y, z). */
const float * gh_ai_dest(const unsigned char *ai)
{
    (void)ai;
    return NULL;
}

/* Its x. */
float gh_ai_dest_x(const unsigned char *ai)
{
    (void)ai;
    return 0;
}

/* Its y. */
float gh_ai_dest_y(const unsigned char *ai)
{
    (void)ai;
    return 0;
}

/* Its z. */
float gh_ai_dest_z(const unsigned char *ai)
{
    (void)ai;
    return 0;
}

/* Sets the bot's move destination. */
void gh_ai_set_dest(unsigned char *ai, float x, float y, float z)
{
    (void)ai;
    (void)x;
    (void)y;
    (void)z;
}

/* Sets the bot's arrival countdown (s). */
void gh_ai_set_hold(unsigned char *ai, float value)
{
    (void)ai;
    (void)value;
}

/* Sets the bot's path recalculation countdown (s). */
void gh_ai_set_repath(unsigned char *ai, float value)
{
    (void)ai;
    (void)value;
}

/* Empties the bot's path. */
void gh_ai_clear_path(unsigned char *ai)
{
    (void)ai;
}

/* Sets the bot's capture-zone field. */
void gh_ai_set_zone(unsigned char *ai, int value)
{
    (void)ai;
    (void)value;
}

/* The bot's path points. */
const float * gh_ai_path(const unsigned char *ai)
{
    (void)ai;
    return NULL;
}

/* Their number. */
unsigned gh_ai_path_count(const unsigned char *ai)
{
    (void)ai;
    return 0;
}

/* The index of the next one. */
unsigned gh_ai_path_cursor(const unsigned char *ai)
{
    (void)ai;
    return 0;
}

/* Sets the move node's arrival distance. */
void gh_node_set_arrival(unsigned char *node, float value)
{
    (void)node;
    (void)value;
}

/* The move node's arrival distance. */
float gh_node_arrival(const unsigned char *node)
{
    (void)node;
    return 0;
}

/* The point count of a native path result. */
uint32_t gh_path_points(const unsigned char *path)
{
    (void)path;
    return 0;
}

/* The start of a native path request. */
const float * gh_path_from(const unsigned char *request)
{
    (void)request;
    return NULL;
}

/* The end of a native path request. */
const float * gh_path_to(const unsigned char *request)
{
    (void)request;
    return NULL;
}

/* The turret records. */
unsigned char * gh_turrets_records(const unsigned char *turrets)
{
    (void)turrets;
    return NULL;
}

/* Their number. */
uint32_t gh_turrets_count(const unsigned char *turrets)
{
    (void)turrets;
    return 0;
}

/* The turret's current yaw relative to the hull (deg). */
float gh_turret_yaw(const unsigned char *rec)
{
    (void)rec;
    return 0;
}

/* Sets the turret's current pitch. */
void gh_turret_set_pitch(unsigned char *rec, float value)
{
    (void)rec;
    (void)value;
}

/* The turret's current pitch (deg). */
float gh_turret_pitch(const unsigned char *rec)
{
    (void)rec;
    return 0;
}

/* Sets the turret's external pitch command. */
void gh_turret_set_ext_pitch(unsigned char *rec, float value)
{
    (void)rec;
    (void)value;
}

/* The turret's external pitch command. */
float gh_turret_ext_pitch(const unsigned char *rec)
{
    (void)rec;
    return 0;
}

/* Sets the gunner's commanded pitch. */
void gh_turret_set_cmd_pitch(unsigned char *rec, float value)
{
    (void)rec;
    (void)value;
}

/* The gunner's commanded pitch. */
float gh_turret_cmd_pitch(const unsigned char *rec)
{
    (void)rec;
    return 0;
}

/* The weapon of a turret record. */
unsigned char * gh_turret_gun(const unsigned char *rec)
{
    (void)rec;
    return NULL;
}

/* The gun's properties. */
unsigned char * gh_gun_props(const unsigned char *gun)
{
    (void)gun;
    return NULL;
}

/* The gun's stabiliser data. */
unsigned char * gh_gun_stab_object(const unsigned char *props)
{
    (void)props;
    return NULL;
}

/* Whether the stabiliser has the horizontal axis. */
unsigned char gh_stab_axis_h(const unsigned char *so)
{
    (void)so;
    return 0;
}

/* Whether the stabiliser has the vertical axis. */
unsigned char gh_stab_axis_v(const unsigned char *so)
{
    (void)so;
    return 0;
}

/* The stabiliser's horizontal omega multiplier. */
float gh_stab_omega_h(const unsigned char *so)
{
    (void)so;
    return 0;
}

/* The stabiliser's vertical omega multiplier. */
float gh_stab_omega_v(const unsigned char *so)
{
    (void)so;
    return 0;
}

/* The stabiliser's horizontal speed limit (m/s). */
float gh_stab_limit_h(const unsigned char *so)
{
    (void)so;
    return 0;
}

/* The stabiliser's vertical speed limit (m/s). */
float gh_stab_limit_v(const unsigned char *so)
{
    (void)so;
    return 0;
}

/* The stabiliser's yaw drive speed. */
float gh_stab_speed_yaw(const unsigned char *so)
{
    (void)so;
    return 0;
}

/* The stabiliser's pitch drive speed. */
float gh_stab_speed_pitch(const unsigned char *so)
{
    (void)so;
    return 0;
}

/* Nonzero when the collision header has node names. */
uint32_t gh_coll_has_names(const unsigned char *hdr)
{
    (void)hdr;
    return 0;
}

/* The number of collision nodes. */
uint32_t gh_coll_count(const unsigned char *hdr)
{
    (void)hdr;
    return 0;
}

/* The collision nodes. */
const unsigned char * gh_coll_nodes(const unsigned char *hdr)
{
    (void)hdr;
    return NULL;
}

/* The node name table. */
const char * gh_coll_names(const unsigned char *hdr)
{
    (void)hdr;
    return NULL;
}

/* The name of one collision node. */
const char * gh_coll_node_name(const char *names, const unsigned char *nodes, uint32_t i)
{
    (void)names;
    (void)nodes;
    (void)i;
    return NULL;
}

/* Writes the throttle input byte. */
void gh_drive_set_throttle(unsigned char *phys, unsigned char value)
{
    (void)phys;
    (void)value;
}

/* Writes the steer input byte. */
void gh_drive_set_steer(unsigned char *phys, unsigned char value)
{
    (void)phys;
    (void)value;
}

/* Clears both track brakes. */
void gh_drive_release_brakes(unsigned char *controls)
{
    (void)controls;
}

/* The throttle input byte. */
unsigned char gh_drive_throttle(const unsigned char *controls)
{
    (void)controls;
    return 0;
}

/* The steer input byte. */
unsigned char gh_drive_steer(const unsigned char *controls)
{
    (void)controls;
    return 0;
}

/* The left brake byte. */
unsigned char gh_drive_brake_left(const unsigned char *controls)
{
    (void)controls;
    return 0;
}

/* The right brake byte. */
unsigned char gh_drive_brake_right(const unsigned char *controls)
{
    (void)controls;
    return 0;
}

/* Writes the four control bytes. */
void gh_drive_write(unsigned char *controls, const void *src, size_t n)
{
    (void)controls;
    (void)src;
    (void)n;
}

/* Copies the control bytes. */
void gh_drive_read(void *out, const unsigned char *phys, size_t n)
{
    (void)out;
    (void)phys;
    (void)n;
}

/* Copies the control bytes the last physics step used. */
void gh_vphys_applied(void *out, const unsigned char *phys, size_t n)
{
    (void)out;
    (void)phys;
    (void)n;
}

/* The physics tick number. */
uint32_t gh_vphys_tick(const unsigned char *phys)
{
    (void)phys;
    return 0;
}

/* The gear in use. */
uint8_t gh_vphys_gear(const unsigned char *phys)
{
    (void)phys;
    return 0;
}

/* The gear asked for. */
uint8_t gh_vphys_gear_request(const unsigned char *phys)
{
    (void)phys;
    return 0;
}

/* The vehicle's mass (kg). */
float gh_vphys_mass(const unsigned char *phys)
{
    (void)phys;
    return 0;
}

/* Its maximum brake force (N). */
float gh_vphys_max_brake(const unsigned char *phys)
{
    (void)phys;
    return 0;
}

/* Its drive gear radius (m). */
float gh_vphys_wheel_radius(const unsigned char *phys)
{
    (void)phys;
    return 0;
}

/* The index of its neutral gear. */
int gh_vphys_neutral_gear(const unsigned char *phys)
{
    (void)phys;
    return 0;
}

/* The highest speed at which it engages a reverse gear (m/s). */
float gh_vphys_rear_engage_speed(const unsigned char *phys)
{
    (void)phys;
    return 0;
}

/* Its minimal gear change time (s). */
float gh_vphys_min_shift_time(const unsigned char *phys)
{
    (void)phys;
    return 0;
}

/* Whether it is tracked. */
uint8_t gh_vphys_tracked(const unsigned char *phys)
{
    (void)phys;
    return 0;
}

/* Its steer type. */
int gh_vphys_steer_type(const unsigned char *phys)
{
    (void)phys;
    return 0;
}

/* Its physics timestep (s). */
float gh_vphys_timestep(const unsigned char *phys)
{
    (void)phys;
    return 0;
}

/* Its velocity (double x, y, z). */
const double * gh_vphys_velocity(const unsigned char *phys)
{
    (void)phys;
    return NULL;
}

/* Its acceleration (double x, y, z). */
const double * gh_vphys_accel(const unsigned char *phys)
{
    (void)phys;
    return NULL;
}

/* Its angular velocity (double x, y, z). */
const double * gh_vphys_omega(const unsigned char *phys)
{
    (void)phys;
    return NULL;
}

/* Whether its gearbox is automatic. */
unsigned char gh_vphys_auto_gear(const unsigned char *phys)
{
    (void)phys;
    return 0;
}

/* A branch flag of the input layer (probe only). */
unsigned char gh_vphys_input_flag_a(const unsigned char *phys)
{
    (void)phys;
    return 0;
}

/* A second branch flag of the input layer (probe only). */
unsigned char gh_vphys_input_flag_b(const unsigned char *phys)
{
    (void)phys;
    return 0;
}

/* A third branch flag of the input layer (probe only). */
unsigned char gh_vphys_input_flag_c(const unsigned char *phys)
{
    (void)phys;
    return 0;
}

/* Sets the turret's current yaw. */
void gh_turret_set_yaw(unsigned char *rec, float value)
{
    (void)rec;
    (void)value;
}

/* Sets the turret's external yaw command. */
void gh_turret_set_ext_yaw(unsigned char *rec, float value)
{
    (void)rec;
    (void)value;
}

/* The turret's external yaw command. */
float gh_turret_ext_yaw(const unsigned char *rec)
{
    (void)rec;
    return 0;
}

/* Sets the gunner's commanded yaw. */
void gh_turret_set_cmd_yaw(unsigned char *rec, float value)
{
    (void)rec;
    (void)value;
}

/* The gunner's commanded yaw. */
float gh_turret_cmd_yaw(const unsigned char *rec)
{
    (void)rec;
    return 0;
}

/* Whether the engine stabilises the turret horizontally now. */
unsigned char gh_turret_engine_stab_h(const unsigned char *rec)
{
    (void)rec;
    return 0;
}

/* Whether the engine stabilises the turret vertically now. */
unsigned char gh_turret_engine_stab_v(const unsigned char *rec)
{
    (void)rec;
    return 0;
}

/* Whether the turret drive works (the stabiliser needs it). */
unsigned char gh_turret_drive_intact(const unsigned char *rec)
{
    (void)rec;
    return 0;
}

/* A turret drive state flag (stab_diag only). */
unsigned char gh_turret_state_flag_a(const unsigned char *rec)
{
    (void)rec;
    return 0;
}

/* Whether the vertical drive is blocked (no vertical stabiliser then). */
unsigned char gh_turret_vertical_blocked(const unsigned char *rec)
{
    (void)rec;
    return 0;
}

/* The unit's simulation mode byte (stab_diag only). */
unsigned char gh_unit_sim_mode(const unsigned char *unit)
{
    (void)unit;
    return 0;
}

/* Nonzero when the unit is exempt from the stabiliser's native-bot test. */
uint32_t gh_unit_stab_exempt(const unsigned char *unit)
{
    (void)unit;
    return 0;
}

/* The player's flag byte (stab_diag only). */
unsigned char gh_player_flags(const unsigned char *player)
{
    (void)player;
    return 0;
}

/* Sets the controls' "stabiliser allowed" byte. */
void gh_controls_set_stab_allowed(unsigned char *controls, unsigned char value)
{
    (void)controls;
    (void)value;
}

/* The controls' "stabiliser allowed" byte. */
unsigned char gh_controls_stab_allowed(const unsigned char *controls)
{
    (void)controls;
    return 0;
}

/* Whether the stabiliser has the vertical free mode. */
unsigned char gh_stab_free_mode(const unsigned char *so)
{
    (void)so;
    return 0;
}

/* Whether the stabiliser is forced on. */
unsigned char gh_stab_forced(const unsigned char *so)
{
    (void)so;
    return 0;
}

/* A first state byte of that host feature (log only). */
unsigned char gh_stab_patch_mask_a(void *host)
{
    (void)host;
    return 0;
}

/* A second state byte of that host feature (log only). */
unsigned char gh_stab_patch_mask_b(void *host)
{
    (void)host;
    return 0;
}

/* The unit's damage model. */
unsigned char * gh_unit_damage_model(const unsigned char *unit)
{
    (void)unit;
    return NULL;
}

/* The collision object of a damage model. */
unsigned char * gh_dm_collision(const unsigned char *dm)
{
    (void)dm;
    return NULL;
}

/* The node header of a collision object. */
unsigned char * gh_collision_header(const unsigned char *coll)
{
    (void)coll;
    return NULL;
}

/* The model-space box of a collision node (current transform). */
void gh_node_box(void *host, unsigned char *coll, float *out, int n)
{
    (void)host;
    (void)coll;
    (void)out;
    (void)n;
}

/* The number of bullet sets of the gun properties. */
uint32_t gh_props_set_count(const unsigned char *props)
{
    (void)props;
    return 0;
}

/* The gun's current bullet set. */
unsigned char gh_gun_current_set(const unsigned char *gun)
{
    (void)gun;
    return 0;
}

/* Nonzero when the weapon is not a gun. */
int gh_weapon_not_gun(void *host, const unsigned char *gun)
{
    (void)host;
    (void)gun;
    return 0;
}

/* One bullet set of the gun properties. */
unsigned char * gh_props_set(unsigned char *props, uint32_t cur)
{
    (void)props;
    (void)cur;
    return NULL;
}

/* The belt of a bullet set (its rounds). */
unsigned char ** gh_set_belt(unsigned char *set, uint32_t count)
{
    (void)set;
    (void)count;
    return NULL;
}

/* The number of rounds in a set's belt. */
uint32_t gh_set_rounds(const unsigned char *set)
{
    (void)set;
    return 0;
}

/* The shots fired from a set. */
uint16_t gh_set_shots(const unsigned char *set)
{
    (void)set;
    return 0;
}

/* One round (bullet) of a belt. */
unsigned char * gh_belt_round(unsigned char **belt, uint32_t k)
{
    (void)belt;
    (void)k;
    return NULL;
}

/* The shell id of a bullet. */
int32_t gh_bullet_shell_id(const unsigned char *bullet)
{
    (void)bullet;
    return 0;
}

/* The base shell id of a bullet (used when the shell id is -1). */
int32_t gh_bullet_base_id(const unsigned char *bullet)
{
    (void)bullet;
    return 0;
}

/* The muzzle speed of a bullet. */
float gh_bullet_speed(const unsigned char *bullet)
{
    (void)bullet;
    return 0;
}

/* The number of rows of a shell's penetration table. */
uint32_t gh_shell_pen_rows(const unsigned char *props)
{
    (void)props;
    return 0;
}

/* A shell's penetration-by-distance table. */
const float * gh_shell_pen_table(const unsigned char *props)
{
    (void)props;
    return NULL;
}

/* The unit of an entity id (generation-checked). */
unsigned char * gh_entity_unit(void *host, uint32_t eid)
{
    (void)host;
    (void)eid;
    return NULL;
}

/* The unit's current bot aim scatter (world offset). */
const float * gh_unit_aim_scatter(const unsigned char *unit)
{
    (void)unit;
    return NULL;
}

/* The unit's whole-vehicle bounding box (model space). */
const float * gh_unit_bbox(const unsigned char *unit)
{
    (void)unit;
    return NULL;
}

/* Casts the game's collision ray, unguarded (the caller guards it). */
unsigned char gh_ray_cast(void *host, const float *p, const float *dir, float *t, int flags, int *kind)
{
    (void)host;
    (void)p;
    (void)dir;
    (void)t;
    (void)flags;
    (void)kind;
    return 0;
}

/* The engine's bot scatter scale against bots. */
float gh_bot_scatter_vs_bot(void *host)
{
    (void)host;
    return 0;
}

/* The engine's bot scatter scale against humans. */
float gh_bot_scatter_vs_human(void *host)
{
    (void)host;
    return 0;
}

/* The game's damage-model settings. */
unsigned char * gh_dm_settings(void *host)
{
    (void)host;
    return NULL;
}

/* Damage-model setting: armour effective scale (log). */
float gh_dm_max_armor_scale(const unsigned char *dms)
{
    (void)dms;
    return 0;
}

/* Damage-model setting: armour breach factor (log). */
float gh_dm_armor_breach_k(const unsigned char *dms)
{
    (void)dms;
    return 0;
}

/* Damage-model setting: slope angle multiplier (log). */
float gh_dm_slope_angle_mult(const unsigned char *dms)
{
    (void)dms;
    return 0;
}

/* Damage-model setting: ricochet test threshold, low (log). */
float gh_dm_ricochet_min(const unsigned char *dms)
{
    (void)dms;
    return 0;
}

/* Damage-model setting: ricochet test threshold, high (log). */
float gh_dm_ricochet_max(const unsigned char *dms)
{
    (void)dms;
    return 0;
}

/* Sets the gun's reload end (game clock). */
void gh_gun_set_reload_end(unsigned char *gun, float value)
{
    (void)gun;
    (void)value;
}

/* The gun's reload end (game clock). */
float gh_gun_reload_end(const unsigned char *gun)
{
    (void)gun;
    return 0;
}

/* A gun state flag: the gun fires only while it is 0. */
unsigned char gh_gun_state_flag_a(const unsigned char *gun)
{
    (void)gun;
    return 0;
}

/* A gun state flag: the gun fires only while it is set. */
unsigned char gh_gun_state_flag_b(const unsigned char *gun)
{
    (void)gun;
    return 0;
}

/* A gun timer: the gun fires only while it is < 0. */
float gh_gun_timer_a(const unsigned char *gun)
{
    (void)gun;
    return 0;
}

/* A gun timer: the gun fires only while it is < 0. */
float gh_gun_timer_b(const unsigned char *gun)
{
    (void)gun;
    return 0;
}

/* The unit a gun belongs to. */
unsigned char * gh_gun_unit(const unsigned char *gun)
{
    (void)gun;
    return NULL;
}

/* The gun's world position. */
const float * gh_gun_position(const unsigned char *gun)
{
    (void)gun;
    return NULL;
}

/* Whether the unit's kind is a tank (both tank kind codes). */
int gh_unit_is_tank(const unsigned char *unit)
{
    (void)unit;
    return 0;
}

/* Whether the unit's kind is the basic tank code (probe). */
int gh_unit_is_basic_tank(const unsigned char *unit)
{
    (void)unit;
    return 0;
}

/* Whether a zone shape type stores a corner plus extent (centre = corner + extent / 2). */
int gh_zone_shape_is_area(uint32_t type)
{
    (void)type;
    return 0;
}

/* Whether a player uid is an AI uid of a slot 0..63. */
int gh_uid_ai_slot_valid(int32_t uid)
{
    (void)uid;
    return 0;
}

/* The slot an AI uid encodes. */
int gh_ai_slot_of_uid(int32_t uid)
{
    (void)uid;
    return 0;
}

/* Whether a player uid is a human's. */
int gh_uid_is_human(int32_t uid)
{
    (void)uid;
    return 0;
}

/* Whether a player uid is an AI uid. */
int gh_uid_is_ai(int32_t uid)
{
    (void)uid;
    return 0;
}

/* Whether a bot destination height means "no destination". */
int gh_ai_dest_height_none(float y)
{
    (void)y;
    return 0;
}

/* Whether a bot destination height is a real one. */
int gh_ai_dest_height_valid(float y)
{
    (void)y;
    return 0;
}

#ifdef GROUND_HOST_STUB_MAIN
/* The link test: the library is linked whole (/WHOLEARCHIVE), so every symbol it needs must resolve here. */
int main(void)
{
    return 0;
}
#endif
