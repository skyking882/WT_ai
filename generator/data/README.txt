data/: the inputs
==================

This folder ships empty. Below is every file the code reads, as the reading code shows it: who reads it, whether it
is required, its format, and where it comes from. Locations are defined in paths.py. "out/<m>/" is the output folder
of mission <m>; run_map.py copies the four base layers from data/maps/<m>/ into out/<m>/ (run_map.stage_base) and the
generator reads them from there. "<game>" is the installed game's folder (environment variable WT_GAME_DIR).

Summary
  file                                    used by            required?                     produced by this tree?
  common/ground_battle_areas.json         generator + prep   yes                           yes: prep/build_battle_areas.py
                                                                                             (from mission files unpacked
                                                                                             by an external BLK unpacker)
  common/spawn_protection.json            generator          no (no kill areas)            yes: prep/extract_spawn_protection.py
                                                                                             (needs vromfs / blk packages)
  common/rendinst_dmg.blk                 prep               yes for prep                  yes: prep/blk_to_text.py
                                                                                             (needs vromfs / blk packages)
  maps/<m>/map.npz, sight.npz,            generator          yes                           yes: prep/prep_base.py
    foliage.npz, map_report.json
  maps/<m>/ri_v3/<level>_ri_raster.npz    nothing            no                            yes: prep/prep_base.py (copy)
  runs/<run>/                             generator + prep   no (fallback speeds;          no (logs of played battles)
                                                             no deck layer)
  sweden/routes.json                      generator          no (sweden_dom only)          no (logs of played battles)
  closed/<m>.closed.json                  generator          no                            no (hand-written)
  obstacles/<m>.bin                       prep               no (see 9: what is lost)      NO: cannot be produced offline
  terrain/<m>.bin                         prep               yes for prep                  yes: prep/gen_terrain.py
  lnav/<level>.bin                        prep               no (no road raster)           yes: prep/lnav_extract.py


From an installed game to data/maps/<m>/ (Windows; run from the tree's folder; <game> = the game's folder)
  set WT_GAME_DIR=<game>
  1. python prep/build_battle_areas.py <missions dir>
       <missions dir> = gamedata/missions/cta/tanks of <game>/mis.vromfs.bin unpacked to JSON by a BLK unpacker such
       as wt_ext_cli (not part of this tree). Writes data/common/ground_battle_areas.json (all tank missions).
  2. python prep/blk_to_text.py
       writes data/common/rendinst_dmg.blk from <game>/aces.vromfs.bin (needs the vromfs / blk Python packages).
  3. python prep/extract_spawn_protection.py <m> [<m> ...]          (optional)
       writes data/common/spawn_protection.json from <game>/mis.vromfs.bin (needs the vromfs / blk Python packages).
  4. python prep/lnav_extract.py <game>/levels/<level>.bin data/lnav/<level>.bin     (optional, once per level)
  5. python prep/gen_terrain.py <m> [<m> ...] [--runs <folder> ...]
       writes data/terrain/<m>.bin; logged battles with ground_samples.csv (default under data/runs/) add the deck
       layer, without them there is none.
       If several missions share a level, run it for all of them: a mission without a water level (NaN) takes it
       from another mission's terrain file of the same level (item 10).
  6. python prep/prep_base.py <m> [<m> ...]
       writes data/maps/<m>/ (with data/obstacles/<m>.bin when it exists, else without the obstacle scan).
  Then: python run_map.py <m> --check, python run_map.py <m>.

Third-party tools you need for prep
  * the game CDK's codec DLL daKernel-dev.dll (<game>/WarThunderCDK/dagor_cdk/windows-x86_64/; comes with the game)
  * a BLK unpacker that writes mission files as JSON (for example wt_ext_cli), for step 1
  * the Python packages vromfs and blk (readers for .vromfs.bin archives and binary BLK), for steps 2 and 3; they use
    the construct package (prep/blk_archive.py adapts its call signatures when needed)
  * Python with numpy and Pillow (as for the generator)

What cannot be produced offline
  data/obstacles/<m>.bin, the 2 m obstacle scan (item 9). It is made by casting collision rays inside a running game;
  no script in this tree can make it. Prep works without it; what is lost then is listed in item 9.


1. common/ground_battle_areas.json  (paths.BATTLE_AREAS)
   Read by: run_map.py:inputs (first thing it does), run30.py:variants_report, prep/prep_base.py:prep and, through
     common30.py:battle_area / mission_level / zones / spawns, by field30.py:spawn_areas, lanes37.py:main,
     render30.py, common30.py:read_terrain, prep/build_map30.py:area_of, prep/build_ri_s30.py, prep/trench31.py;
     prep/scan_area.py (the scan rectangle without a scan) and prep/gen_terrain.py (level, crop).
   Required. Missing file: FileNotFoundError at the start. Mission not listed: run_map.py reports
     "<m> is not in data/common/ground_battle_areas.json" and stops.
   Format: one JSON object, key = mission name, value:
     "level"    string, the level name (selects runs/, lnav/<level>.bin and the game's levels/<level>.bin)
     "captures" object: capture name -> [x, z, r] (metres). The last two digits of the name give the zone letter
                (_01 = A ... _08 = H). A name may end in _arcade / _realistic / _hardcore; when several such sets
                are present the _realistic set is used (common30.CAPTURE_MODE); several sets without _realistic, or
                a letter twice in the chosen set, is an error.
     "spawns"   object: "t1" (army 1) and "t2" (army 2), each a list of [x, z] spawn points (exactly two numbers).
     "bbox"     [x0, z0, x1, z1]: the battle rectangle (base-layer area in prep; spawn-area naming; pictures).
     Also written by build_battle_areas.py, not read: "area" (name of the battle area used), "polygon", "size_m".
   Comes from: prep/build_battle_areas.py <missions dir> [<out json>]. Input: the tank mission files
     (gamedata/missions/cta/tanks/<location>/<mission>.blk inside <game>/mis.vromfs.bin) unpacked to JSON by an
     external BLK unpacker such as wt_ext_cli (not part of this tree). Per mission (a file whose
     mission_settings.mission.level is an avg_* level): the areas of the file and of the files it imports; the
     battle area = an area named *battle_area* ("hardcore" preferred over "arcade"), a Box (bbox of its 4 corners)
     or a Cylinder / Sphere (a 24-gon); captures = the circular *capture_area* areas of that mode; spawns = the
     centres of the areas t1_tankspawn* / t2_tankspawn*.

2. common/spawn_protection.json  (paths.SPAWN_PROTECTION, common30.PROTECT)
   Read by: common30.py:kill_areas (lanes37.py:main, pack31.py, tools/plans_only.py), run_map.py:inputs,
     run30.py:variants_report.
   Optional. Missing file or mission not listed: no kill areas (run_map.py --check notes "no spawn-protection (kill
     area) entry").
   Format: JSON object, key = mission name, value = object: army ("1" / "2") -> {"name": str, "shapes": [shape ...]},
     shape = {"name": str, "type": str (kept in reports; the inside test below is the same for every type),
     "tm": [[ax_x, ax_y, ax_z], [ay_x, ay_y, ay_z], [az_x, az_y, az_z], [px, py, pz]]} (4 x 3 rows: x, y, z axis,
     position), "file" (the mission file that defines it; not read). The area is a unit disc (radius 0.5 in shape
     units) scaled by tm; only the xz parts are used (tm[0][0], tm[0][2], tm[2][0], tm[2][2], tm[3][0], tm[3][2]): a
     point is inside when |inverse(2x2) . (x - px, z - pz)| <= 0.5. It is the army's own spawn protection; the other
     army may not stand there.
   Comes from: prep/extract_spawn_protection.py <m> [...] [--mis <archive>] [--out <file>] (optional step; needs the
     third-party vromfs / blk Python packages). It reads the mission file gamedata/missions/cta/tanks/<location>/<m>.blk
     from <game>/mis.vromfs.bin and the files it imports (3 levels) as BLK text, and takes the area (or area squad)
     t<N>_killarea_realistic, else t<N>_killarea, for army N (the mission template turns t1_killarea into the area
     army 2 may not enter and t2_killarea into the one army 1 may not enter).

3. common/rendinst_dmg.blk  (paths.DMG_BLK; the name is fixed in paths.py and carries no game version)
   Read by: prep/build_ri_obstacles.py:parse_dmg, called from prep/build_ri_s30.py (as --dmg), prep/build_map30.py:
     foliage_instances and collision_tops (common30.DMG_BLK); prep/build_ri_raster.py:main (--dmg).
   Required for prep (prep_base.py stops with "missing data/common/rendinst_dmg.blk"); the generator does not read it.
   Format: the game's config/rendinst_dmg.blk (inside aces.vromfs.bin) as BLK TEXT, UTF-8, strict dialect of the blk
     package. parse_dmg uses regular expressions: every top-level block "dmg{" ... "}" (block start at column 0) with
     a line name:t="<object>" and parameter lines "<key>:<type>=<value>"; and the top-level block "riExtra{" whose
     two-space-indented sub-blocks "  <object>{" ... "  }" carry parameter lines (riExtra values win). Keys used:
     immortal, destructionImpulse, bushBehaviour, treeBehaviour, hp, physRes. Object names are matched lower case.
   Comes from: prep/blk_to_text.py [--vromfs <archive>] [--path <inner path>] [--out <file>] (defaults
     <game>/aces.vromfs.bin, config/rendinst_dmg.blk, data/common/rendinst_dmg.blk; needs the third-party vromfs / blk
     Python packages). Use the table of the same game version as the level files prep reads.
     (prep/catalog_reader.py also contains readers for VROMFS archives (vromfs()) and binary BLK (Blks), not used for
     this.)

4. maps/<m>/  the base layers  (paths.MAPS, paths.BASE_FILES)
   Read by: run_map.py:inputs (all four must exist, else "missing data/maps/<m>/<file> (run prep/prep_base.py ...)"
     and the run stops), run_map.py:stage_base (copies them to out/<m>/), run_map.py:check (map_report.json),
     run_all.py (a mission is listed when map.npz exists), tools/compare_base.py. From out/<m>/: field30.py:Field,
     sight30.py:Sight, lanes37.py:main, run30.py:run_once / bridge_check / main.
   Required.
   Grid: X0, Z0 = the area's south-west corner (world x, z in metres); rows run along +z, columns along +x. H, W =
     rows, columns of the 0.25 m class grid; both are multiples of 8 (one 2 m scan cell = 8 x 8 class cells).
     The area is the battle rectangle cut to the scan rectangle and aligned outward to its 2 m cells
     (build_map30.area_of). Without a scan the scan rectangle is the one the scan would have (prep/scan_area.py),
     so the grid is the same.
   map.npz (np.load; arrays):
     cls    uint8 (H, W)       0.25 m classes: 0 free, 1 soft, 2 medium_knockable, 3 hard_ri, 4 hard_static_scan,
                               5 unknown_ri_scan, 6 unknown_edge_scan, 7 water, 8 steep, 9 hard_building_part,
                               10 drivable_object, 11 overhead (common30.CLASS_NAMES). Hard for driving: 3, 4, 5, 6, 7,
                               8, 9 (common30.HARD_CODES). Classes 4, 5, 6 and 11 exist only with an obstacle scan.
     tcost  float32 (H/2, W/2) 0.5 m terrain cost factor (1 + slope and water terms)
     X0, Z0 float scalars; cell = 0.25 (written, not read)
     The generator reads cls, tcost, X0, Z0.
   sight.npz:
     G       float32 (H/8, W/8) ground height at the 2 m cell centres
     objh    float32 (H/8, W/8) height of a hard object above the ground (0 = none)
     objsrc  uint8 (H/8, W/8)   where objh came from (1 own scan hit, 2 neighbour hit, 3 assumed, 4 collision top);
                                not read by the generator. Without a scan only 3 and 4 occur.
     road    uint8 (H/2, W/2)   1 = 0.5 m cell covered by the level's Lnav road mesh; (1, 1) zeros when there is none
     has_road bool              False: no road mesh, every cell counts as off road
     X0, Z0, cell = 2.0
     The generator reads G, objh, road, has_road; it asserts that G is exactly (H/8, W/8).
   foliage.npz:
     inst    float32 (n, 4)     one row per bush / tree instance in the area: x, z, kind (0 bush, 1 tree),
                                collision top height (m)
   map_report.json (JSON object). Keys read:
     scan_rules.bridge_decks  list, REQUIRED by lanes37.py:main (KeyError otherwise; [] without a scan); per deck:
                              end_a [x, z], end_b [x, z], x, z (centre), deck_length, lane_width, connected (bool),
                              joined (optional); run30.py:bridge_check (standalone run30.py only) also reads
                              deck_width, lane_classes_before_m2 and ends{a, b: {fail, closed_by_m2}}
     road         string; run_map.py --check warns when it contains "no Lnav"
     water_note, area   shown by run_map.py --check
     obstacle_scan  present only when prep ran without the scan: "no obstacle scan: static scene objects that are not
                  placed instances (bridges and some structures) are missing"; run_map.py --check lists it as unusual
     The whole report is copied into analysis.json by standalone run30.py.
   Comes from: prep/prep_base.py <m>, which runs prep/build_ri_s30.py <m> (object raster into out/<m>/ri_v3/) and
     prep/build_map30.py <m> (writes the four files into out/<m>/), then copies them to data/maps/<m>/. Inputs of
     that step: items 1, 3, 10 (required), 9, 11 (optional) and the installed game (see "What prep needs from
     outside").

5. maps/<m>/ri_v3/<level>_ri_raster.npz
   Read by: nothing. prep/prep_base.py copies it here; prep/build_map30.py:merged reads the copy in
     out/<m>/ri_v3/ that prep/build_ri_s30.py wrote in the same prep run.
   Format (np.savez_compressed by prep/build_ri_raster.py:main): grid uint8 (rows, cols) at 0.25 m over the whole
     scan rectangle (cols = nx * step / 0.25, rows = nz * step / 0.25), classes 0 free, 1 drivable object, 2 soft,
     3 knockable medium, 4 building part, 5 hard (prep/build_ri_s30.py:material_class); x0, z0 = the scan origin;
     cell = 0.25. build_map30 uses only grid.
   Comes from: prep/build_ri_s30.py (via prep/prep_base.py).

6. runs/<run>/  logged battles  (paths.RUNS, common30.RUNS_DIR)
   Read by: common30.py:runs_of_level (run_map.py:inputs, lanes37.py:main, tools/speeds.py, tools/plans_only.py),
     field30.py:log_speeds (lanes37.py:main). Standalone run30.py:main also uses common30.run_zones,
     run30.spawn_check and run30.tracks_check. prep/gen_terrain.py:find_runs (deck layer; default folder, --runs
     gives others).
   Optional. No folder or no run of the mission's level: speeds fall back to 9.0 m/s road, 7.0 m/s off road; the
     same fallback applies to road or off-road when fewer than 200 moving samples (> 2 m/s) are found. For
     gen_terrain.py: no deck layer.
   A run belongs to a level when its ground_plan.json contains the exact text "level": "<level>" within its first
     400 characters (generator); gen_terrain.py parses ground_plan.json and compares its "level". Folders named in
     the environment variable WT_RUNS_EXCLUDE (comma-separated) are skipped by the generator.
   Files (CSV with a header row, read with csv.DictReader; only these columns are used):
     ground_plan.json      only the level (see above)
     ground_movement.csv   x, z, measured_speed (m/s); rows that do not parse are skipped (bot samples)
     units.csv             t (s), id, x, z, is_bot ("0" = human), active ("1"); speed = chord of the same id over
                           3-7 s, kept when 2-25 m/s (human samples). run30.spawn_check / tracks_check use id, x, z,
                           active.
     zones.csv             i, x, z (standalone run30.py only)
     ground_samples.csv    t, seat, x, y, z (prep/gen_terrain.py only; files under 20000 bytes are not used; a run
                           folder found twice with the same name and size counts once)
     Samples are split into road / off road by sight.npz road; the medians are the speeds.
   Comes from: not produced by this tree (logs of played battles).

7. sweden/routes.json  (paths.SWEDEN_ROUTES)
   Read by: pack31.py:poses_for (called by lanes37.py:main) when the mission is sweden_dom and the file exists;
     run_map.py:inputs notes when it is missing.
   Optional, sweden_dom only. Missing: sweden_dom's poses are its spawn points from item 1 with heading NaN, as for
     every other mission; run_map.py writes "no data/sweden/routes.json: spawn poses are the mission's spawn points
     with unknown heading (as for every other mission)" into run_report.json (notes, inputs.poses_note) and check.json
     (unusual). Not read for any other mission.
   Format: {"poses": [{"id": str (at most 8 bytes in routes.bin), "side": "S" (army 1) or anything else (army 2),
     "x": float, "z": float, "heading": float (degrees, written to routes.bin as heading_deg), "area": str
     (optional; kept as s25_area)}, ...]}.
   Comes from: not produced by this tree (spawn positions and headings logged in battles).

8. closed/<m>.closed.json  (paths.CLOSED)
   Read by: gen/closed35.py:load (called by lanes37.py:main before plan.json is written; also python gen/closed35.py
     <m> on an existing pack), run_map.py:inputs (notes whether it exists).
   Optional. Missing: nothing is closed.
   Format: {"closed": [{"x": float, "z": float, "r": float, "why": str (free text, only echoed in the report)}]}.
     A lane with any slot within r metres of (x, z) is removed from plan.json.
   Comes from: written by hand; nothing in this tree produces it.

9. obstacles/<m>.bin  the 2 m obstacle scan  (paths.OBSTACLES)
   Read by: common30.py:read_scan (prep/build_map30.py:main); prep/scan_area.py (header; build_ri_s30.py passes
     --scan to prep/build_ri_raster.py, which reads the header); prep/gen_terrain.py (header: crop). Standalone
     run30.py:variants_report checks only that it exists. prep/build_ri_obstacles.py takes --scan (header only).
   Optional. When present, prep uses it exactly as before. When absent (prep/scan_area.py):
     * the grid: the rectangle the scan would have: the battle rectangle + 150 m on every side at 2 m (origin and
       far corner written with one decimal and read as float32, cells = int((far - origin) / 2) in float32; this
       reproduces the header of every scan this tree was built with), so map area, raster and terrain crop are the
       same as with the scan; build_ri_raster.py / build_ri_obstacles.py get it as --box x0 z0 step nx nz;
     * no scan classes: hard_static_scan (4), unknown_ri_scan (5), unknown_edge_scan (6), overhead (11) do not occur;
     * water is judged at every cell (with a scan, cells under a scan hit are never water);
     * no bridge decks: scan_rules.bridge_decks = [] (the bridge rule finds decks only from scan hits over deep
       water), so no bridge lanes; only the terrain's deck layer (item 10) still keeps water off logged tracks;
     * object heights (sight.npz objh): the collision top of a hard rendinst where there is one, else the assumed
       2.5 m (common30 low_obj_h); with a scan the scan's hit heights come first and the collision top replaces
       them only where it is higher;
     * map_report.json gets "obstacle_scan": "no obstacle scan: static scene objects that are not placed instances
       (bridges and some structures) are missing".
     What is lost: everything that is solid in the game but not a placed rendinst instance (static scene geometry:
     bridges, some buildings and walls, other objects the scan hit but the object raster does not have). Bridges
     over deep water become water (closed), so places joined only by a bridge may be cut off.
   Format (little endian):
     @0   char[8]  "WTOBS001"
     @8   u32 nx, u32 nz             columns (x), rows (z)
     @16  f32 x0, f32 z0, f32 step   origin and cell size (2 m)
     @28  u16[nz][nx]                one word per cell, row = z. Bits as build_map30 uses them:
            0x8000 cell known; bits 8-10 hit kind ((S >> 8) & 7: 0 edge ray, 2 static, 4 rendinst; 0x0700 = any hit);
            bits 0-5 hit height code (S & 63, x 0.5 m above the ground); 0x0040 / 0x0080 edge ray blocked towards
            +x / +z; solid = known and (S & 0x02C0, or a hit with height code 1-7).
   Comes from: NOT produced by this tree and cannot be produced offline: it is made by casting collision rays inside
     a running game (code not part of this tree).

10. terrain/<m>.bin  terrain heights  (paths.TERRAIN)
   Read by: common30.py:_terrain / read_terrain (prep/build_map30.py:main, prep/trench31.py:trench_mask);
     prep/build_ri_raster.py:main (--terrain, given by prep/build_ri_s30.py).
   Required for prep/build_map30.py and prep/trench31.py (prep_base.py stops with "missing data/terrain/<m>.bin").
     For build_ri_raster.py it is optional: without it the hull band is measured from each instance's base.
   Format (little endian):
     @0   char[8]  "WTTER001"
     @8   u32 nx, u32 nz
     @16  f32 x0, z0, step, offset, scale, water_level   (water_level may be NaN)
     @40  u32 flags                                       bit 0: a deck layer follows; @44 u32 reserved (0)
     @48  u16[nz][nx] heights; height = value * scale + offset
     then, when flags & 1: u8[nz][nx] deck flags (bit 0 = bridge deck)
     A NaN water level is taken from another mission's terrain file of the same level whose heights agree within
     0.05 m; if none, rivers are not closed (noted in map_report.json water_note).
   Comes from: prep/gen_terrain.py <m> [...] [--runs <folder> ...] [--no-runs]. Heights: the level's HM2 heightfield
     (levels/<level>.bin of the game, decoded by prep/extract_terrain.py with the CDK's codec DLL), unchanged,
     cropped to the scan rectangle (or the rectangle the scan would have) united with the battle rectangle, + 300 m.
     Water level: the table WATER (per level) and MISSION_WATER (per mission, wins) in gen_terrain.py; the values
     were found by observation. Deck layer: from logged battles with ground_samples.csv of the level (item 6): a
     sample more than 0.8 m above the HM2 ground marks the track through it with a 5 m band. Without such battles
     (or with --no-runs) there is no deck layer: flags bit 0 clear, and gen_terrain.py says so.

11. lnav/<level>.bin  the level's road navigation mesh  (paths.LNAV, common30.LNAV_DIR)
   Read by: prep/build_map30.py:road_raster, parsed with prep/lnav_extract.py:parse.
   Optional. Missing or empty: no road raster (sight.npz has_road False, map_report.json road "no Lnav file for the
     level"); the generator then uses one speed everywhere.
   Format: the decompressed Lnav block of the level (Detour single-tile navmesh). parse() reads: @4 i32 version,
     @24 i32 polyCount, i32 vertCount, @100 vertCount x 3 f32 (x, y, z), then polyCount records of 32 bytes
     (@+4 6 x u16 vertex indices, @+16 6 x u16 neighbours, @+30 u8 vertex count). The polygons are drawn into the
     0.5 m road grid.
   Comes from: prep/lnav_extract.py, run by hand (prep_base.py does not run it):
       python prep/lnav_extract.py <game>/levels/<level>.bin data/lnav/<level>.bin
     It finds the "Lnav" block in the level file (magic "DBLD3x64", blocks of u32 size | flags + 4-char tag), checks
     the zstd frame and decompresses it with the game CDK's zstd_decompress (needs WT_GAME_DIR, see below).


What prep needs from outside
  The installed game, folder given ONLY by the environment variable WT_GAME_DIR (paths.GAME / paths.game_dir();
  prep stops with "the environment variable WT_GAME_DIR is not set ..." when it is missing); the CDK DLL folder is
  derived from it (paths.cdk_dir()). prep_base.py and gen_terrain.py refuse to run on anything but Windows.
  Files of the game it opens (read only):
    <game>/levels/<level>.bin        block "RIGz" (pregenerated rendinst placements), decoded by
                                     prep/build_ri_obstacles.py:rigz_decode. Opened by build_ri_obstacles.py:main,
                                     build_ri_raster.py:main (via build_ri_s30.py), build_map30.py:foliage_instances
                                     and collision_tops, trench31.py:trench_mask; and by lnav_extract.py when it is run
                                     by hand. (Also gen/run30.py:instance_index, used only when run30.py runs on its
                                     own, never by run_map.py.)
                                     block "\0HM2" (heightfield), decoded by prep/extract_terrain.py for
                                     prep/gen_terrain.py.
    <game>/content/base/res/**/*.grp every GRP2 resource pack, scanned by build_ri_obstacles.py:Collision for
                                     CollisionResource entries "<name>_collision" (class 0xACE50000, stream label
                                     0xACE50003), decoded by build_ri_obstacles.py and prep/colres.py:ColRes.
    <game>/aces.vromfs.bin           prep/blk_to_text.py (config/rendinst_dmg.blk), with the vromfs / blk packages
    <game>/mis.vromfs.bin            prep/extract_spawn_protection.py (mission files), with the vromfs / blk packages;
                                     its tank mission folder unpacked to JSON by an external BLK unpacker is the input
                                     of prep/build_battle_areas.py
  External library:
    <game>/WarThunderCDK/dagor_cdk/windows-x86_64/daKernel-dev.dll (the game CDK's codec DLL), loaded with
    ctypes.CDLL after os.add_dll_directory(<that folder>):
    * prep/catalog_reader.py:Codecs (used by build_ri_obstacles.py, build_ri_raster.py, build_map30.py, trench31.py,
      extract_terrain.py, gen_terrain.py) resolves at load time
        ?zstd_decompress@@YA_KPEAX_KPEBX1@Z                    zstd_decompress(dst, dst_size, src, src_size)
        ?oodle_decompress@@YA_KPEAX_KPEBX1@Z                   oodle_decompress(dst, dst_size, src, src_size)
        ?zstd_create_ddict@@YAPEAUZSTD_DDict_s@@V?$Span@$$CBD@dag@@_N@Z
        ?zstd_create_dctx@@YAPEAUZSTD_DCtx_s@@_N@Z             (called once at load: zstd_create_dctx(false))
        ?zstd_decompress_with_dict@@YA_KPEAUZSTD_DCtx_s@@PEAX_KPEBX2PEBUZSTD_DDict_s@@@Z
      In use: oodle_decompress (RIGz block A, HM2 chunks with codec flag 2), zstd_decompress (RIGz cell data,
      compressed collision resources, HM2 chunks with codec flag 1). The dictionary functions serve only
      catalog_reader.Blks, which nothing in this tree calls.
    * prep/lnav_extract.py:zstd loads the same DLL and calls only ?zstd_decompress@@YA_KPEAX_KPEBX1@Z.
  Third-party Python packages (steps 2 and 3 only): vromfs and blk, imported as installed packages
    (prep/blk_archive.py). Without them data/common/rendinst_dmg.blk and spawn_protection.json must come from
    elsewhere.
  Python: numpy and Pillow (as for the generator).
