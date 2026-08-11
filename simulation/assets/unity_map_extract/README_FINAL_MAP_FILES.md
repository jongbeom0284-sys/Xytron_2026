Final Unity map extraction files
================================

Use only these files. They are generated from per-level Unity scene data with parent transforms accumulated into world coordinates.

- `level_world_map_overview.png`: top-down overview image.
- `level_world_cones.csv`: cone world coordinates.
- `level_world_traffic_lights.csv`: traffic light and signal-related world coordinates.
- `level_world_police.csv`: police car / police obstacle world coordinates.
- `level_world_waypoints.csv`: course waypoint world coordinates.
- `level_world_triggers.csv`: start, shortcut, slow zone, traffic sign check trigger coordinates.
- `level_world_lane_lines.csv`: extracted lane/line object coordinates when present.

Older files named `map_*`, `world_*`, `scene_*`, and exported `meshes/textures/sprites` were intermediate or incorrectly transformed outputs and were removed.
