# One physical object in the semantic map. Positions are in the `map` frame (ENU).
string id                       # e.g. "chair_01"
string class_name               # detector class, e.g. "chair"
geometry_msgs/Point position    # map frame [m]
geometry_msgs/Vector3 size      # bounding box extents [m] (0 = unknown)
float32 confidence              # fused detection confidence 0..1
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
uint32 observation_count
string[] relations              # "near:table_01", "left_of:bed_01" ... (Phase 6+)
