# Per-frame mesh-triangle hits from the FLS raycast, for CIR post-processing.
# Each (geometry_ids[i], primitive_ids[i]) is one triangle hit at least once in
# this frame -- deduplicated per FRAME, not per ray. Misses (0xFFFFFFFF) are
# filtered out before publishing.
#
# geometry_id convention (load order in sensor_publisher): 0 = seabed, i+1 = box_i.
# Map primitive_id -> face OFFLINE (box: face = primitive_id // 2; seabed: identity).
std_msgs/Header header
int32[] geometry_ids
int32[] primitive_ids
