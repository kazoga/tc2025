# route_msgs/msg/ObstacleAvoidanceHint.msg
# obstacle_monitor が出力する障害物検知ヒント

std_msgs/Header header

bool front_blocked
float32 front_clearance_m
float32 left_offset_m
float32 right_offset_m
