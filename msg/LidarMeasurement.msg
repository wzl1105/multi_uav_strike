# 前向激光雷达单次测量。距离和命中类型在同一消息内，避免跨话题不同步。
uint8 HIT_NONE=0
uint8 HIT_GROUND=1
uint8 HIT_BUILDING=2

std_msgs/Header header
float32 range
float32 min_range
float32 max_range
uint8 hit_type
geometry_msgs/Point hit_point
