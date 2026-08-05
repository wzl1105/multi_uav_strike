# //===================制导修改====================
# 当前 mission/primary_target 对应目标的最新图像观测。
# 图像坐标为 [0,1]；world_ray_nwu 是 bbox 中心经相机姿态转换后的单位视线向量。
std_msgs/Header header
uint32 frame_seq
uint64 stamp_us
uint64 cluster_id
string label
float32 confidence
float32 center_x
float32 center_y
float32 width
float32 height
geometry_msgs/Vector3 world_ray_nwu
bool visible
# //===================制导修改====================
