# 新目标聚类晋升事件（target_estimator_node -> 下游节点）
#
# 发布时机：tentative cluster 首次晋升为正式 cluster 时发布一次。
# 坐标系：NWU（North-West-Up），原点与 target_estimator 的本地 map 原点一致。

uint64   cluster_id        # target_estimator 内部递增 ID
uint32   label_hash        # YoloDetection.label 的哈希
string   label             # 原始目标类别，例如 "person" / "car"

float32  confidence        # 晋升时记录的最佳置信度
uint64   first_seen_us     # 首次观测时间戳（微秒）
float64  nwu_x             # 北向坐标，单位：米
float64  nwu_y             # 西向坐标，单位：米
float64  nwu_alt           # 上向坐标，单位：米；地面目标通常为 0

# 由锁定的 PX4 home 和上述 NWU 坐标换算得到的 WGS84 位置。
# gps_altitude 使用 MSL 高度；home 尚未锁定时 gps_valid=false，其余字段为 0。
bool     gps_valid
float64  gps_latitude
float64  gps_longitude
float64  gps_altitude
