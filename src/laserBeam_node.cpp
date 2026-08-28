// ============================================================================
// laserBeam_node.cpp
//   单线激光雷达驱动 + LidarMeasurement 适配节点
//
//   硬件：SPD1200N2 单线激光雷达 (RS-232/RS-485 接口)
//   串口协议：0x55 0xAA 0x89 0xFF 0xFF 0xFF 0xFF 0x85 触发连续测距
//             回复帧: 55 AA 89 STA D1 D2 D3 D4
//             校验和: sum(frame[0..6]) == frame[7]
//             STA = 0x01 有效,距离 = (D3<<8 | D4) / 10.0 (单位:米)
//             STA = 0x89 停止指令: 55 AA 8E FF FF FF FF 8A
//
//   跟 gimbal_simulator_node 的区别:
//     - 仿真: gimbal_simulator_node 在 lidarTimerCallback 里 ray-cast 仿真建筑
//       + 地面,发布 lidar/measurement (含 hit_type=HIT_GROUND/HIT_BUILDING)
//     - 真机: 本节点从串口读 SPD1200N2 距离,按同样 hit_type 分类规则发布
//       lidar/measurement,waypoint_executor 不需要切换上游,只换 launch 节点
//
//   hit_type 分类 (参考 gimbal_simulator_node::lidarTimerCallback L515-528):
//     比较 measured_range vs ground_range = (ground_z - uav_z) / ray_dir.z,
//     差值在 tolerance (max(abs_tol, rel_tol * ground_range)) 内 → HIT_GROUND,
//     否则 → HIT_BUILDING (视为障碍)。
//
//   hit_point / frame_id: 跟 gimbal_simulator 一致(参考其 L530-556):
//     hit_point = uav_pose + ray_dir * range
//     frame_id = yaml "frame_id" (默认 "map",NWU 系)
//
// 坐标系:
//   - 雷达射线方向 = q_body_world * UnitX() (机体系 +X)
//   - ray_dir.z < 0 表示朝下 (NWU, z 朝上)
//   - UAV 高度 = uav_pose.z (NWU, m, 地面为 0)
//
// 使用 (launch 片段):
//   <node name="laserBeam_node" pkg="multi_uav_strike" type="laserBeam_node">
//     <param name="port" value="/dev/ttyACM0"/>
//     <param name="baudrate" value="115200"/>
//     <param name="min_range" value="0.1"/>
//     <param name="max_range" value="60.0"/>
//     <param name="ground_range_abs_tolerance" value="0.5"/>
//     <param name="ground_range_rel_tolerance" value="0.05"/>
//     <param name="frame_id" value="uav0/map"/>
//     <param name="uav_pose_topic" value="mavros/local_position/pose"/>
//     <param name="publish_rate" value="20.0"/>
//   </node>
// ============================================================================

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <serial/serial.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <limits>
#include <cstring>
#include <vector>

#include "multi_uav_strike/LidarMeasurement.h"

class LaserBeamNode {
public:
    LaserBeamNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
        : nh_(nh), nh_private_(nh_private),
          // 串口超时短,避免单字节卡 read 线程
          serial_("", 115200, serial::Timeout::simpleTimeout(100)) {

        // === 参数 ===
        std::string port = "/dev/ttyS6";
        int baudrate = 115200;
        nh_private_.param<std::string>("port", port, "/dev/ttyS6");
        nh_private_.param<int>("baudrate", baudrate, 115200);
        nh_private_.param<double>("min_range", min_range_, 0.1);
        nh_private_.param<double>("max_range", max_range_, 300.0);
        nh_private_.param<double>("ground_range_abs_tolerance", ground_range_abs_tol_, 0.5);
        nh_private_.param<double>("ground_range_rel_tolerance", ground_range_rel_tol_, 0.05);
        nh_private_.param<double>("ground_z", ground_z_, 0.0);
        nh_private_.param<std::string>("frame_id", frame_id_, std::string("map"));
        std::string uav_pose_topic = "mavros/local_position/pose";
        nh_private_.param<std::string>("uav_pose_topic", uav_pose_topic,
                                       "mavros/local_position/pose");
        double publish_rate = 20.0;
        nh_private_.param<double>("publish_rate", publish_rate, 20.0);
        // 看门狗超时:超过此时间未收到任何有效距离帧 → 重发 0x89 触发指令
        // 解决"启动时设备未接 / 中途拔掉再接上"后从机不会自动开始发的恢复问题
        nh_private_.param<double>("trigger_resend_interval", trigger_resend_interval_, 2.0);

        // === 订阅 uav pose (用于算 ray_dir + hit_point) ===
        uav_pose_sub_ = nh_.subscribe(uav_pose_topic, 10,
            &LaserBeamNode::uavPoseCallback, this);

        // === 发布 LidarMeasurement (waypoint_executor 订阅的同 topic) ===
        measurement_pub_ = nh_.advertise<multi_uav_strike::LidarMeasurement>(
            "lidar/measurement", 10);

        // === 配置串口 ===
        serial_.setPort(port);
        serial_.setBaudrate(baudrate);
        // setTimeout 形参是非 const 引用,必须先存到 lvalue 再传入
        serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(100);
        serial_.setTimeout(serial_timeout);

        try {
            serial_.open();
        } catch (serial::IOException& e) {
            ROS_ERROR("[LaserBeam] SPD1200N2: open %s failed: %s",
                      port.c_str(), e.what());
            ros::shutdown();
            return;
        }

        if (!serial_.isOpen()) {
            ROS_ERROR("[LaserBeam] SPD1200N2: serial port %s not open", port.c_str());
            ros::shutdown();
            return;
        }

        // 触发连续测距
        uint8_t cmd[] = {0x55, 0xAA, 0x89, 0xFF, 0xFF, 0xFF, 0xFF, 0x85};
        serial_.write(cmd, sizeof(cmd));
        ROS_INFO("[LaserBeam] SPD1200N2 opened on %s @ %d baud, frame_id=%s, "
                 "range=[%.2f, %.2f] m, uav_pose=%s",
                 port.c_str(), baudrate, frame_id_.c_str(),
                 min_range_, max_range_, uav_pose_topic.c_str());

        // === 启动串口读取线程 ===
        read_thread_ = boost::thread(&LaserBeamNode::readLoop, this);

        // === 启动发布定时器 (publish_rate Hz) ===
        publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate),
            &LaserBeamNode::publishTimerCallback, this);

        // === 启动触发指令看门狗 (1Hz 检查,超时重发 0x89) ===
        //   协议是"请求触发型":主机发 0x89 后从机才连续回,设备中途接入
        //   必须再发一次 0x89 从机才会开始。连接正常时 timer 立即 return,
        //   不会冗余触发。
        trigger_watchdog_timer_ = nh_.createTimer(ros::Duration(1.0),
            &LaserBeamNode::triggerWatchdogCallback, this);
    }

    ~LaserBeamNode() {
        if (serial_.isOpen()) {
            // 停止连续测距
            uint8_t stop[] = {0x55, 0xAA, 0x8E, 0xFF, 0xFF, 0xFF, 0xFF, 0x8A};
            serial_.write(stop, sizeof(stop));
            serial_.close();
        }
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
    }

private:
    ros::NodeHandle nh_, nh_private_;
    ros::Subscriber uav_pose_sub_;
    ros::Publisher  measurement_pub_;
    ros::Timer      publish_timer_;
    ros::Timer      trigger_watchdog_timer_;   // 触发指令看门狗
    serial::Serial  serial_;
    boost::thread   read_thread_;

    // 参数
    double min_range_, max_range_;
    double ground_range_abs_tol_, ground_range_rel_tol_;
    double ground_z_;
    std::string frame_id_;
    double trigger_resend_interval_;           // 看门狗超时阈值 (s)

    // 状态
    geometry_msgs::PoseStamped current_uav_pose_;
    bool have_uav_pose_ = false;

    // 串口读到的最新距离 (publish_timer 按 publish_rate 取最新一次发布)
    boost::mutex latest_mutex_;
    double  latest_distance_m_ = std::numeric_limits<double>::quiet_NaN();
    bool    have_distance_     = false;
    ros::Time latest_distance_time_;
    ros::Time last_data_time_;                 // 最近一次有效距离帧 wall clock (看门狗用)

    // 异常计数 (throttle warn 用)
    int checksum_fail_count_ = 0;
    int sta_fail_count_      = 0;

    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 真机: mavros/local_position/pose 是 ENU; 这里直接当 NWU 处理
        // (z 朝上 = ENU z 朝上 = NWU z 朝上, 与 gimbal_simulator 真机分支一致)
        current_uav_pose_ = *msg;
        have_uav_pose_    = true;
    }

    // === SPD1200N2 串口读取: 解析 8 字节帧, 校验和检查, 缓存最新有效距离 ===
    void readLoop() {
        std::vector<uint8_t> buffer;
        while (ros::ok()) {
            if (serial_.available()) {
                uint8_t byte;
                serial_.read(&byte, 1);
                buffer.push_back(byte);

                if (buffer.size() >= 8) {
                    // 找帧头 0x55 0xAA
                    size_t pos = 0;
                    while (pos < buffer.size() - 1) {
                        if (buffer[pos] == 0x55 && buffer[pos + 1] == 0xAA) break;
                        pos++;
                    }

                    if (pos + 8 > buffer.size()) {
                        break;  // 累积字节不足以构成完整帧,继续累积
                    }

                    uint8_t frame[8];
                    memcpy(frame, &buffer[pos], 8);

                    uint8_t sum = 0;
                    for (int i = 0; i < 7; ++i) sum += frame[i];
                    if (sum != frame[7]) {
                        checksum_fail_count_++;
                        ROS_WARN_THROTTLE(5.0,
                            "[LaserBeam] SPD1200N2 checksum failed (count=%d)",
                            checksum_fail_count_);
                        buffer.erase(buffer.begin(), buffer.begin() + pos + 1);
                        continue;
                    }

                    if (frame[2] == 0x89) {
                        // 连续测距回复
                        const uint8_t sta = frame[3];
                        if (sta == 0x01) {
                            const uint16_t dist_raw = (frame[5] << 8) | frame[6];
                            const double distance_m = dist_raw / 10.0;

                            boost::lock_guard<boost::mutex> lock(latest_mutex_);
                            latest_distance_m_    = distance_m;
                            latest_distance_time_ = ros::Time::now();
                            have_distance_        = true;
                            last_data_time_       = ros::Time::now();  // 看门狗:连接正常
                        } else {
                            sta_fail_count_++;
                            ROS_WARN_THROTTLE(5.0,
                                "[LaserBeam] SPD1200N2 measurement failed, STA=0x%02X "
                                "(count=%d)", sta, sta_fail_count_);
                        }
                    }
                    // 已处理完整帧,丢弃
                    buffer.erase(buffer.begin(), buffer.begin() + pos + 8);
                }
            }
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
    }

    // === 发布 LidarMeasurement (按 publish_rate 拍最新距离 + uav_pose 计算) ===
    void publishTimerCallback(const ros::TimerEvent&) {
        if (!have_uav_pose_) return;

        double range;
        ros::Time distance_time;
        {
            boost::lock_guard<boost::mutex> lock(latest_mutex_);
            if (!have_distance_) return;
            range         = latest_distance_m_;
            distance_time = latest_distance_time_;
        }

        if (!std::isfinite(range)) return;
        if (range < min_range_ || range > max_range_) return;  // 量程外丢弃

        // ray_dir = q_body_world * UnitX() (机体系 +X 方向)
        const auto& oq = current_uav_pose_.pose.orientation;
        Eigen::Quaterniond q(oq.w, oq.x, oq.y, oq.z);
        if (q.norm() < 1e-9) q = Eigen::Quaterniond::Identity();
        q.normalize();
        const Eigen::Vector3d ray_dir = q * Eigen::Vector3d::UnitX();

        // ground_range: 射线打到 ground_z 平面的距离 (NWU z 朝上)
        //   ray_dir.z < 0: 射线朝下
        //   uav_z > ground_z: UAV 在地面之上
        //   t = (ground_z - uav_z) / ray_dir.z > 0
        double ground_range = std::numeric_limits<double>::infinity();
        if (ray_dir.z() < -1e-9 && current_uav_pose_.pose.position.z > ground_z_) {
            ground_range = (ground_z_ - current_uav_pose_.pose.position.z) / ray_dir.z();
            if (ground_range < min_range_ || ground_range > max_range_) {
                ground_range = std::numeric_limits<double>::infinity();
            }
        }

        // hit_type 判据 (参考 gimbal_simulator L515-528 tolerance 法)
        uint8_t hit_type = multi_uav_strike::LidarMeasurement::HIT_NONE;
        if (std::isfinite(ground_range)) {
            const double tolerance = std::max(
                ground_range_abs_tol_, ground_range_rel_tol_ * ground_range);
            if (std::fabs(range - ground_range) <= tolerance) {
                hit_type = multi_uav_strike::LidarMeasurement::HIT_GROUND;
            } else {
                hit_type = multi_uav_strike::LidarMeasurement::HIT_BUILDING;
            }
        } else {
            // 射线朝上或 UAV 已在地面之下,不可能是地面 → 视为建筑障碍
            hit_type = multi_uav_strike::LidarMeasurement::HIT_BUILDING;
        }

        // hit_point = uav_pose + ray_dir * range
        const geometry_msgs::Point& p = current_uav_pose_.pose.position;

        multi_uav_strike::LidarMeasurement detailed;
        detailed.header.stamp    = ros::Time::now();
        detailed.header.frame_id = frame_id_;
        detailed.range           = static_cast<float>(range);
        detailed.min_range       = static_cast<float>(min_range_);
        detailed.max_range       = static_cast<float>(max_range_);
        detailed.hit_type        = hit_type;
        detailed.hit_point.x     = p.x + ray_dir.x() * range;
        detailed.hit_point.y     = p.y + ray_dir.y() * range;
        detailed.hit_point.z     = p.z + ray_dir.z() * range;

        measurement_pub_.publish(detailed);
    }

    // === 触发指令看门狗 (1Hz) ===
    //   协议是请求触发式:0x89 触发后从机才连续回。
    //   场景:
    //     (1) 启动时设备未接 → 构造时的 0x89 写入空 → 永远没数据
    //     (2) 设备中途拔掉再接上 → 从机默认等待指令,不会自动开始发
    //   解决: 1Hz 检查 last_data_time_, 超过 trigger_resend_interval_ 重发 0x89。
    //   连接正常时 since_data < threshold → 立即 return, 不冗余触发。
    void triggerWatchdogCallback(const ros::TimerEvent&) {
        if (!serial_.isOpen()) return;

        ros::Time data_time;
        {
            boost::lock_guard<boost::mutex> lock(latest_mutex_);
            data_time = last_data_time_;
        }

        const ros::Time now = ros::Time::now();
        // last_data_time_ 仍为 ros::Time(0) 表示从未收到过 → 强制首次重发
        // (构造时已发过一次 0x89,启动 1s 后仍无数据 = 第一次触发丢失,重发)
        const double since_data = data_time.isZero()
            ? trigger_resend_interval_ + 1.0
            : (now - data_time).toSec();

        if (since_data < trigger_resend_interval_) {
            return;  // 连接正常,不重发
        }

        // 超时 → 重发 0x89 触发指令
        uint8_t cmd[] = {0x55, 0xAA, 0x89, 0xFF, 0xFF, 0xFF, 0xFF, 0x85};
        serial_.write(cmd, sizeof(cmd));

        if (data_time.isZero()) {
            ROS_WARN_THROTTLE(2.0,
                "[LaserBeam] SPD1200N2 no data yet, re-triggering 0x89");
        } else {
            ROS_WARN_THROTTLE(2.0,
                "[LaserBeam] SPD1200N2 no data for %.1fs, re-triggering 0x89",
                (now - data_time).toSec());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laserBeam_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    LaserBeamNode node(nh, nh_private);
    ros::spin();
    return 0;
}