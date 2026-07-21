#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <multi_uav_strike/SimTarget.h>
#include <multi_uav_strike/SimTargets.h>
#include <XmlRpcValue.h>
#include <cmath>
#include <random>
#include <chrono>
#include <vector>
#include <string>

// ============================================================================
// target_motion_simulator_node — 多目标地面运动模拟器
//
// 2026-07 多目标改造:
//   - 支持 YAML 配置目标列表(不同类别 + 同类别多实例),每个目标独立 2D 随机运动模型。
//   - 发布 /sim_targets (SimTargets):全部目标的 label + NWU 位置,供 gimbal_simulator。
//   - 保留 /target_position (Point):主目标(primary_index,默认 0)位置,兼容
//     guidance_control_node 的 real_target_sub_ 与 rviz echo。
//   - target_marker 改为 MarkerArray,每目标一个 marker(按 label 上色)。
//
// 目标运动模型与单目标版本一致(切向/法向随机加速度),只是复制成 N 份。
// ============================================================================

class TargetMotionSimulator {
private:
    ros::NodeHandle nh_;
    ros::Publisher sim_targets_pub_;      // 全部目标 (SimTargets)
    ros::Publisher target_pub_;           // 主目标 (Point,兼容)
    ros::Publisher target_marker_pub_;    // MarkerArray (RViz)
    ros::Timer sim_timer_;
    ros::Timer acc_trigger_timer_;

    // 单个目标的完整状态 + 运动参数
    struct Target {
        uint32_t    id = 0;
        std::string label = "general_target";
        // 运动状态
        double x = 0.0, y = 0.0;
        double vx = 0.0, vy = 0.0;
        double theta = 0.0, omega = 0.0;

        // 初始运动参数
        double speed_init = 1.0;
        double heading_init = 0.0;
        // 运动参数(每目标独立)
        double max_speed = 10.0;
        double min_speed = 1.0;
        double max_tangential_acc = 2.0;
        double max_normal_acc = 1.0;
        // 当前随机加速度
        double cur_tang_acc = 0.0;
        double cur_norm_acc = 0.0;
    };
    std::vector<Target> targets_;

    // 全局默认(YAML 目标项缺省字段回落到这里,保持"删一行不炸"惯例)
    double def_max_speed_, def_min_speed_, def_max_tang_acc_, def_max_norm_acc_;
    double sim_freq_;
    double trigger_freq_;
    int    primary_index_;   // /target_position 取哪个目标(默认 0)

    // Marker
    double marker_size_;
    std::string marker_frame_;

    std::default_random_engine random_engine_;

public:
    TargetMotionSimulator() {
        unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
        random_engine_.seed(seed);
        ros::NodeHandle n("~");

        // 全局默认运动参数
        n.param<double>("max_speed", def_max_speed_, 10.0);
        n.param<double>("min_speed", def_min_speed_, 1.0);
        n.param<double>("max_tangential_acc", def_max_tang_acc_, 2.0);
        n.param<double>("max_normal_acc", def_max_norm_acc_, 1.0);
        n.param<double>("sim_freq", sim_freq_, 100.0);
        n.param<double>("trigger_freq", trigger_freq_, 0.1);
        n.param<int>("primary_index", primary_index_, 0);

        n.param<double>("marker_size", marker_size_, 1.0);
        n.param<std::string>("marker_frame", marker_frame_, "map");

        loadTargets(n);

        sim_targets_pub_   = nh_.advertise<multi_uav_strike::SimTargets>("sim_targets", 10);
        target_pub_        = nh_.advertise<geometry_msgs::Point>("target_position", 10);
        target_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("target_marker", 10);

        sim_timer_ = nh_.createTimer(ros::Duration(1.0/sim_freq_),
                                     &TargetMotionSimulator::simTimerCallback, this);
        acc_trigger_timer_ = nh_.createTimer(ros::Duration(1.0/trigger_freq_),
                                             &TargetMotionSimulator::accTriggerCallback, this);

        ROS_INFO("[TargetMotionSim] initialized with %zu target(s), primary_index=%d",
                 targets_.size(), primary_index_);
        for (const auto& t : targets_) {
            // ROS_INFO("  target[%u] label=%s init=(%.1f,%.1f) max_speed=%.2f",
            //          t.id, t.label.c_str(), t.x, t.y, t.max_speed);
            ROS_INFO("target[%u] %s pos=(%.1f %.1f) speed=%.1f heading=%.1fdeg",
                        t.id, t.label.c_str(), t.x, t.y, t.speed_init, t.heading_init*180/M_PI);
        }
    }

    // 从 ~targets 数组读取目标列表;若未配置则退化为单个默认目标(读旧的 x_init/y_init)
    void loadTargets(ros::NodeHandle& n) {
        XmlRpc::XmlRpcValue arr;
        if (n.getParam("targets", arr) && arr.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < arr.size(); ++i) {
                XmlRpc::XmlRpcValue& item = arr[i];
                Target t;
                t.id = static_cast<uint32_t>(i);
                t.label              = getStr(item, "label", "general_target");
                t.x                  = getNum(item, "x_init", 0.0);
                t.y                  = getNum(item, "y_init", 0.0);
                // 初始速度
                t.speed_init         = getNum(item,"speed_init",t.min_speed);
                // 初始方向
                t.heading_init       = getNum(item,"heading_init",0.0) * M_PI / 180.0;
                t.theta = t.heading_init;
                // 根据速度和方向计算速度分量
                t.vx = t.speed_init * cos(t.theta);
                t.vy = t.speed_init * sin(t.theta);

                t.max_speed          = getNum(item, "max_speed", def_max_speed_);
                t.min_speed          = getNum(item, "min_speed", def_min_speed_);
                t.max_tangential_acc = getNum(item, "max_tangential_acc", def_max_tang_acc_);
                t.max_normal_acc     = getNum(item, "max_normal_acc", def_max_norm_acc_);
                // t.vx = t.min_speed;
                // t.vy = 0.0;
                targets_.push_back(t);
            }
        }
        if (targets_.empty()) {
            // 兼容旧单目标配置
            Target t;
            t.id = 0;
            n.param<std::string>("label", t.label, "general_target");
            n.param<double>("x_init", t.x, 0.0);
            n.param<double>("y_init", t.y, 0.0);
            t.max_speed = def_max_speed_;
            t.min_speed = def_min_speed_;
            t.max_tangential_acc = def_max_tang_acc_;
            t.max_normal_acc = def_max_norm_acc_;

            // t.vx = t.min_speed;
            n.param<double>("speed_init", t.speed_init, t.min_speed);
            n.param<double>("heading_init", t.heading_init, 0.0);
            t.heading_init *= M_PI/180.0;
            t.theta = t.heading_init;
            t.vx    = t.speed_init*cos(t.theta);
            t.vy    = t.speed_init*sin(t.theta);

            targets_.push_back(t);
            ROS_WARN("[TargetMotionSim] no ~targets list found, fell back to single target");
        }
        if (primary_index_ < 0 || primary_index_ >= (int)targets_.size()) {
            primary_index_ = 0;
        }
    }

    static double getNum(XmlRpc::XmlRpcValue& item, const std::string& key, double def) {
        if (!item.hasMember(key)) return def;
        XmlRpc::XmlRpcValue& v = item[key];
        if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(v);
        if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)    return static_cast<int>(v);
        return def;
    }
    static std::string getStr(XmlRpc::XmlRpcValue& item, const std::string& key, const std::string& def) {
        if (!item.hasMember(key)) return def;
        XmlRpc::XmlRpcValue& v = item[key];
        if (v.getType() == XmlRpc::XmlRpcValue::TypeString) return static_cast<std::string>(v);
        return def;
    }

    // 低频触发:为每个目标重新随机切向/法向加速度
    void accTriggerCallback(const ros::TimerEvent&) {
        for (auto& t : targets_) {
            std::uniform_real_distribution<double> td(-t.max_tangential_acc, t.max_tangential_acc);
            std::uniform_real_distribution<double> nd(-t.max_normal_acc, t.max_normal_acc);
            t.cur_tang_acc = td(random_engine_);
            t.cur_norm_acc = nd(random_engine_);
        }
    }

    // 主仿真:更新所有目标并发布
    void simTimerCallback(const ros::TimerEvent&) {
        double dt = 1.0 / sim_freq_;
        for (auto& t : targets_) updateMotion(t, dt);

        // 1. 全部目标 (SimTargets)
        multi_uav_strike::SimTargets msg;
        msg.targets.reserve(targets_.size());
        for (const auto& t : targets_) {
            multi_uav_strike::SimTarget st;
            st.id = t.id;
            st.label = t.label;
            st.position.x = t.x;
            st.position.y = t.y;
            st.position.z = 0.0;
            msg.targets.push_back(st);
        }
        sim_targets_pub_.publish(msg);

        // 2. 主目标 (Point,兼容)
        if (!targets_.empty()) {
            const Target& p = targets_[primary_index_];
            geometry_msgs::Point pt;
            pt.x = p.x; pt.y = p.y; pt.z = 0.0;
            target_pub_.publish(pt);
        }

        // 3. MarkerArray
        publishTargetMarkers();
    }

    void publishTargetMarkers() {
        visualization_msgs::MarkerArray arr;
        for (const auto& t : targets_) {
            visualization_msgs::Marker m;
            m.header.frame_id = marker_frame_;
            m.header.stamp = ros::Time::now();
            m.ns = "target_marker";
            m.id = static_cast<int>(t.id);
            m.type = visualization_msgs::Marker::SPHERE;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = t.x;
            m.pose.position.y = t.y;
            m.pose.position.z = 0.0;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = marker_size_;
            colorForLabel(t.label, m.color);
            m.lifetime = ros::Duration(0);
            arr.markers.push_back(m);

            // 类别文字标签
            visualization_msgs::Marker txt;
            txt.header = m.header;
            txt.ns = "target_label";
            txt.id = static_cast<int>(t.id);
            txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            txt.action = visualization_msgs::Marker::ADD;
            txt.pose.position.x = t.x;
            txt.pose.position.y = t.y;
            txt.pose.position.z = marker_size_ + 0.5;
            txt.pose.orientation.w = 1.0;
            txt.scale.z = 1.0;
            txt.color.r = txt.color.g = txt.color.b = 1.0;
            txt.color.a = 1.0;
            txt.text = t.label;
            txt.lifetime = ros::Duration(0);
            arr.markers.push_back(txt);
        }
        target_marker_pub_.publish(arr);
    }

    // 简单的按 label 上色(人=红,车=蓝,其他=绿)
    static void colorForLabel(const std::string& label, std_msgs::ColorRGBA& c) {
        c.a = 1.0;
        if (label == "person" || label == "people" || label == "human") {
            c.r = 1.0; c.g = 0.0; c.b = 0.0;
        } else if (label == "car" || label == "vehicle" || label == "truck" || label == "bus") {
            c.r = 0.0; c.g = 0.3; c.b = 1.0;
        } else {
            c.r = 0.0; c.g = 1.0; c.b = 0.0;
        }
    }

    // 单目标运动模型(与原单目标版本一致)
    void updateMotion(Target& t, double dt) {
        double speed = std::sqrt(t.vx*t.vx + t.vy*t.vy);
        if (speed < 1e-6) speed = 1e-6;

        double new_speed = speed + t.cur_tang_acc * dt;
        new_speed = std::max(t.min_speed, std::min(t.max_speed, new_speed));

        t.omega = t.cur_norm_acc / speed;
        t.theta += t.omega * dt;
        t.theta = std::atan2(std::sin(t.theta), std::cos(t.theta));

        t.vx = new_speed * std::cos(t.theta);
        t.vy = new_speed * std::sin(t.theta);

        t.x += t.vx * dt;
        t.y += t.vy * dt;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_motion_simulator_node");
    TargetMotionSimulator simulator;
    ros::spin();
    return 0;
}
