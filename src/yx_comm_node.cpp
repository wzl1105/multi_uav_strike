// =============================================================================
// yx_comm_node: YX 弹药载荷 RS-485 通信节点
// =============================================================================
//
// 协议详见 docs/YX_485通信协议.md
//   - 下行帧 10 字节 (机载 → YX):EB 90 + 01H + CMD + D[2] + E[2] + CRC16/MODBUS[2]
//   - 上行帧 16 字节 (YX → 机载):AA 55 + 02H + 状态字0 + 预留[11] + 累加和[1]
//   - 串口 /dev/ttyS3 @ 115200,GPIO1-C2 高发低收
//
// ROS 接口:
//   发布 /system/payload_type  =  std_msgs/UInt8  =  7 (NHY_PAYLOAD_TYPE_MUNITION)
//   发布 /yx/status            =  multi_uav_strike/YxStatus (8 个状态位 + 原始字节 + 校验)
//   发布 /yx/raw_frame         =  std_msgs::UInt8MultiArray (最近合法上行帧 16 字节)
//   订阅 /yx/cmd               =  std_msgs/UInt8  (0..5,按"连续 N 次"规则触发)
//
// "连续 N 次"语义(2026-08-03 与用户确认):同指令连续 N 次发送,中间不能夹杂其他指令,
// YX 端按"同一指令连续 N 次"判定。本节点用定时器 50ms 一帧循环发送 N 次。
//
// 不进 systemd 自启动,由用户在板子自启动脚本里调 yx_comm.launch 启动。
// =============================================================================

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_srvs/Trigger.h>
#include <multi_uav_strike/YxStatus.h>
#include <serial/serial.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <map>
#include <signal.h>
#include <string>
#include <sys/types.h>
#include <unistd.h>

// ---- 协议常量 ----
static constexpr uint8_t  YX_SYNC_HI     = 0xEB;
static constexpr uint8_t  YX_SYNC_LO     = 0x90;
static constexpr uint8_t  YX_ADDR_OB     = 0x01;  // 机载地址(下行帧 B 字段)
static constexpr uint8_t  YX_ADDR_YX     = 0x02;  // YX 地址(上行帧地址字段)

static constexpr uint8_t  YX_UP_SYNC_HI  = 0xAA;
static constexpr uint8_t  YX_UP_SYNC_LO  = 0x55;

static constexpr int      YX_DL_FRAME_LEN = 10;   // 下行帧总长
static constexpr int      YX_UL_FRAME_LEN = 16;   // 上行帧总长

// 指令码 + 触发"连续 N 次"的 N (自检无 N 概念,周期发)
struct CmdEntry { uint8_t code; int repeat_count; const char* name; };
static const std::map<uint8_t, CmdEntry> YX_CMD_TABLE = {
    {0, {0x3C, 0,  "self_test"}},
    {1, {0x69, 10, "stage2_release"}},
    {2, {0x87, 10, "qb_charge"}},
    {3, {0x1B, 3,  "qb_discharge"}},
    {4, {0x4E, 10, "smoke_trigger"}},
    {5, {0xD2, 10, "self_destruct"}},
};

// =============================================================================
// CRC-16/MODBUS (poly=0xA001, init=0xFFFF),输出大端
// =============================================================================
static uint16_t crc16Modbus(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x0001) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001)
                                 : static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}

// =============================================================================
// GPIO 控制(收发切换)
// =============================================================================
// 参考板子自启动脚本 set_rs485_receive_mode 的逻辑:
//   1) 启动后台 `gpioset -m signal -b <chip> <line>=0` 持续占有并保持低电平
//      (板子有 libgpiod 命令时优先,失败回退 sysfs)
//   2) sysfs: export GPIO N → echo low > direction (原子切为输出低电平)
//      失败回退 echo out > direction + echo 0 > value
//   3) 节点要发送时: kill gpioset 释放 line → 节点 sysfs writeValue(1) →
//      发送完 writeValue(0) → 重新启动 gpioset 占位 (避免漂移)
//
// 注: 本节点**只**依赖 sysfs(板子上一定可用),不依赖 libgpiod C 库,
// 启动时 std::system() 调 `gpioset` 命令作为可选 backend。开发机没装也无所谓,
// 失败回退 sysfs。
// =============================================================================
class GpioPin {
public:
    GpioPin() : fd_(-1), gpioset_pid_(-1) {}
    ~GpioPin() { close(); }

    struct Config {
        std::string gpio_chip;     // libgpiod chip 名字, 如 "gpiochip1"
        int         gpio_line;     // libgpiod line 偏移, 如 18 (gpio1_C2)
        int         gpio_number;   // sysfs 全局 GPIO 编号, 如 50 (gpio1_C2)
        std::string pidfile;       // gpioset PID 写入路径, 节点退出时 kill
        bool        kill_stale;    // 启动时是否先杀残留 gpioset
        bool        use_gpioset;   // 是否启用 gpioset backend (空闲时占位)
    };

    bool open(const Config& cfg) {
        if (fd_ >= 0) return true;
        cfg_ = cfg;
        // ---- 步骤 1: 杀掉残留的 gpioset 进程(若之前的脚本/systemd 留下的) ----
        //   必须在 sysfs export 前做,否则 gpioset 持有同一 line 会让 sysfs write 被拒
        if (cfg_.kill_stale) {
            killStaleGpioset();
        }
        // ---- 步骤 2: 优先 gpioset 后台占位 ----
        if (cfg_.use_gpioset) {
            if (startGpiosetHoldLow()) {
                ROS_INFO("[yx_comm] GPIO %s line %d held low by background gpioset (pid=%d)",
                         cfg_.gpio_chip.c_str(), cfg_.gpio_line, gpioset_pid_);
                // 注: gpioset -m signal 持有时,本节点的 sysfs 不能 write 同一 line。
                // 所以 **仅记录 PID 用于关闭**,不返回 true 让 sendSingleFrame 走 sysfs。
                // 改为:不依赖 gpioset,直接 sysfs 接管(下方步骤 3)。
                killGpioset();  // 杀掉,下面 sysfs 接管
            } else {
                ROS_WARN("[yx_comm] gpioset backend unavailable, falling back to sysfs");
            }
        }
        // ---- 步骤 3: sysfs 接管 ----
        const std::string base = "/sys/class/gpio/gpio" + std::to_string(cfg_.gpio_number);
        // 单次初始化最多等待1秒；失败后由节点定时器继续重试，避免阻塞其它回调。
        for (int i = 0; i < 5 && !dirExists(base); ++i) {
            int export_fd = ::open("/sys/class/gpio/export", O_WRONLY);
            if (export_fd >= 0) {
                const std::string number = std::to_string(cfg_.gpio_number);
                const ssize_t written = ::write(export_fd, number.data(), number.size());
                const int write_errno = errno;
                ::close(export_fd);
                if (written < 0 && write_errno != EBUSY) {
                    ROS_WARN_THROTTLE(1.0,
                        "[yx_comm] export GPIO %d failed (errno=%d %s), retrying",
                        cfg_.gpio_number, write_errno, std::strerror(write_errno));
                }
            } else {
                ROS_WARN_THROTTLE(1.0,
                    "[yx_comm] open GPIO export failed (errno=%d %s), retrying",
                    errno, std::strerror(errno));
            }
            usleep(200 * 1000);
        }
        if (!dirExists(base)) {
            ROS_WARN("[yx_comm] GPIO %d export did not become ready in this attempt",
                      cfg_.gpio_number);
            return false;
        }
        // 原子切为输出低电平: echo low > direction (跟脚本逻辑一致)
        std::ofstream dir_low(base + "/direction");
        bool low_atomic_ok = false;
        if (dir_low.is_open()) {
            dir_low << "low";
            dir_low.close();
            low_atomic_ok = true;
        }
        if (!low_atomic_ok) {
            // 回退: direction=out + value=0
            std::ofstream dir_out(base + "/direction");
            if (dir_out.is_open()) { dir_out << "out"; dir_out.close(); }
            std::ofstream v0(base + "/value");
            if (v0.is_open()) { v0 << "0"; v0.close(); }
            ROS_WARN("[yx_comm] GPIO %d direction 'low' not supported, fallback to out+0",
                     cfg_.gpio_number);
        } else {
            ROS_INFO("[yx_comm] GPIO %d set to output-low atomically via direction=low",
                     cfg_.gpio_number);
        }
        // ---- 步骤 4: 打开 value fd 给后续动态切换 ----
        fd_ = ::open((base + "/value").c_str(), O_WRONLY);
        if (fd_ < 0) {
            ROS_ERROR("[yx_comm] GPIO %d value open failed (errno=%d %s)",
                      cfg_.gpio_number, errno, std::strerror(errno));
            return false;
        }
        // ---- 步骤 5: 节点退出前若需要,重新启动 gpioset 占位(可选,默认 false) ----
        //   节点正常运行时 GPIO 由本节点 sysfs 控制,空闲漂移风险低;
        //   节点退出后由 systemd 重启脚本决定是否再调 set_rs485_receive_mode。
        //   不主动重启 gpioset,避免节点退出时多一次 fork。
        ROS_INFO("[yx_comm] GPIO %d (chip=%s line=%d) ready, mode=sysfs-direct",
                 cfg_.gpio_number, cfg_.gpio_chip.c_str(), cfg_.gpio_line);
        return true;
    }

    void writeValue(int v) {
        if (fd_ < 0) return;
        const char c = v ? '1' : '0';
        ssize_t n = ::write(fd_, &c, 1);
        (void)n;
    }

    void close() {
        // 节点退出时主动恢复低电平(接收模式)
        if (fd_ >= 0) {
            const char c = '0';
            (void)::write(fd_, &c, 1);
            ::close(fd_);
            fd_ = -1;
        }
        // 不主动重启 gpioset 占位,留给 systemd 脚本决定
        killGpioset();
    }

private:
    static bool dirExists(const std::string& path) {
        std::ifstream f(path + "/direction");
        return f.good();
    }

    void killStaleGpioset() {
        // pkill -f 'gpioset.*gpio1_C2' 或通用 line 匹配
        // 不强依赖 pkill 存在,失败忽略
        const std::string cmd =
            "pkill -f 'gpioset.*line " + std::to_string(cfg_.gpio_line) + "' "
            "2>/dev/null; sleep 0.05; true";
        (void)std::system(cmd.c_str());
    }

    void killGpioset() {
        if (gpioset_pid_ > 0) {
            ::kill(gpioset_pid_, SIGTERM);
            // wait briefly
            for (int i = 0; i < 10; ++i) {
                if (::kill(gpioset_pid_, 0) != 0) break;
                usleep(50 * 1000);
            }
            ::kill(gpioset_pid_, SIGKILL);  // 兜底
            gpioset_pid_ = -1;
            if (!cfg_.pidfile.empty()) {
                ::unlink(cfg_.pidfile.c_str());
            }
        }
    }

    // 启动 `gpioset -m signal -b <chip> <line>=0` 后台持有 line 拉低
    // 返回 true 表示 gpioset 命令可执行且成功 fork;否则返回 false。
    bool startGpiosetHoldLow() {
        // 用 popen 先做 which-style 检查: gpioset 是否存在
        if (std::system("command -v gpioset >/dev/null 2>&1") != 0) {
            return false;
        }
        // fork 后台
        const std::string cmd =
            "nohup gpioset -m signal -b " + cfg_.gpio_chip + " " +
            std::to_string(cfg_.gpio_line) + "=0 "
            "> /dev/null 2>&1 & echo $!";
        FILE* p = ::popen(cmd.c_str(), "r");
        if (!p) return false;
        char buf[32] = {0};
        if (!::fgets(buf, sizeof(buf), p)) {
            ::pclose(p);
            return false;
        }
        ::pclose(p);
        int pid = std::atoi(buf);
        if (pid <= 0) return false;
        gpioset_pid_ = pid;
        if (!cfg_.pidfile.empty()) {
            std::ofstream pf(cfg_.pidfile);
            if (pf.is_open()) { pf << pid; pf.close(); }
        }
        // 给 gpioset 一点时间抢到 line,然后立即在 open() 后续杀掉它
        // 让 sysfs 接管(否则 gpioset -m signal 持有时 sysfs write 会被内核拒)
        usleep(200 * 1000);
        return true;
    }

    int  fd_;
    int  gpioset_pid_;
    Config cfg_;
};

// =============================================================================
// 节点
// =============================================================================
class YxCommNode {
public:
    YxCommNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
        // ---- 参数 ----
        pnh_.param<std::string>("serial_port", serial_port_, "/dev/ttyS3");
        pnh_.param<int>("baudrate",         baudrate_,    115200);
        // GPIO 配置:跟板子自启动脚本对齐,chip+line 给 gpioset 命令用,gpio_number 给 sysfs 用。
        // 节点主路径是 sysfs,gpioset 仅作为"启动时清理残留占位"的可选 backend。
        pnh_.param<std::string>("gpio_chip", gpio_chip_,   "gpiochip1");
        pnh_.param<int>("gpio_line",        gpio_line_,     18);     // gpio1_C2 = line 18
        pnh_.param<int>("gpio_number",      gpio_number_,   50);     // sysfs 全局编号
        pnh_.param<std::string>("pidfile",   pidfile_, "/tmp/yx_rs485_gpioset.pid");
        pnh_.param<bool>("kill_stale_gpioset", kill_stale_gpioset_, true);
        pnh_.param<bool>("use_gpioset",     use_gpioset_, false);   // 默认 sysfs,板子上有需要可改 true
        pnh_.param<double>("self_test_hz",  self_test_hz_, 1.0);
        pnh_.param<double>("send_interval_sec", send_interval_sec_, 0.05);  // 50ms 一帧
        pnh_.param<int>("gpio_settle_ms",   gpio_settle_ms_, 2);
        pnh_.param<double>("gpio_retry_interval_sec", gpio_retry_interval_sec_, 1.0);
        pnh_.param<int>("payload_type",     payload_type_, 7);     // NHY_PAYLOAD_TYPE_MUNITION

        // ---- payload_type latched 发布 ----
        pub_payload_type_ = nh_.advertise<std_msgs::UInt8>("system/payload_type", 1, true);
        std_msgs::UInt8 pt;
        pt.data = static_cast<uint8_t>(payload_type_);
        pub_payload_type_.publish(pt);
        ROS_INFO("[yx_comm] /system/payload_type = %d (latched)", payload_type_);

        // ---- 状态/原始帧发布 ----
        pub_status_    = nh_.advertise<multi_uav_strike::YxStatus>("yx/status", 10);
        pub_raw_frame_ = nh_.advertise<std_msgs::UInt8MultiArray>("yx/raw_frame", 10);

        // ---- 指令订阅 ----
        sub_cmd_ = nh_.subscribe("yx/cmd", 10, &YxCommNode::cmdCallback, this);

        // 拉烟与弹射气体发生器一样使用 Trigger Service 作为上层入口，
        // 底层仍走 YX RS-485 的 0x4E 连续 10 帧序列，不经过 power_interact_node。
        srv_trigger_smoke_ = nh_.advertiseService(
            "trigger_smoke", &YxCommNode::triggerSmokeCallback, this);

        // ---- GPIO 初始化 ----
        gpio_cfg_.gpio_chip   = gpio_chip_;
        gpio_cfg_.gpio_line   = gpio_line_;
        gpio_cfg_.gpio_number = gpio_number_;
        gpio_cfg_.pidfile     = pidfile_;
        gpio_cfg_.kill_stale  = kill_stale_gpioset_;
        gpio_cfg_.use_gpioset = use_gpioset_;
        gpio_ready_ = gpio_.open(gpio_cfg_);
        if (!gpio_ready_) {
            ROS_WARN("[yx_comm] GPIO init failed; retrying every %.2fs; transmit is disabled until ready",
                     gpio_retry_interval_sec_);
        }

        // 上电时其它服务可能暂时占用GPIO；失败后持续重试，成功后回调自动空转。
        gpio_retry_timer_ = nh_.createTimer(
            ros::Duration(std::max(gpio_retry_interval_sec_, 0.1)),
            &YxCommNode::gpioRetryTimerCb, this);

        // ---- 串口初始化 ----
        try {
            ser_.setPort(serial_port_);
            ser_.setBaudrate(baudrate_);
            auto to = serial::Timeout::simpleTimeout(100);
            ser_.setTimeout(to);
            ser_.open();
            ROS_INFO("[yx_comm] Serial %s @ %d 8N1 opened", serial_port_.c_str(), baudrate_);
        } catch (serial::IOException& e) {
            ROS_ERROR("[yx_comm] Serial open failed: %s — running in DEGRADED mode (no comm)",
                      e.what());
        }

        // ---- 自检周期 timer ----
        self_test_timer_ = nh_.createTimer(
            ros::Duration(1.0 / std::max(self_test_hz_, 0.01)),
            &YxCommNode::selfTestTimerCb, this);

        // ---- 主循环 timer (100Hz 接收 + 状态发布 + SENDING 序列推进) ----
        run_timer_ = nh_.createTimer(
            ros::Duration(0.01),
            &YxCommNode::runTimerCb, this);
    }

    ~YxCommNode() {
        gpio_.writeValue(0);
        if (ser_.isOpen()) ser_.close();
    }

private:
    // ---- ROS 回调 ----
    void gpioRetryTimerCb(const ros::TimerEvent&) {
        if (gpio_ready_ || sending_active_) return;
        ROS_WARN_THROTTLE(2.0, "[yx_comm] GPIO %d not ready, retrying initialization",
                          gpio_number_);
        if (gpio_.open(gpio_cfg_)) {
            gpio_ready_ = true;
            ROS_WARN("[yx_comm] GPIO %d recovered; RS485 transmit is now enabled",
                     gpio_number_);
        }
    }

    bool startCommandSequence(uint8_t cmd_id, const char* source, bool allow_interrupt,
                              std::string* error = nullptr) {
        auto it = YX_CMD_TABLE.find(cmd_id);
        if (it == YX_CMD_TABLE.end() || cmd_id == 0) {
            if (error) *error = "invalid burst command";
            return false;
        }
        if (!ser_.isOpen()) {
            if (error) *error = "YX serial is not open";
            return false;
        }
        if (!gpio_ready_) {
            if (error) *error = "YX RS485 direction GPIO is not ready";
            return false;
        }
        if (sending_active_) {
            if (!allow_interrupt) {
                if (error) {
                    *error = "another YX command sequence is already active: " +
                             current_cmd_name_;
                }
                return false;
            }
            ROS_WARN("[yx_comm] %s interrupting in-flight sequence %s (%d/%d sent)",
                     source, current_cmd_name_.c_str(), sent_count_, total_count_);
        }

        current_cmd_code_ = it->second.code;
        current_cmd_name_ = it->second.name;
        total_count_ = it->second.repeat_count;
        sent_count_ = 0;
        last_send_time_ = ros::Time(0);
        sending_active_ = true;
        ROS_WARN("[yx_comm] %s -> %s (0x%02X), starting burst %d frames @ %.0fms",
                 source, current_cmd_name_.c_str(), current_cmd_code_, total_count_,
                 send_interval_sec_ * 1000.0);
        return true;
    }

    bool triggerSmokeCallback(std_srvs::Trigger::Request&,
                              std_srvs::Trigger::Response& res) {
        std::string error;
        // 2026-10-10: 拉烟优先级高于其它指令,允许打断 in-flight 序列。
        // 理由: 现场反馈"有时不触发"——0x69/0x87/0x4E/0xD2 等 burst 都是 10 帧 @50ms
        //   (=500ms), 期间任何第二次 /trigger_smoke 都会被原来的 allow_interrupt=false
        //   直接拒掉。拉烟是应急收尾动作, 必须保触发; 打断的代价是当前在跑的指令
        //   计数被 reset (YX 端校验逻辑需要连续 N 帧, 中断的序列对 YX 端而言是无效帧,
        //   本来也不期望有 ACK 反馈)。和 /yx/cmd 走 cmd=4 时的 allow_interrupt=true 一致。
        res.success = startCommandSequence(4, "/trigger_smoke", true, &error);
        if (res.success) {
            res.message = "Smoke trigger accepted: sending 0x4E burst (10 frames); "
                          "no completion feedback is required";
        } else {
            res.message = "Smoke trigger rejected: " + error;
            ROS_ERROR("[yx_comm] /trigger_smoke rejected: %s", error.c_str());
        }
        return true;
    }

    void cmdCallback(const std_msgs::UInt8::ConstPtr& msg) {
        const uint8_t cmd_id = msg->data;
        auto it = YX_CMD_TABLE.find(cmd_id);
        if (it == YX_CMD_TABLE.end()) {
            ROS_WARN_THROTTLE(2.0, "[yx_comm] /yx/cmd invalid id=%u (expect 0..5)", cmd_id);
            return;
        }
        // 自检指令走周期 timer,不进 SENDING 序列
        if (cmd_id == 0) {
            if (!gpio_ready_) {
                ROS_WARN_THROTTLE(2.0, "[yx_comm] GPIO not ready, drop periodic command");
                return;
            }
            sendSingleFrame(it->second.code);
            ROS_INFO("[yx_comm] /yx/cmd -> %s (single, periodic path)", it->second.name);
            return;
        }
        std::string error;
        if (!startCommandSequence(cmd_id, "/yx/cmd", true, &error)) {
            ROS_WARN("[yx_comm] /yx/cmd id=%u rejected: %s", cmd_id, error.c_str());
        }
    }

    void selfTestTimerCb(const ros::TimerEvent&) {
        // === 2026-08-03: 自检 timer 必须避让 SENDING 序列 ===
        //   协议要求"同指令连续 N 次,中间不能夹杂其他指令"。SENDING 在跑时
        //   插入 0x3C 自检会把 0x69/0x87/0x1B/0x4E/0xD2 的连续计数打断,YX 端
        //   把"连续"判定清零 → 解保/起爆/自毁永远凑不到 N 次。
        //   跳过本周期自检,下个周期 SENDING 结束后再补发(1 Hz 时少一两次自检
        //   对设备无害,自检是周期性的)。
        if (!gpio_ready_) return;
        if (sending_active_) {
            ROS_DEBUG_THROTTLE(2.0,
                "[yx_comm] self-test 0x3C skipped: SENDING %s in progress (%d/%d)",
                current_cmd_name_.c_str(), sent_count_, total_count_);
            return;
        }
        sendSingleFrame(0x3C);
    }

    // ---- 主循环:接收 + SENDING 推进 + 状态发布 ----
    void runTimerCb(const ros::TimerEvent&) {
        // (a) 推进 SENDING 序列
        if (sending_active_) {
            ros::Time now = ros::Time::now();
            if (last_send_time_.toSec() == 0.0 ||
                (now - last_send_time_).toSec() >= send_interval_sec_) {
                sendSingleFrame(current_cmd_code_);
                ++sent_count_;
                last_send_time_ = now;
                if (sent_count_ >= total_count_) {
                    sending_active_ = false;
                    ROS_INFO("[yx_comm] burst %s complete (%d frames sent)",
                             current_cmd_name_.c_str(), sent_count_);
                }
            }
        }

        // (b) 接收 + 解析上行帧
        if (ser_.isOpen()) {
            size_t avail = ser_.available();
            if (avail > 0) {
                std::vector<uint8_t> buf(avail);
                size_t got = ser_.read(buf.data(), avail);
                recv_buf_.insert(recv_buf_.end(), buf.begin(), buf.begin() + got);
                parseUplink();
            }
        }

        // (c) 周期发布 yx/status (即使没新帧也维持低频,便于 GS 探活)
        publishStatus();
    }

    // ---- 下行帧组帧 + 发送(带 GPIO 切换)----
    void sendSingleFrame(uint8_t cmd) {
        if (!gpio_ready_) {
            ROS_WARN_THROTTLE(2.0, "[yx_comm] GPIO not ready, drop cmd 0x%02X", cmd);
            return;
        }
        if (!ser_.isOpen()) {
            ROS_WARN_THROTTLE(2.0, "[yx_comm] serial not open, drop cmd 0x%02X", cmd);
            return;
        }
        uint8_t frame[YX_DL_FRAME_LEN] = {0};
        frame[0] = YX_SYNC_HI;          // 0xEB
        frame[1] = YX_SYNC_LO;          // 0x90
        frame[2] = YX_ADDR_OB;          // 0x01
        frame[3] = cmd;                 // CMD
        frame[4] = 0x00; frame[5] = 0x00;   // D 预留
        frame[6] = 0x00; frame[7] = 0x00;   // E 预留
        // 已确认的拉烟完整帧为 EB 90 01 4E 00 00 00 00 F6 26，
        // 其 CRC 计算范围包含同步头 bytes[0..7]。其他旧指令暂保持原有
        // bytes[2..7] 范围，避免在未获得其它指令的确认报文前改变旧链路。
        uint16_t crc = (cmd == 0x4E)
            ? crc16Modbus(frame, 8)
            : crc16Modbus(frame + 2, 6);
        frame[8] = (crc     ) & 0xFF;   // 低字节在前 (MODBUS 标准序,与 YX 端对齐)
        frame[9] = (crc >> 8) & 0xFF;

        // 切发送 → 发送 → 切接收
        gpio_.writeValue(1);
        if (gpio_settle_ms_ > 0) {
            usleep(static_cast<useconds_t>(gpio_settle_ms_) * 1000);
        }
        try {
            ser_.write(frame, YX_DL_FRAME_LEN);
        } catch (serial::IOException& e) {
            ROS_WARN_THROTTLE(2.0, "[yx_comm] write failed: %s", e.what());
        }
        ser_.flush();
        if (gpio_settle_ms_ > 0) {
            usleep(static_cast<useconds_t>(gpio_settle_ms_) * 1000);
        }
        gpio_.writeValue(0);
        ROS_DEBUG("[yx_comm] sent cmd=0x%02X crc=0x%04X", cmd, crc);
    }

    // ---- 上行帧解析(滑动找 0xAA 0x55 同步头)----
    void parseUplink() {
        while (recv_buf_.size() >= YX_UL_FRAME_LEN) {
            // 找同步头
            size_t start = 0;
            while (start + 1 < recv_buf_.size()) {
                if (recv_buf_[start] == YX_UP_SYNC_HI && recv_buf_[start + 1] == YX_UP_SYNC_LO) {
                    break;
                }
                ++start;
            }
            if (start + YX_UL_FRAME_LEN > recv_buf_.size()) {
                // 凑不齐一帧,留着等下一波
                if (start > 0) recv_buf_.erase(recv_buf_.begin(), recv_buf_.begin() + start);
                return;
            }
            // 提取帧
            uint8_t frame[YX_UL_FRAME_LEN];
            std::memcpy(frame, &recv_buf_[start], YX_UL_FRAME_LEN);
            recv_buf_.erase(recv_buf_.begin(), recv_buf_.begin() + start + YX_UL_FRAME_LEN);

            // 地址码检查
            if (frame[2] != YX_ADDR_YX) {
                ROS_WARN_THROTTLE(2.0, "[yx_comm] uplink addr=0x%02X != 0x02, drop",
                                  frame[2]);
                continue;
            }
            // 累加和校验
            uint8_t sum = 0;
            for (int i = 0; i < YX_UL_FRAME_LEN - 1; ++i) sum += frame[i];
            bool valid = ((sum & 0xFF) == frame[15]);
            if (!valid) {
                ROS_WARN_THROTTLE(2.0,
                    "[yx_comm] uplink checksum fail: sum=0x%02X frame[15]=0x%02X",
                    sum, frame[15]);
                last_frame_valid_ = false;
                continue;
            }

            // 解析
            last_status_byte_ = frame[3];
            last_frame_valid_ = true;
            last_frame_recv_time_ = ros::Time::now();

            // 发布原始帧
            std_msgs::UInt8MultiArray raw;
            raw.data.assign(frame, frame + YX_UL_FRAME_LEN);
            pub_raw_frame_.publish(raw);
        }
    }

    void publishStatus() {
        multi_uav_strike::YxStatus s;
        s.header.stamp = ros::Time::now();
        s.header.frame_id = "yx";
        s.raw_status_byte = last_status_byte_;
        s.frame_valid = last_frame_valid_;
        const uint8_t b = last_status_byte_;
        s.device_ok        = (b >> 0) & 0x01;
        s.self_test_done   = (b >> 1) & 0x01;
        s.mode_qb_reserved = (b >> 2) & 0x01;
        s.stage1_released  = (b >> 3) & 0x01;
        s.stage2_released  = (b >> 4) & 0x01;
        s.in_attack_state  = (b >> 5) & 0x01;
        s.qb_done          = (b >> 6) & 0x01;
        s.self_destruct    = (b >> 7) & 0x01;
        pub_status_.publish(s);
    }

    // ---- 成员 ----
    ros::NodeHandle nh_, pnh_;
    ros::Publisher  pub_payload_type_, pub_status_, pub_raw_frame_;
    ros::Subscriber sub_cmd_;
    ros::ServiceServer srv_trigger_smoke_;
    ros::Timer      self_test_timer_, run_timer_, gpio_retry_timer_;

    serial::Serial  ser_;
    GpioPin         gpio_;
    GpioPin::Config gpio_cfg_;
    bool            gpio_ready_ = false;
    std::vector<uint8_t> recv_buf_;

    // SENDING 序列
    bool        sending_active_   = false;
    uint8_t     current_cmd_code_ = 0;
    std::string current_cmd_name_;
    int         sent_count_       = 0;
    int         total_count_      = 0;
    ros::Time   last_send_time_;

    // 解析状态
    uint8_t     last_status_byte_ = 0;
    bool        last_frame_valid_ = false;
    ros::Time   last_frame_recv_time_;

    // 参数
    std::string serial_port_;
    int         baudrate_;
    std::string gpio_chip_;
    int         gpio_line_;
    int         gpio_number_;
    std::string pidfile_;
    bool        kill_stale_gpioset_;
    bool        use_gpioset_;
    double      self_test_hz_;
    double      send_interval_sec_;
    int         gpio_settle_ms_;
    double      gpio_retry_interval_sec_;
    int         payload_type_;
};

// =============================================================================
// main
// =============================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "yx_comm_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    YxCommNode node(nh, pnh);
    ros::spin();
    return 0;
}
