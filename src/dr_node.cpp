// ============================================================================
//  dr_node.cpp —— 纯 DR 的 ROS 2 节点
//
//  订阅: /imu/data       (sensor_msgs/Imu)          → 姿态(AHRS) + 积分节拍
//        /chassis_info   (chassis_msgs/ChassisInfo) → speed_mps
//  发布: /tf              (tf2_msgs/TFMessage)       → odom -> base_link  ← 唯一 TF 输出
//        /dr_odom        (nav_msgs/Odometry)        → 位姿 + 速度（rviz 箭头/给其他节点用）
//        /dr_path        (nav_msgs/Path)            → DR 轨迹（rviz 画轨迹必须靠它）
//
//  注意：
//   * 不发布 /tf_static（按需求）；因此 rviz 的 Fixed Frame 用 odom。
//   * /chassis_info 的 header.stamp 在实车包里恒为 0（无时间戳）⇒ 车速时间基准取"收到那一刻的 IMU 时间"，
//     这样仿真钟(use_sim_time)与墙钟混用也不会算错年龄。
// ============================================================================
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <chassis_msgs/msg/chassis_info.hpp>

#include "dr_engine/dr_engine.h"
#include "dr_engine/log_sink.h"

namespace dr_engine
{

class DrNode : public rclcpp::Node
{
public:
    DrNode() : rclcpp::Node("dr_engine_node")
    {
        loadParams();

        // ---- 日志同时落盘（默认 <工程根>/log/dr.text，参数 log_file 可改）----
        {
            std::string err;
            if (LogFileSink::init(m_logFile, err))
                RCLCPP_INFO(get_logger(), "[日志] 同时写入文件: %s", LogFileSink::path().c_str());
            else
                RCLCPP_WARN(get_logger(), "[日志] 文件日志未启用: %s", err.c_str());
        }

        m_engine.init(m_params);

        // ---- 订阅 ----
        auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(m_qosDepth));
        auto spd_qos = rclcpp::QoS(rclcpp::KeepLast(m_qosDepth));
        if (!m_qosReliable)
        {
            imu_qos.best_effort();
            spd_qos.best_effort();
        }
        m_subImu = create_subscription<sensor_msgs::msg::Imu>(
            m_imuTopic, imu_qos,
            [this](const sensor_msgs::msg::Imu::ConstSharedPtr &msg) { onImu(msg); });
        m_subSpeed = create_subscription<chassis_msgs::msg::ChassisInfo>(
            m_speedTopic, spd_qos,
            [this](const chassis_msgs::msg::ChassisInfo::ConstSharedPtr &msg) { onSpeed(msg); });

        // ---- 发布 ----
        m_pubTf    = create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
        m_pubOdom  = create_publisher<nav_msgs::msg::Odometry>(m_odomTopic, rclcpp::QoS(10));
        m_pubPath  = create_publisher<nav_msgs::msg::Path>(m_pathTopic, rclcpp::QoS(10));
        m_pubMileage = create_publisher<geometry_msgs::msg::PointStamped>(m_mileageTopic, rclcpp::QoS(10));
        m_path.header.frame_id = m_parentFrame;

        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, m_publishRateHz));
        m_timerPub  = create_wall_timer(period, [this]() { onPublishTimer(); });
        // /dr_path 独立定时器（默认 1 Hz）：与上面的高频发布解耦，避免整条轨迹被几十 Hz 重发
        const auto path_period = std::chrono::duration<double>(1.0 / std::max(0.1, m_pathPublishHz));
        m_timerPath = create_wall_timer(path_period, [this]() { onPathTimer(); });
        m_timerLog  = create_wall_timer(std::chrono::duration<double>(m_logPeriodS), [this]() { onLogTimer(); });

        RCLCPP_INFO(get_logger(), "[启动] DrEngine 节点已就绪");
        RCLCPP_INFO(get_logger(), "[启动] 订阅 %s (%s) / %s (%s)  QoS=%s depth=%d",
                    m_imuTopic.c_str(), "sensor_msgs/Imu",
                    m_speedTopic.c_str(), "chassis_msgs/ChassisInfo",
                    m_qosReliable ? "reliable" : "best_effort", m_qosDepth);
        RCLCPP_INFO(get_logger(), "[启动] 发布 /tf (%s -> %s) + %s @%.1fHz  + %s @%.1fHz  + %s(总里程, PointStamped.%s)",
                    m_parentFrame.c_str(), m_childFrame.c_str(),
                    m_odomTopic.c_str(), m_publishRateHz,
                    m_pathTopic.c_str(), m_pathPublishHz,
                    m_mileageTopic.c_str(), m_mileageField.c_str());
        RCLCPP_INFO(get_logger(), "[启动] 位置解算模式: z_mode=%s", m_zModeName.c_str());
        RCLCPP_INFO(get_logger(), "[启动] 车速符号: gear_reverse_fix=%s (reverse_gear_value=%d ⇒ 该挡速度取负; "
                    "GEAR: N=1 D=2 R=3 P=4)", m_gearReverseFix ? "on" : "off", m_reverseGearValue);
        RCLCPP_INFO(get_logger(), "[启动] 算法口径: 姿态=FusionAhrs(6轴,gain=%.2f,accReject=%.1fdeg) "
                    "速度=chassis speed(scale=%.3f,timeout=%.2fs,hold_last=%d) 位置=∫v·dt",
                    m_params.ahrs_gain, m_params.ahrs_acc_reject_deg,
                    m_params.speed_scale, m_params.speed_timeout_s, static_cast<int>(m_params.hold_last_speed));
        RCLCPP_INFO(get_logger(), "[启动] 静止初始化: %.1fs / 最少 %d 帧; g_auto=%d (配置 g=%.5f); yaw0=%.2fdeg; "
                    "max_gap=%.3fs; use_sim_time=%d",
                    m_params.init_static_s, m_params.init_min_samples, static_cast<int>(m_params.g_auto),
                    m_params.g, m_params.yaw0_deg, m_params.max_gap_s,
                    static_cast<int>(this->get_parameter("use_sim_time").as_bool()));
        RCLCPP_INFO(get_logger(), "[启动] 不发布 /tf_static ⇒ rviz Fixed Frame 请用 '%s'",
                    m_parentFrame.c_str());
    }

    ~DrNode() override
    {
        RCLCPP_INFO(get_logger(), "[退出] IMU 帧=%ld 车速帧=%ld 里程=%.2fm 最终位姿=(%.3f, %.3f, %.3f) rpy=(%.2f, %.2f, %.2f)deg",
                    m_engine.stats().imu_cnt, m_engine.stats().speed_cnt,
                    m_engine.traveledDistance(),
                    m_engine.pose().pos.x(), m_engine.pose().pos.y(), m_engine.pose().pos.z(),
                    m_engine.pose().rpy.x() * 180.0 / M_PI,
                    m_engine.pose().rpy.y() * 180.0 / M_PI,
                    m_engine.pose().rpy.z() * 180.0 / M_PI);
        RCLCPP_INFO(get_logger(), "[退出] z_mode=%s /total_mileage=%s(字段 %s) 里程=%.2fm",
                    m_zModeName.c_str(), m_mileageTopic.c_str(), m_mileageField.c_str(),
                    m_engine.traveledDistance());
        if (m_engine.stats().imu_gap_cnt > 0 || m_engine.stats().imu_nonmono_cnt > 0 ||
            m_engine.stats().speed_stale_cnt > 0)
            RCLCPP_WARN(get_logger(), "[退出] 异常统计: IMU 断流=%ld 时间戳回跳=%ld 车速过期=%ld",
                        m_engine.stats().imu_gap_cnt, m_engine.stats().imu_nonmono_cnt,
                        m_engine.stats().speed_stale_cnt);
    }

private:
    void loadParams()
    {
        m_imuTopic   = declare_parameter<std::string>("imu_topic", "/imu/data");
        m_speedTopic = declare_parameter<std::string>("speed_topic", "/chassis_info");
        m_odomTopic  = declare_parameter<std::string>("odom_topic", "/dr_odom");
        m_pathTopic  = declare_parameter<std::string>("path_topic", "/dr_path");
        m_parentFrame = declare_parameter<std::string>("parent_frame", "odom");
        m_childFrame  = declare_parameter<std::string>("child_frame", "base_link");

        m_publishRateHz = declare_parameter<double>("publish_rate_hz", 50.0);
        // /dr_path 单独节流：它每条消息携带整条轨迹数组（可达 MB 级），
        // 不随 publish_rate_hz（tf/odom/里程的高频率）重发。
        m_pathPublishHz = declare_parameter<double>("path_publish_hz", 1.0);
        m_pathAppendHz  = declare_parameter<double>("path_append_hz", 10.0);
        m_pathMaxPoses  = declare_parameter<int>("path_max_poses", 200000);
        m_logPeriodS    = declare_parameter<double>("log_period_s", 1.0);
        m_qosReliable   = declare_parameter<bool>("qos_reliable", false);
        m_qosDepth      = declare_parameter<int>("qos_depth", 50);

        m_params.g                   = declare_parameter<double>("g", 9.80665);
        m_params.g_auto              = declare_parameter<bool>("g_auto", true);
        m_params.init_static_s       = declare_parameter<double>("init_static_s", 2.0);
        m_params.init_min_samples    = declare_parameter<int>("init_min_samples", 50);
        m_params.yaw0_deg            = declare_parameter<double>("yaw0_deg", 0.0);
        m_params.ahrs_gain           = declare_parameter<double>("ahrs_gain", 0.5);
        m_params.ahrs_acc_reject_deg = declare_parameter<double>("ahrs_acc_reject_deg", 10.0);
        m_params.ahrs_reject_timeout_s = declare_parameter<double>("ahrs_reject_timeout_s", 5.0);
        m_params.use_ahrs_offset     = declare_parameter<bool>("use_ahrs_offset", true);
        m_params.speed_scale         = declare_parameter<double>("speed_scale", 1.0);
        m_params.speed_timeout_s     = declare_parameter<double>("speed_timeout_s", 0.3);
        m_params.hold_last_speed     = declare_parameter<bool>("hold_last_speed", true);
        m_params.use_abs_speed       = declare_parameter<bool>("use_abs_speed", false);
        m_params.max_gap_s           = declare_parameter<double>("max_gap_s", 0.2);

        // 位置解算模式: "3d" = z 由 pitch 积分; "planar" = z 恒 0（地面车辆推荐）
        const std::string zmode = declare_parameter<std::string>("z_mode", "3d");
        if (zmode == "planar" || zmode == "2d")
            m_params.z_mode = ZMode::Planar;
        else
            m_params.z_mode = ZMode::Mode3D;
        m_zModeName = (m_params.z_mode == ZMode::Planar) ? "planar(2D,z恒0)" : "3d(z由pitch积分)";

        // 总里程发布（geometry_msgs/PointStamped，字段可选 x/y/z，默认 x）
        m_mileageTopic = declare_parameter<std::string>("mileage_topic", "/total_mileage");
        m_mileageField = declare_parameter<std::string>("mileage_field", "x");

        // 日志文件（空 = 默认 <工程根>/log/dr.text）
        m_logFile = declare_parameter<std::string>("log_file", "");

        // ★车速正负：chassis_info.speed_mps 是【幅值】(实测恒 ≥0)，倒挡要取负
        //   口径同 hg-slam external_data_node.cpp: if (gear == GEAR_R) speed = -speed;
        //   GEAR 编码（common_msgs/msg/Enum）: N=1, D=2, R=3, P=4
        m_gearReverseFix   = declare_parameter<bool>("gear_reverse_fix", true);
        m_reverseGearValue = declare_parameter<int>("reverse_gear_value", 3);

        // IMU + 车速【同时】超时 ⇒ 停止递推、TF 位置归零
        m_imuTimeoutS = declare_parameter<double>("imu_timeout_s", 0.3);
        m_pauseOnBothTimeout = declare_parameter<bool>("pause_on_both_timeout", true);
    }

    // ---------------- IMU ----------------
    void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
    {
        m_lastImuRecvS = wall_s();
        const int64_t t_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
                             static_cast<int64_t>(msg->header.stamp.nanosec);
        const Eigen::Vector3d acc(msg->linear_acceleration.x,
                                  msg->linear_acceleration.y,
                                  msg->linear_acceleration.z);
        const Eigen::Vector3d gyr(msg->angular_velocity.x,
                                  msg->angular_velocity.y,
                                  msg->angular_velocity.z);

        if (!m_imuFrameLogged)
        {
            m_imuFrameLogged = true;
            RCLCPP_INFO(get_logger(), "[IMU] 首帧 frame_id='%s' stamp=%.6f acc=(%.4f, %.4f, %.4f) |a|=%.4f "
                        "gyr=(%.5f, %.5f, %.5f)",
                        msg->header.frame_id.c_str(), t_ns * 1e-9,
                        acc.x(), acc.y(), acc.z(), acc.norm(), gyr.x(), gyr.y(), gyr.z());
            if (msg->header.frame_id.empty())
                RCLCPP_WARN(get_logger(), "[IMU] frame_id 为空，建议在驱动侧命名（本节点不依赖 frame_id）");
        }

        DrPose out;
        const bool had_pose = m_engine.inited();
        const bool updated  = m_engine.updateImu(t_ns, acc, gyr, out);
        if (!had_pose && updated)
        {
            const auto &s = m_engine.stats();
            RCLCPP_INFO(get_logger(), "[初始化] 静态窗口完成: 样本=%ld 跨度=%.2fs 陀螺零偏=(%.6f, %.6f, %.6f)rad/s "
                        "静止比力=(%.4f, %.4f, %.4f) |a|=%.5f (g=%.5f) ⇒ 初始 roll=%.3fdeg pitch=%.3fdeg yaw0=%.2fdeg",
                        s.imu_cnt, m_params.init_static_s,
                        m_engine.gyroBias().x(), m_engine.gyroBias().y(), m_engine.gyroBias().z(),
                        m_engine.initAccMean().x(), m_engine.initAccMean().y(), m_engine.initAccMean().z(),
                        m_engine.initAccMean().norm(), m_engine.gravity(),
                        out.rpy.x() * 180.0 / M_PI, out.rpy.y() * 180.0 / M_PI, out.rpy.z() * 180.0 / M_PI);
            m_initDoneTime = now_s();
            m_lastPose = out;
        }
        if (updated)
            m_lastPose = out;

        // 静态初始化超时告警（例如车在动 / 数据不稳导致迟迟不收敛）
        if (!m_engine.inited() && m_initWarned == 0 && m_firstImuTime > 0.0 &&
            (now_s() - m_firstImuTime) > (3.0 * m_params.init_static_s))
        {
            m_initWarned = 1;
            RCLCPP_WARN(get_logger(), "[初始化] 已过 %.1fs 仍未完成静止初始化（需要 %.1fs 静止数据）；"
                        "若此时车辆在动属正常，但初始姿态/零偏精度会下降",
                        3.0 * m_params.init_static_s, m_params.init_static_s);
        }
        if (m_firstImuTime <= 0.0)
            m_firstImuTime = now_s();
    }

    // ---------------- 车速 ----------------
    void onSpeed(const chassis_msgs::msg::ChassisInfo::ConstSharedPtr &msg)
    {
        m_lastSpeedRecvS = wall_s();

        // ★车速正负值归一（借鉴 hg-slam external_data_node.cpp procChassisInfoArrive）：
        //   speed_mps 是幅值（实测恒 ≥0，倒车时 gear==R 而 speed 仍为正），
        //   ⇒ 倒挡取负，否则纯 DR 会朝前推、位姿与里程全错。
        double v = static_cast<double>(msg->speed_mps);
        m_isReverse = false;
        if (m_gearReverseFix && static_cast<int>(msg->gear) == m_reverseGearValue)
        {
            v = -v;
            m_isReverse = (std::fabs(v) > 1e-3);
            if (!m_reverseLogged && std::fabs(v) > 1e-3)
            {
                m_reverseLogged = true;
                RCLCPP_INFO(get_logger(), "[车速] 检测到倒挡 gear=%u ⇒ 速度取负 %.3f m/s（DR 后退）",
                            static_cast<unsigned>(msg->gear), v);
            }
        }
        m_engine.updateSpeed(v);
        if (!m_speedFrameLogged)
        {
            m_speedFrameLogged = true;
            const bool stampZero = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0);
            RCLCPP_INFO(get_logger(), "[车速] 首帧 speed_mps=%.4f  header.stamp=%s",
                        msg->speed_mps, stampZero ? "0(无时间戳,已改用 IMU 时间对齐)" : "有效");
        }
    }

    // ---------------- 周期发布 ----------------
    void onPublishTimer()
    {
        if (!m_engine.inited() || !m_lastPose.valid)
            return;

        // ---- IMU 与 车速【同时】超时 ⇒ 停止递推，输出位置归零 ----
        if (m_pauseOnBothTimeout && m_lastImuRecvS > 0.0 && m_lastSpeedRecvS > 0.0)
        {
            const double nowWall = wall_s();
            const double imuAge = nowWall - m_lastImuRecvS;
            const double spdAge = nowWall - m_lastSpeedRecvS;
            const bool imuStale = (imuAge > m_imuTimeoutS);
            const bool spdStale = (spdAge > m_params.speed_timeout_s);
            const bool bothTimeout = imuStale && spdStale;
            const bool bothFresh   = !imuStale && !spdStale;
            if (bothTimeout && !m_paused)
            {
                m_paused = true;
                m_engine.setPoseIntegrationEnabled(false);   // 停止递推（姿态仍跟踪）
                m_engine.resetPositionToOrigin();            // 积分状态清到原点
                m_lastPose = m_engine.pose();
                RCLCPP_WARN(get_logger(),
                            "[中断] IMU(%.3fs) 与 车速(%.3fs) 同时超时（门限 %.3fs/%.3fs）⇒ 停止递推，"
                            "TF 位置输出 (0,0,0)，里程保持 %.2fm",
                            imuAge, spdAge, m_imuTimeoutS, m_params.speed_timeout_s,
                            m_engine.traveledDistance());
            }
            else if (bothFresh && m_paused)
            {
                m_paused = false;
                m_engine.setPoseIntegrationEnabled(true);    // 恢复递推（从原点重新累计）
                RCLCPP_INFO(get_logger(),
                            "[恢复] 数据恢复（IMU 年龄 %.3fs / 车速年龄 %.3fs）⇒ 继续递推，位置从 (0,0,0) 重新累计",
                            imuAge, spdAge);
            }
        }

        // 暂停期间：位置强制为 0（姿态沿用最后一次估计）
        const Eigen::Vector3d pos = m_paused ? Eigen::Vector3d::Zero() : m_lastPose.pos;

        const rclcpp::Time stamp(m_lastPose.t_ns, RCL_ROS_TIME);
        const Eigen::Vector3d &rpy = m_lastPose.rpy;

        // 四元数（ZYX）
        Eigen::Quaterniond q =
            Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX());
        q.normalize();

        // ---- /tf: parent -> child ----
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = m_parentFrame;
        tf.child_frame_id  = m_childFrame;
        tf.transform.translation.x = pos.x();
        tf.transform.translation.y = pos.y();
        tf.transform.translation.z = pos.z();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        tf2_msgs::msg::TFMessage tfMsg;
        tfMsg.transforms.push_back(tf);
        m_pubTf->publish(tfMsg);
        if (!m_tfLogged)
        {
            m_tfLogged = true;
            RCLCPP_INFO(get_logger(), "[TF] 首次发布 %s -> %s: pos=(%.3f, %.3f, %.3f) rpy=(%.2f, %.2f, %.2f)deg",
                        m_parentFrame.c_str(), m_childFrame.c_str(),
                        pos.x(), pos.y(), pos.z(),
                        rpy.x() * 180.0 / M_PI, rpy.y() * 180.0 / M_PI, rpy.z() * 180.0 / M_PI);
        }

        // ---- /dr_odom ----
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = m_parentFrame;
        odom.child_frame_id  = m_childFrame;
        odom.pose.pose.position.x = pos.x();
        odom.pose.pose.position.y = pos.y();
        odom.pose.pose.position.z = pos.z();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        // 车体系速度（twist 按惯例在 child frame）
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Vector3d vBody = R.transpose() * (m_paused ? Eigen::Vector3d::Zero() : m_lastPose.vel);
        odom.twist.twist.linear.x = vBody.x();
        odom.twist.twist.linear.y = vBody.y();
        odom.twist.twist.linear.z = vBody.z();
        for (int i = 0; i < 36; ++i)
        {
            odom.pose.covariance[i]  = 0.0;
            odom.twist.covariance[i] = 0.0;
        }
        odom.pose.covariance[0] = odom.pose.covariance[7] = odom.pose.covariance[14] = 0.01;
        odom.pose.covariance[21] = odom.pose.covariance[28] = odom.pose.covariance[35] = 0.001;
        m_pubOdom->publish(odom);

        // ---- /dr_path ----
        const double nowS = stamp.seconds();
        if (!m_paused &&
            (m_path.header.stamp.sec == 0 || (nowS - m_pathLastAppendS) >= (1.0 / std::max(0.1, m_pathAppendHz))))
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = odom.header;
            ps.pose = odom.pose.pose;
            m_path.poses.push_back(ps);
            m_path.header.stamp = stamp;
            m_pathLastAppendS = nowS;
            if (m_pathMaxPoses > 0 && static_cast<int>(m_path.poses.size()) > m_pathMaxPoses)
                m_path.poses.erase(m_path.poses.begin(),
                                   m_path.poses.begin() + (m_path.poses.size() - m_pathMaxPoses));
        }
        // /dr_path 的发布已移到 onPathTimer()（默认 1 Hz）：整条轨迹数组体积大，
        // 不能跟着这里的高频定时器一起重发（否则 50 Hz × MB 级 = 数十 MB/s）。
        // 此处只负责【追加采样点】。

        // ---- /total_mileage: 总里程（geometry_msgs/PointStamped，字段由 mileage_field 指定）----
        geometry_msgs::msg::PointStamped ms;
        ms.header.stamp = stamp;
        ms.header.frame_id = m_parentFrame;
        ms.point.x = 0.0; ms.point.y = 0.0; ms.point.z = 0.0;
        const double mileage = m_engine.traveledDistance();
        if (m_mileageField == "y")      ms.point.y = mileage;
        else if (m_mileageField == "z") ms.point.z = mileage;
        else                            ms.point.x = mileage;
        m_pubMileage->publish(ms);
    }

    // ---------------- /dr_path 低频发布（默认 1 Hz） ----------------
    // 轨迹数组随运行时长增长（每 path_append_hz 追加一点），单条消息可达 MB 级，
    // 因此与 /tf、/dr_odom、/total_mileage 的高频发布解耦。
    void onPathTimer()
    {
        if (!m_engine.inited() || !m_lastPose.valid)
            return;
        if (m_path.header.stamp.sec == 0)      // 尚未追加过任何轨迹点
            return;
        m_pubPath->publish(m_path);
    }

    // ---------------- 周期日志 ----------------
    void onLogTimer()
    {
        const auto &s = m_engine.stats();
        const double nowS = now_s();
        const double dt_mean = (s.dt_n > 0) ? (s.dt_sum / static_cast<double>(s.dt_n)) : 0.0;
        const double dt_rate = (dt_mean > 0.0) ? (1.0 / dt_mean) : 0.0;

        if (!m_engine.inited())
        {
            RCLCPP_INFO(get_logger(), "[日志] 初始化中… IMU=%ld 帧(%.1fHz, dt均值=%.1fms) 车速=%ld 帧 "
                        "已静 %.1fs(需 %.1fs) 累计 |a| 采样中",
                        s.imu_cnt, dt_rate, dt_mean * 1e3, s.speed_cnt,
                        (m_firstImuTime > 0.0) ? (nowS - m_firstImuTime) : 0.0,
                        m_params.init_static_s);
            m_engine.resetRateStats();
            return;
        }

        const auto &p = m_engine.pose();
        RCLCPP_INFO(get_logger(),
                    "[日志] pos=(%8.3f,%8.3f,%8.3f)m v=(%6.3f,%6.3f,%6.3f)m/s speed=%.3fm/s "
                    "rpy=(%7.2f,%6.2f,%7.2f)deg 里程=%.2fm | IMU %ld 帧 %.1fHz dt_max=%.1fms "
                    "断流=%ld 回跳=%ld 车速过期=%ld 车速年龄=%.3fs 倒挡=%d%s",
                    p.pos.x(), p.pos.y(), p.pos.z(), p.vel.x(), p.vel.y(), p.vel.z(), p.speed,
                    p.rpy.x() * 180.0 / M_PI, p.rpy.y() * 180.0 / M_PI, p.rpy.z() * 180.0 / M_PI,
                    m_engine.traveledDistance(), s.imu_cnt, dt_rate, s.dt_max * 1e3,
                    s.imu_gap_cnt, s.imu_nonmono_cnt, s.speed_stale_cnt, s.speed_age,
                    static_cast<int>(m_isReverse),
                    m_paused ? "  [已中断:IMU+车速同时超时,位置归零]" : "");
        m_engine.resetRateStats();

        // 车速长期过期/长期缺失告警（每秒最多一条，避免刷屏）
        if (s.speed_cnt == 0 && m_speedWarned++ < 5)
            RCLCPP_WARN(get_logger(), "[车速] 尚未收到 %s —— 速度恒为 0（原地不动，位置不会推进）",
                        m_speedTopic.c_str());
        else if (s.speed_cnt > 0 && s.speed_age > m_params.speed_timeout_s && m_speedStaleWarned++ < 5)
            RCLCPP_WARN(get_logger(), "[车速] 已 %.3fs 无更新（门限 %.3fs）⇒ %s",
                        s.speed_age, m_params.speed_timeout_s,
                        m_params.hold_last_speed ? "保持上次速度" : "速度置零");
    }

    double now_s() const
    {
        return this->now().seconds();      // ROS 时间（受 use_sim_time 影响，用于日志/时间戳）
    }

    /// 墙钟(单调)秒：用于数据超时判定。
    /// ★不能用 ROS 时间——回放时 use_sim_time=true，数据一断 /clock 就停，
    ///   用 ROS 时间算年龄会永远小于门限，中断永远触发不了。
    static double wall_s()
    {
        using namespace std::chrono;
        return duration<double>(steady_clock::now().time_since_epoch()).count();
    }

    // ---- 参数 ----
    std::string m_imuTopic, m_speedTopic, m_odomTopic, m_pathTopic, m_parentFrame, m_childFrame;
    std::string m_mileageTopic = "/total_mileage", m_mileageField = "x", m_zModeName = "3d";
    std::string m_logFile;   // 空 ⇒ LogFileSink 用 <工程根>/log/dr.text
    double m_imuTimeoutS = 0.3;        // IMU 超时门限
    bool   m_pauseOnBothTimeout = true;// IMU+车速同时超时 ⇒ 停止递推、位置归零
    double m_lastImuRecvS = -1.0;      // 最近一次 IMU 到达（节点时钟 s）
    bool   m_gearReverseFix = true;    // 倒挡取负（speed_mps 是幅值）
    int    m_reverseGearValue = 3;     // GEAR_R=3 (common_msgs Enum: N=1 D=2 R=3 P=4)
    bool   m_isReverse = false;        // 当前是否倒挡（日志用）
    bool   m_reverseLogged = false;    // 首次检测到倒挡打印一次
    double m_lastSpeedRecvS = -1.0;    // 最近一次车速到达
    bool   m_paused = false;           // 当前是否处于"数据全丢"中断态
    double m_publishRateHz = 50.0, m_pathPublishHz = 1.0, m_pathAppendHz = 10.0, m_logPeriodS = 1.0;
    int m_pathMaxPoses = 200000, m_qosDepth = 50;
    bool m_qosReliable = false;
    DrParams m_params;

    // ---- 引擎与状态 ----
    DrEngine m_engine;
    DrPose m_lastPose;
    nav_msgs::msg::Path m_path;
    double m_pathLastAppendS = 0.0;
    double m_firstImuTime = 0.0, m_initDoneTime = 0.0;
    bool m_imuFrameLogged = false, m_speedFrameLogged = false, m_tfLogged = false;
    int  m_initWarned = 0, m_speedWarned = 0, m_speedStaleWarned = 0;

    // ---- ROS 实体 ----
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_subImu;
    rclcpp::Subscription<chassis_msgs::msg::ChassisInfo>::SharedPtr m_subSpeed;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr m_pubTf;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_pubOdom;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_pubPath;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_pubMileage;
    rclcpp::TimerBase::SharedPtr m_timerPub, m_timerPath, m_timerLog;
};

} // namespace dr_engine

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    {
        auto node = std::make_shared<dr_engine::DrNode>();
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    dr_engine::LogFileSink::shutdown();   // 关闭文件日志（写"退出"分隔行）
    return 0;
}
