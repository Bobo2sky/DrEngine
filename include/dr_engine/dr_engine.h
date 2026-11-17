// ============================================================================
//  dr_engine.h —— 纯 DR（Dead Reckoning）引擎，不依赖 ROS
//
//  口径与 hg-slam `src/asyn_proc/dead_reckoning_proc.cpp` 一致：
//    1) 姿态：FusionAhrs（Madgwick，6 轴，无磁力计）融合加速度计 + 陀螺仪；
//    2) 速度：**大小来自底盘车速 (chassis_info.speed_mps)**，方向由姿态投影到世界系：
//         v = speed * (cos(pitch)cos(yaw), cos(pitch)sin(yaw), sin(pitch))
//    3) 位置：只对速度积分  pos += v * dt
//    加速度**不参与位置积分** —— 这是该 DR 能长时间不漂的关键（纯惯性二重积分几十秒就会发散）。
//
//  纯 DR 的固有限制：无绝对航向（无磁力计/无 GNSS），yaw 只能由陀螺积分 + 初始基准给出。
// ============================================================================
#pragma once

#include <cstdint>
#include <Eigen/Dense>

#include "madgwick/FusionAhrs.h"
#include "madgwick/FusionOffset.h"

namespace dr_engine
{

/// 位置解算模式
enum class ZMode
{
    Mode3D = 0,   ///< 3D：z 由 pitch 积分（z = ∫v·sin(pitch)·dt），坡道可用
    Planar = 1    ///< 2D/planar：z 恒为 0，不参与积分（地面车辆平面作业，避免 pitch 偏差积成米级高程漂）
};

struct DrParams
{
    double g = 9.80665;            ///< 重力常数；g_auto=true 时用静止实测 |a| 覆盖
    bool   g_auto = true;          ///< true = 用静止窗口实测的 |a| 作为 g（更贴合本传感器标度）
    double init_static_s = 2.0;    ///< 静止初始化窗口长度（按 IMU 时间）
    int    init_min_samples = 50;  ///< 窗口内最少样本数
    double yaw0_deg = 0.0;         ///< 初始航向基准（纯 DR 无绝对航向，由用户给定）

    double ahrs_gain = 0.5;            ///< Madgwick 比例增益
    double ahrs_acc_reject_deg = 10.0; ///< 加速度计拒绝阈值（度）；越大越依赖陀螺
    double ahrs_mag_reject_deg = 0.0;  ///< 无磁力计，保持 0
    double ahrs_reject_timeout_s = 5.0;///< 拒绝超时（完全依赖陀螺的最长时间）
    bool   use_ahrs_offset = true;     ///< 陀螺零偏在线估计（FusionOffset）

    double speed_scale = 1.0;      ///< 车速标度修正
    double speed_timeout_s = 0.3;  ///< 车速过期门限（超时按 hold_last_speed 处理）
    bool   hold_last_speed = true; ///< true=保持上次速度；false=置零
    bool   use_abs_speed = false;  ///< true=只用速度绝对值（倒车也当作前进）

    double max_gap_s = 0.2;        ///< IMU 间隔超过此值视为断流：不积分，只重同步
    ZMode  z_mode = ZMode::Mode3D; ///< 位置解算模式（3d / planar）
};

struct DrPose
{
    int64_t t_ns = 0;                          ///< 本帧 IMU 时间戳（ns）
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();  ///< 世界系位置 (m)
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();  ///< 世界系速度 (m/s)
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();  ///< roll/pitch/yaw (rad)
    double speed = 0.0;                        ///< 车体系前向速度 (m/s)
    bool valid = false;                        ///< 是否已在输出有效位姿（静态初始化完成）
};

struct DrStats
{
    long imu_cnt = 0;
    long imu_gap_cnt = 0;       ///< IMU 间隔过大（断流）
    long imu_nonmono_cnt = 0;   ///< 时间戳非单调
    long speed_cnt = 0;
    long speed_stale_cnt = 0;   ///< 车速过期仍在使用
    long pose_skip_cnt = 0;     ///< 未积分帧数
    long ahrs_init_cnt = 0;     ///< AHRS 收敛中（initialising）帧数
    double dt_last = 0.0;
    double dt_sum = 0.0;
    double dt_max = 0.0;
    long   dt_n = 0;
    double speed_age = 0.0;     ///< 最近一次积分时车速的"年龄"(s)
};

class DrEngine
{
public:
    void init(const DrParams &p);

    /// 喂一帧 IMU。返回 true 表示本次产出了新的有效位姿（可发布）。
    bool updateImu(int64_t t_ns, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr, DrPose &out);

    /// 喂一次底盘车速（时间基准自动对齐到最近一次 IMU 时间，避免混用墙钟/仿真钟）
    void updateSpeed(double speed_mps);

    const DrPose  &pose()  const { return m_pose; }
    const DrStats &stats() const { return m_stats; }
    bool inited() const { return m_inited; }

    double gravity() const { return m_g; }
    const Eigen::Vector3d &gyroBias() const { return m_bg; }
    const Eigen::Vector3d &initAccMean() const { return m_initAcc; }
    double traveledDistance() const { return m_distance; }

    /// 位置/速度归零（姿态、零偏、总里程保留）。
    /// 用于"IMU 与车速同时超时 ⇒ 停止递推"：把积分状态清到原点，
    /// 数据恢复后从原点重新累计，避免 TF 位置跳变。
    void resetPositionToOrigin();

    /// 位置积分总开关（false = "停止递推"）：姿态/零偏照常更新，位置与速度保持 0、里程不再累加。
    /// 用于 IMU 与车速同时超时的中断态；恢复后置回 true 从原点继续。
    void setPoseIntegrationEnabled(bool on) { m_integrate = on; }
    bool poseIntegrationEnabled() const { return m_integrate; }
    void resetRateStats() { m_stats.dt_sum = 0.0; m_stats.dt_n = 0; m_stats.dt_max = 0.0; }

private:
    bool tryInitStatic(int64_t t_ns, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr);

    DrParams m_p;
    FusionAhrs   m_ahrs{};
    FusionOffset m_offset{};

    bool m_inited = false;
    bool m_integrate = true;   ///< 位置积分开关（false = 停止递推）

    // 静止初始化累计
    int64_t m_tFirst = 0;
    int     m_initN = 0;
    Eigen::Vector3d m_accSum = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_gyrSum = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_bg = Eigen::Vector3d::Zero();       ///< 陀螺零偏
    Eigen::Vector3d m_initAcc = Eigen::Vector3d::Zero();  ///< 静止窗口平均比力
    double m_g = 9.80665;
    double m_yaw0 = 0.0;

    // 运行状态
    int64_t m_lastImuT = 0;
    bool    m_hasSpeed = false;
    double  m_speed = 0.0;
    double  m_speedT = -1.0;   ///< 最近一次车速对应的 IMU 时间 (s)
    DrPose  m_pose;
    DrStats m_stats;
    double  m_distance = 0.0;
};

} // namespace dr_engine
