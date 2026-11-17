// ============================================================================
//  dr_engine.cpp —— 纯 DR 引擎实现（详见 dr_engine.h 的口径说明）
// ============================================================================
#include "dr_engine/dr_engine.h"

#include <cmath>
#include <cstring>

namespace dr_engine
{

namespace
{
constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kDeg2Rad = M_PI / 180.0;

/// 四元数(z,w,x,y) -> ZYX 欧拉角 roll/pitch/yaw (rad)，与 tf::Matrix3x3::getRPY 同口径
Eigen::Vector3d quatToRpy(double w, double x, double y, double z)
{
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    if (n < 1e-9)
        return Eigen::Vector3d::Zero();
    w /= n; x /= n; y /= n; z /= n;

    const double roll  = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    double sinp        = 2.0 * (w * y - z * x);
    sinp               = std::max(-1.0, std::min(1.0, sinp));
    const double pitch = std::asin(sinp);
    const double yaw   = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return Eigen::Vector3d(roll, pitch, yaw);
}
} // namespace

void DrEngine::init(const DrParams &p)
{
    m_p = p;
    m_yaw0 = p.yaw0_deg * kDeg2Rad;

    m_inited = false;
    m_tFirst = 0;
    m_initN = 0;
    m_accSum.setZero();
    m_gyrSum.setZero();
    m_bg.setZero();
    m_initAcc.setZero();
    m_g = (p.g > 1.0) ? p.g : 9.80665;

    m_lastImuT = 0;
    m_hasSpeed = false;
    m_speed = 0.0;
    m_speedT = -1.0;
    m_pose = DrPose();
    m_stats = DrStats();
    m_distance = 0.0;
}

void DrEngine::updateSpeed(double speed_mps)
{
    ++m_stats.speed_cnt;
    m_hasSpeed = true;
    m_speed = m_p.speed_scale * speed_mps;
    if (m_p.use_abs_speed)
        m_speed = std::fabs(m_speed);
    // 时间基准统一到 IMU 时间轴：用"收到车速那一刻最新的 IMU 时间"
    if (m_lastImuT > 0)
        m_speedT = m_lastImuT * 1e-9;
}

void DrEngine::resetPositionToOrigin()
{
    m_pose.pos.setZero();
    m_pose.vel.setZero();
}

bool DrEngine::tryInitStatic(int64_t t_ns, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
{
    if (m_initN == 0)
        m_tFirst = t_ns;
    m_accSum += acc;
    m_gyrSum += gyr;
    ++m_initN;

    const double span = static_cast<double>(t_ns - m_tFirst) * 1e-9;
    if (span < m_p.init_static_s || m_initN < m_p.init_min_samples)
        return false;

    const double n = static_cast<double>(m_initN);
    m_bg      = m_gyrSum / n;                  // 陀螺零偏
    m_initAcc = m_accSum / n;                  // 静止平均比力（含重力，指向"上"）
    const double g_meas = m_initAcc.norm();
    m_g = (m_p.g_auto && g_meas > 5.0) ? g_meas : ((m_p.g > 1.0) ? m_p.g : 9.80665);

    // 用静止比力给 AHRS 一个初值（roll/pitch 直接由重力方向定）
    const double roll0  = std::atan2(m_initAcc[1], m_initAcc[2]);
    const double pitch0 = std::atan2(-m_initAcc[0], std::hypot(m_initAcc[1], m_initAcc[2]));

    const double dt_mean = (m_initN > 1) ? span / (m_initN - 1) : 0.01;
    const unsigned int rate = static_cast<unsigned int>(std::max(1.0, std::min(1000.0, 1.0 / dt_mean)));

    FusionAhrsInitialise(m_ahrs);
    FusionAhrsSettings settings;
    std::memset(&settings, 0, sizeof(settings));
    settings.gain                 = static_cast<float>(m_p.ahrs_gain);
    settings.accelerationRejection = static_cast<float>(m_p.ahrs_acc_reject_deg);
    settings.magneticRejection     = static_cast<float>(m_p.ahrs_mag_reject_deg);
    settings.rejectionTimeout      = static_cast<unsigned int>(m_p.ahrs_reject_timeout_s * rate);
    FusionAhrsSetSettings(m_ahrs, settings);

    FusionOffsetInitialise(&m_offset, rate);

    // 姿态与位姿初值
    m_pose = DrPose();
    m_pose.t_ns = t_ns;
    m_pose.rpy = Eigen::Vector3d(roll0, pitch0, m_yaw0);
    m_pose.pos.setZero();
    m_pose.vel.setZero();
    m_pose.speed = 0.0;
    m_pose.valid = true;

    m_lastImuT = t_ns;
    m_inited = true;
    return true;
}

bool DrEngine::updateImu(int64_t t_ns, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr, DrPose &out)
{
    ++m_stats.imu_cnt;

    // ---- dt 记账 + 时间戳健壮性（含初始化阶段，便于日志里看真实 IMU 频率）----
    double dt = 0.0;
    if (m_lastImuT > 0)
    {
        dt = static_cast<double>(t_ns - m_lastImuT) * 1e-9;
        if (dt <= 1e-6)
        {
            ++m_stats.imu_nonmono_cnt;   // 时间戳回跳/重复：重同步，本帧不积分
            m_lastImuT = t_ns;
            ++m_stats.pose_skip_cnt;
            return false;
        }
        if (dt > m_p.max_gap_s)
        {
            ++m_stats.imu_gap_cnt;       // 断流：重同步，本帧不积分
            m_lastImuT = t_ns;
            ++m_stats.pose_skip_cnt;
            return false;
        }
        m_stats.dt_last = dt;
        m_stats.dt_sum += dt;
        m_stats.dt_max = std::max(m_stats.dt_max, dt);
        ++m_stats.dt_n;
    }
    m_lastImuT = t_ns;

    if (!m_inited)
    {
        if (!tryInitStatic(t_ns, acc, gyr))
            return false;
        out = m_pose;
        return true;
    }

    // ---- 姿态：FusionAhrs（6 轴） ----
    //   加速度计只用于姿态（AHRS 内部会归一化，故无需减重力/加速度零偏）；
    //   陀螺仪减静止零偏，再可选经 FusionOffset 做零偏在线跟随。
    const Eigen::Vector3d gyr_b = gyr - m_bg;

    FusionVector gyroscope = {static_cast<float>(gyr_b[0] * kRad2Deg),
                              static_cast<float>(gyr_b[1] * kRad2Deg),
                              static_cast<float>(gyr_b[2] * kRad2Deg)};
    FusionVector accelerometer = {static_cast<float>(acc[0] / m_g),
                                  static_cast<float>(acc[1] / m_g),
                                  static_cast<float>(acc[2] / m_g)};
    if (m_p.use_ahrs_offset)
        gyroscope = FusionOffsetUpdate(&m_offset, gyroscope);

    FusionAhrsUpdateNoMagnetometer(m_ahrs, gyroscope, accelerometer, dt);

    const FusionAhrsFlags flags = FusionAhrsGetFlags(m_ahrs);
    if (flags.initialising)
    {
        ++m_stats.ahrs_init_cnt;
        ++m_stats.pose_skip_cnt;
        return false;
    }

    const FusionQuaternion q = FusionAhrsGetQuaternion(m_ahrs);
    Eigen::Vector3d rpy = quatToRpy(q.element.w, q.element.x, q.element.y, q.element.z);
    rpy[2] = m_yaw0 + rpy[2];
    while (rpy[2] >  M_PI) rpy[2] -= 2.0 * M_PI;
    while (rpy[2] < -M_PI) rpy[2] += 2.0 * M_PI;

    // ---- 速度：大小来自底盘车速，方向来自姿态 ----
    double speed = 0.0;
    const double now_s = static_cast<double>(t_ns) * 1e-9;
    m_stats.speed_age = (m_speedT > 0.0) ? (now_s - m_speedT) : 1e9;
    if (!m_hasSpeed)
    {
        speed = 0.0;   // 还没有车速：静止假设
    }
    else if (m_stats.speed_age > m_p.speed_timeout_s)
    {
        ++m_stats.speed_stale_cnt;
        speed = m_p.hold_last_speed ? m_speed : 0.0;
    }
    else
    {
        speed = m_speed;
    }

    // 停止递推：只更新姿态与车速观测，位置/速度/里程保持不动
    if (!m_integrate)
    {
        m_pose.vel.setZero();
        m_pose.pos.setZero();
        m_pose.rpy = rpy;
        m_pose.speed = speed;
        m_pose.t_ns = t_ns;
        m_pose.valid = true;
        out = m_pose;
        return true;
    }

    const double cp = std::cos(rpy[1]), sp = std::sin(rpy[1]);
    const double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
    Eigen::Vector3d vel(speed * cp * cy, speed * cp * sy, speed * sp);
    if (ZMode::Planar == m_p.z_mode)
        vel[2] = 0.0;   // planar: 不做高程积分（z 恒 0），pitch 仍照常在 rpy 里输出

    const Eigen::Vector3d dpos = vel * dt;
    m_pose.pos += dpos;
    m_pose.vel = vel;
    m_pose.rpy = rpy;
    m_pose.speed = speed;
    m_pose.t_ns = t_ns;
    if (ZMode::Planar == m_p.z_mode)
        m_pose.pos[2] = 0.0;
    m_pose.valid = true;
    m_distance += dpos.norm();

    out = m_pose;
    return true;
}

} // namespace dr_engine
