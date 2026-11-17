// ============================================================================
//  log_sink.h —— 把 ROS 日志同时写一份到文件（默认 <工程根>/log/dr.text）
//
//  设计:
//    · 用 rcutils 的全局 output handler 钩子：链式保留原 handler（控制台照旧输出），
//      再追加写文件 ⇒ 不改任何 LOG 调用点，控制台与文件内容一致；
//    · 线程安全（内部 mutex），每行 flush，进程被杀也不丢日志；
//    · 目录不存在时自动创建；文件以【追加】方式打开，多次运行不会互相覆盖。
// ============================================================================
#pragma once

#include <string>

namespace dr_engine
{

class LogFileSink
{
public:
    /// 打开日志文件（追加）。path 为空 ⇒ 用默认 <工程根>/log/dr.text。
    /// 成功返回 true；失败时 err 里带原因（调用方通常只打一条 WARN 继续跑）。
    static bool init(const std::string &path, std::string &err);

    /// 卸载 handler 并关闭文件（进程退出前调用）
    static void shutdown();

    /// 当前实际使用的日志文件绝对路径（未开启时为空）
    static const std::string &path();

    /// 默认日志路径：从 /proc/self/exe 反推工程根目录（<proj>/build/xxx ⇒ <proj>），
    /// 反推失败则退回当前工作目录。
    static std::string defaultPath();

private:
    static void install();
};

} // namespace dr_engine
