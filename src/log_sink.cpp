// ============================================================================
//  log_sink.cpp —— 见 log_sink.h 的说明
// ============================================================================
#include "dr_engine/log_sink.h"

#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <mutex>
#include <fstream>
#include <string>
#include <vector>

#include <unistd.h>
#include <limits.h>

#include <rcutils/logging.h>

namespace dr_engine
{

namespace
{
std::mutex                       g_mutex;
std::ofstream                    g_file;
std::string                      g_path;
rcutils_logging_output_handler_t g_prev_handler = nullptr;
bool                             g_installed = false;

const char *severityName(int severity)
{
    switch (severity)
    {
        case RCUTILS_LOG_SEVERITY_DEBUG: return "DEBUG";
        case RCUTILS_LOG_SEVERITY_INFO:  return "INFO ";
        case RCUTILS_LOG_SEVERITY_WARN:  return "WARN ";
        case RCUTILS_LOG_SEVERITY_ERROR: return "ERROR";
        case RCUTILS_LOG_SEVERITY_FATAL: return "FATAL";
        default:                         return "UNKWN";
    }
}

/// 链式 handler：先写文件，再交给原 handler（控制台照旧）
void fileOutputHandler(const rcutils_log_location_t *location,
                       int severity,
                       const char *name,
                       rcutils_time_point_value_t timestamp,
                       const char *format,
                       va_list *args)
{
    // ---- 1) 写文件 ----
    {
        std::lock_guard<std::mutex> lk(g_mutex);
        if (g_file.is_open())
        {
            // va_list 只能消费一次 ⇒ 先拷一份给文件用，原件留给控制台
            va_list args_copy;
            va_copy(args_copy, *args);
            char body[2048];
            std::vsnprintf(body, sizeof(body), format, args_copy);
            va_end(args_copy);

            // 时间戳（ns since epoch）→ 本地时间
            const time_t sec = static_cast<time_t>(timestamp / 1000000000LL);
            const long   nsec = static_cast<long>(timestamp % 1000000000LL);
            struct tm tm_local
            {
            };
            localtime_r(&sec, &tm_local);
            char tbuf[32];
            std::strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &tm_local);

            g_file << '[' << tbuf << '.' << std::to_string(nsec / 1000 + 1000000).substr(1) << "] "
                   << '[' << severityName(severity) << "] "
                   << '[' << (name ? name : "-") << "]: " << body;
            if (location && location->function_name)
                g_file << " (" << (location->file_name ? location->file_name : "?")
                       << ':' << location->line_number << ')';
            g_file << '\n';
            g_file.flush();
        }
    }

    // ---- 2) 交回原 handler（控制台）----
    if (g_prev_handler)
        g_prev_handler(location, severity, name, timestamp, format, args);
}
} // namespace

bool LogFileSink::init(const std::string &path, std::string &err)
{
    {
        std::lock_guard<std::mutex> lk(g_mutex);
        if (g_file.is_open())
        {
            err = "日志文件已打开: " + g_path;
            return false;
        }
    }

    const std::string target = path.empty() ? defaultPath() : path;

    // 目录（不存在则创建）
    const std::string::size_type pos = target.find_last_of('/');
    if (pos != std::string::npos)
    {
        const std::string dir = target.substr(0, pos);
        std::string cmd = "mkdir -p '" + dir + "'";
        if (std::system(cmd.c_str()) != 0)
        {
            err = "创建日志目录失败: " + dir;
            return false;
        }
    }

    std::lock_guard<std::mutex> lk(g_mutex);
    g_file.open(target, std::ios::out | std::ios::app);
    if (!g_file.is_open())
    {
        err = "无法打开日志文件: " + target;
        return false;
    }
    g_path = target;
    g_file << "\n==================== DrEngine 启动 ====================\n" << std::flush;
    g_file.flush();
    install();
    return true;
}

void LogFileSink::install()
{
    if (g_installed)
        return;
    g_prev_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(&fileOutputHandler);
    g_installed = true;
}

void LogFileSink::shutdown()
{
    std::lock_guard<std::mutex> lk(g_mutex);
    if (g_installed)
    {
        rcutils_logging_set_output_handler(g_prev_handler ? g_prev_handler
                                                          : rcutils_logging_get_output_handler());
        g_installed = false;
    }
    if (g_file.is_open())
    {
        g_file << "==================== DrEngine 退出 ====================\n";
        g_file.flush();
        g_file.close();
    }
}

const std::string &LogFileSink::path()
{
    return g_path;
}

std::string LogFileSink::defaultPath()
{
    char buf[PATH_MAX] = {0};
    const ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n > 0)
    {
        std::string exe(buf, static_cast<size_t>(n));
        const std::string::size_type p = exe.find_last_of('/');
        if (p != std::string::npos)
        {
            const std::string bin_dir = exe.substr(0, p);       // .../<proj>/build
            const std::string::size_type q = bin_dir.find_last_of('/');
            const std::string proj = (q != std::string::npos) ? bin_dir.substr(0, q) : bin_dir;
            return proj + "/log/dr.text";
        }
    }
    char cwd[PATH_MAX] = {0};
    if (::getcwd(cwd, sizeof(cwd)) != nullptr)
        return std::string(cwd) + "/log/dr.text";
    return "log/dr.text";
}

} // namespace dr_engine
