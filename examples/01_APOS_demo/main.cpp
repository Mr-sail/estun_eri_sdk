// =============================================================================
// 示例名称：ERI 实时控制插补示例
// 功能：配置 PC 端通信参数 → 获取控制权 → 设置运动参数 → MoveJ 到起点 → 启动实时控制
//      → 多维五次多项式轨迹插补 → 实时发送目标位姿 → 结束运动 → 释放控制权
// =============================================================================

#ifdef _WIN32
#include <Windows.h>
#else
#include <thread>
#include <chrono>
#include <limits.h>
#include <unistd.h>
// 在 Linux 下将小写的 sleep 映射到标准库，以兼容 Windows 习惯
#define Sleep(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))
#endif

#include <iostream>
#include <iostream>
#include <thread>
#include <atomic>
#include <cmath>
#include <algorithm>

#include "ERIParamManager.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ------------------------ ERI 控制器实例 ------------------------
static ERIParamManager& getEriManager() {
    // 延迟初始化，确保配置目录切换完成后再加载 SDK 依赖的配置文件。
    static ERIParamManager eriManager;
    return eriManager;
}

static bool switchToExecutableDir() {
#ifdef __linux__
    char exePath[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", exePath, sizeof(exePath) - 1);
    if (len <= 0) {
        return false;
    }
    exePath[len] = '\0';

    std::string fullPath(exePath);
    std::size_t pos = fullPath.find_last_of('/');
    if (pos == std::string::npos) {
        return false;
    }

    std::string exeDir = fullPath.substr(0, pos);
    // SDK 依赖当前工作目录加载配置文件，这里统一切到可执行文件目录。
    return chdir(exeDir.c_str()) == 0;
#else
    return true;
#endif
}

// ------------------------ 五次多项式插值函数 ------------------------
// 公式：pos(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
// 输入：起点 p0、终点 p1，总时间 T，当前时间 t，输出位置 pos
double compute5thPoly(double p0, double p1, double T, double t) {
    double s = t / T;
    double s3 = s * s * s;
    double s4 = s3 * s;
    double s5 = s4 * s;
    return p0 + (p1 - p0) * (10.0 * s3 - 15.0 * s4 + 6.0 * s5);
}

// ------------------------ 实时插补运动主函数 ------------------------
void processRealtimeData() {
    ERIParamManager& eriManager = getEriManager();

    // ===== 控制与滤波参数 =====
    const int sampleTime = 4;   // 控制周期（ms）
    const int filterType = 0;   // 0 = 无滤波

    // ===== 起点与终点位姿 =====
    // 示例点位仅用于演示接口调用。真实设备运行前必须按机器人型号、安装姿态、
    // 负载、工作空间和现场安全边界重新确认，避免直接复制示例点位运行。
    double startPos[6] = { 0, 0, 0, 0, 0, 0 };         // 初始位姿（可替换为实际读取值）
    double goalPos[6] = { 20, 30, 40, 50, 60, 70 };   // 目标位姿
    double target[6];                               // 实时目标位姿
    UINT64 timeStamp = 0;                           // 时间戳

    // ===== 获取控制权 =====
    if (!eriManager.getControlRobot()) {
        std::cerr << "获取机器人控制权失败!" << std::endl;
        return;
    }
    std::cout << "成功获取机器人控制权!" << std::endl;

    // ===== 设置实时控制参数 =====
    if (!eriManager.setERIMotionParam(sampleTime, filterType)) {
        std::cerr << "设置ERI运动参数失败!" << std::endl;
        return;
    }
    std::cout << "已设置实时控制参数。" << std::endl;

    // ===== MoveJ 到起始位置 =====
    int movJRet = eriManager.movJ(startPos);
    if (movJRet != 1) {
        std::cerr << "MoveJ 到起始点未完成，返回值: " << movJRet
                  << " (1=完成, 0=执行中, -1=失败)" << std::endl;
        return;
    }
    std::cout << "MoveJ 成功！等待稳定…" << std::endl;
    Sleep(1000);

    // ===== 启动实时控制（APOS模式） =====
    if (!eriManager.startServo(0)) {
        std::cerr << "启动实时控制失败!" << std::endl;
        return;
    }
    std::cout << "已启动实时控制。" << std::endl;

    // ===== 初始化插补时间参数 =====
    const double T = 2000.0;  // 插补总时间（ms）
    double t = 0.0;           // 当前时间（ms）
    std::copy(startPos, startPos + 6, target); // 初始目标位姿赋值

    // ===== 实时插补主循环（五次多项式）=====
    // servoToAPOS 为阻塞式发送接口；网络通信良好时，单次阻塞时间通常接近 sampleTime。
    while (t <= T) {
        for (int i = 0; i < 6; ++i) {
            target[i] = compute5thPoly(startPos[i], goalPos[i], T, t);
        }

        if (eriManager.servoToAPOS(timeStamp, target, false) == -1) {
            std::cerr << "实时目标发送失败!" << std::endl;
            break;
        }

        timeStamp += 4;
        t += 4;
    }

    // ===== 发送最终点，标记为终点 =====
    eriManager.servoToAPOS(timeStamp, target, true);
    // 不要在发送终点后立刻尝试释放控制权，需要等待机器人实际结束运动。
    std::cout << "已发送终点，等待机器人实际结束运动后再释放控制权。" << std::endl;
    Sleep(5000);

    // ===== 等待机器人实际结束运动后，再释放机器人控制权 =====
    if (eriManager.releaseControlOverRobot())
        std::cout << "控制权释放成功。" << std::endl;
    else
        std::cout << "释放控制权失败。" << std::endl;
}

// ------------------------ 主程序入口 ------------------------
int main() {
    if (!switchToExecutableDir()) {
        std::cerr << "警告: 切换到可执行文件目录失败，可能找不到配置文件。" << std::endl;
    }

    ERIParamManager& eriManager = getEriManager();

    // 仅配置 PC 端通信参数。运行示例前请改成现场机器人控制器 IP。
    std::string robotIp = "172.31.16.227";
    uint16_t cmdPort = 61210;
    uint16_t servoPort = 61211;
    uint16_t statusPort = 61212;

    if (!eriManager.setCommunicationParam(robotIp.c_str(), cmdPort, servoPort, statusPort)) {
        std::cout << "PC端通信参数设置失败!" << std::endl;
        return -1;
    }
    Sleep(50); // 保留短暂等待；连接成功以 SDK 三个通道的 connect success 日志为准。

    // 用户输入控制界面
    char input = '0';
    while (true) {
        std::cout << "输入指令: (3:开始插补, q:退出): ";
        std::cin >> input;

        if (input == 'q') break;
        if (input == '3') processRealtimeData();
    }

    return 0;
}
