// =============================================================================
// 示例名称：ERI 实时圆轨迹控制
// 功能：控制机器人 TCP 沿一个平面圆轨迹运动，使用五次多项式进行平滑插值
// 步骤：配置 PC 端通信参数 → 获取控制权 → 参数设置 → MoveJ → 获取起点 → 实时圆轨迹控制 → 终点 → 释放控制权
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

// ------------------------ 插值函数（五次多项式） ------------------------
// 功能：根据当前时间 t 和总时间 T，平滑计算角度 theta
double computeInterpolatedTheta(double t, double T, double theta0, double theta1) {
    double s = t / T;
    return theta0 + (theta1 - theta0) * (10 * pow(s, 3) - 15 * pow(s, 4) + 6 * pow(s, 5));
}

// ------------------------ 实时圆轨迹执行函数 ------------------------
void processRealtimeData() {
    ERIParamManager& eriManager = getEriManager();

    // ===== 控制与滤波参数 =====
    const int sampleTime = 4;    // 控制周期（ms）
    const int filterType = 0;    // 滤波器类型（0 = 无滤波）

    // ===== 初始位姿（MoveJ 目标） =====
    // 示例点位仅用于演示接口调用。真实设备运行前必须按机器人型号、安装姿态、
    // 负载、工作空间和现场安全边界重新确认，避免直接复制示例点位运行。
    double startPos[6] = { 1.224, -0.999, 19.75, 0, 71.249, 1.209 };  // 自定义起点
    double startTCP[6];    // 实际TCP起点（从控制器读取）
    double target[6];      // 每周期的实时目标位置
    UINT64 timeStamp = 0;  // 实时控制时间戳

    // ===== 圆轨迹参数 =====
    const double r = 50.0;          // 圆半径（mm）
    double centerX, centerY, centerZ; // 圆心坐标
    const double theta_0 = 0.0;
    const double theta_1 = 2 * M_PI; // 绕一整圈

    // ===== 插值参数 =====
    const double T = 1000.0 * sampleTime;  // 总时长（ms）
    const double dt = 1.0 * sampleTime;    // 每周期步进（ms）
    double t = 0.0;           // 当前时间

    // ===== 获取机器人控制权 =====
    if (!eriManager.getControlRobot()) {
        std::cerr << "获取机器人控制权失败!" << std::endl;
        return;
    }
    std::cout << "已获取控制权。" << std::endl;

    // ===== 设置实时控制参数 =====
    if (!eriManager.setERIMotionParam(sampleTime, filterType)) {
        std::cerr << "设置实时控制参数失败!" << std::endl;
        return;
    }

    // ===== MoveJ 到初始位置 =====
    int movJRet = eriManager.movJ(startPos);
    if (movJRet != 1) {
        std::cerr << "MoveJ 到初始位置未完成，返回值: " << movJRet
                  << " (1=完成, 0=执行中, -1=失败)" << std::endl;
        return;
    }
    std::cout << "MoveJ 成功。" << std::endl;
    Sleep(1000); // 等待稳定

    // ===== 获取当前 TCP 位置作为轨迹起点 =====
    if (!eriManager.getWorldCpos(startTCP)) {
        std::cerr << "获取当前TCP位置失败!" << std::endl;
        return;
    }

    // 打印当前 TCP 位置
    std::cout << "当前TCP位置：";
    for (int i = 0; i < 6; ++i) {
        std::cout << startTCP[i] << " ";
        target[i] = startTCP[i]; // 初始化目标位姿
    }
    std::cout << std::endl;

    // ===== 计算圆心位置（假设圆心在 X 方向左移 r）=====
    centerX = startTCP[0] - r;
    centerY = startTCP[1];
    centerZ = startTCP[2]; // 保持Z不变

    // ===== 启动实时控制（CPOS模式）=====
    if (!eriManager.startServo(1)) {
        std::cerr << "开启实时控制失败!" << std::endl;
        return;
    }
    std::cout << "实时控制已开启。" << std::endl;

    // ===== 主控制循环：发送轨迹点 =====
    // servoToCPOS 为阻塞式发送接口；网络通信良好时，单次阻塞时间通常接近 sampleTime。
    while (t <= T) {
        // 插值计算当前角度
        double theta = computeInterpolatedTheta(t, T, theta_0, theta_1);

        // 计算当前圆周点（X-Y 平面圆）
        target[0] = centerX + r * std::cos(theta); // X
        target[1] = centerY + r * std::sin(theta); // Y
        target[2] = centerZ;                       // Z 保持不变

        // 发送实时目标位姿
        if (eriManager.servoToCPOS(timeStamp, target, false) == -1) {
            std::cerr << "实时控制发送失败!" << std::endl;
            break;
        }

        // 更新时间戳与时间变量
        timeStamp += sampleTime;
        t += dt;
    }

    // ===== 最后一点作为终点 =====
    eriManager.servoToCPOS(timeStamp, target, true);
    // 不要在发送终点后立刻尝试释放控制权，需要等待机器人实际结束运动。
    std::cout << "已发送终点，等待机器人实际结束运动后再释放控制权。" << std::endl;
    Sleep(5000);

    // ===== 等待机器人实际结束运动后，再释放机器人控制权 =====
    if (eriManager.releaseControlOverRobot())
        std::cout << "控制权释放成功。" << std::endl;
    else
        std::cerr << "释放控制权失败。" << std::endl;
}

// ------------------------ 主程序入口 ------------------------
int main() {
    if (!switchToExecutableDir()) {
        std::cerr << "警告: 切换到可执行文件目录失败，可能找不到配置文件。" << std::endl;
    }

    ERIParamManager& eriManager = getEriManager();

    // 仅配置 PC 端通信参数。运行示例前请改成现场机器人控制器 IP。
    std::string robotIp = "172.31.16.18";
    uint16_t cmdPort = 61210;
    uint16_t servoPort = 61211;
    uint16_t statusPort = 61212;

    if (!eriManager.setCommunicationParam(robotIp.c_str(), cmdPort, servoPort, statusPort)) {
        std::cerr << "PC端通信参数设置失败!" << std::endl;
        return -1;
    }
    Sleep(50); // 保留短暂等待；连接成功以 SDK 三个通道的 connect success 日志为准。

    // 用户交互命令行界面
    char input = '0';
    while (true) {
        std::cout << "输入指令: (3: 开始画圆, q: 退出): ";
        std::cin >> input;

        if (input == 'q') break;
        if (input == '3') processRealtimeData();
    }

    return 0;
}
