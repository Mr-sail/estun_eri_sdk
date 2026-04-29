#ifdef _WIN32
#include <Windows.h>
#else
#include <thread>
#include <chrono>
#include <limits.h>
#include <unistd.h>
#define Sleep(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))
#endif

#include <iostream>
#include <mutex>
#include <string>

#include "ERIParamManager.h"

static std::mutex g_statusMutex;
static RobotStatus g_latestStatus = {};
static bool g_hasStatus = false;

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
    return chdir(exeDir.c_str()) == 0;
#else
    return true;
#endif
}

static void onRobotStatus(RobotStatus robotStatus) {
    std::lock_guard<std::mutex> lock(g_statusMutex);
    // 回调由 SDK 异步线程触发，只保存最新状态，避免阻塞状态接收线程。
    g_latestStatus = robotStatus;
    g_hasStatus = true;
}

static bool getLatestRobotStatus(RobotStatus& robotStatus) {
    std::lock_guard<std::mutex> lock(g_statusMutex);
    if (!g_hasStatus) {
        return false;
    }

    robotStatus = g_latestStatus;
    return true;
}

static void printDoubleArray(const char* label, const double values[], int count) {
    std::cout << label;
    for (int i = 0; i < count; ++i) {
        std::cout << values[i];
        if (i + 1 < count) {
            std::cout << ", ";
        }
    }
    std::cout << std::endl;
}

static void printJointArrayAsDegrees(const char* label, const double values[], int count) {
    const double radToDeg = 180.0 / 3.14159265358979323846;

    std::cout << label;
    for (int i = 0; i < count; ++i) {
        std::cout << values[i] * radToDeg;
        if (i + 1 < count) {
            std::cout << ", ";
        }
    }
    std::cout << std::endl;
}

static void printLatestRobotStatus() {
    RobotStatus robotStatus = {};
    if (!getLatestRobotStatus(robotStatus)) {
        std::cout << "尚未收到状态回调，请确认状态端口已连接成功。" << std::endl;
        return;
    }

    std::cout << "机器人错误状态: " << robotStatus.isRobotError << std::endl;
    std::cout << "纠偏开始标志: " << static_cast<int>(robotStatus.isStartCorrect) << std::endl;
    std::cout << "终点完成标志: " << robotStatus.isEndPoint << std::endl;
    // worldCpos 前 6 项顺序为 x, y, z, roll, pitch, yaw；位置单位为 mm，姿态单位为 rad。
    printDoubleArray("世界坐标 TCP [x, y, z, roll, pitch, yaw] 单位 [mm, mm, mm, rad, rad, rad]: ",
                     robotStatus.worldCpos,
                     6);
    // SDK 回调中的 jointValue 原始单位为 rad；示例转为 deg 打印，便于现场核对关节角。
    printJointArrayAsDegrees("关节值 [J1, J2, J3, J4, J5, J6] 单位 [deg]: ", robotStatus.jointValue, 6);
}

int main() {
    if (!switchToExecutableDir()) {
        std::cerr << "警告: 切换到可执行文件目录失败，可能找不到配置文件。" << std::endl;
    }

    ERIParamManager eriManager;
    // 仅配置 PC 端通信参数。运行示例前请改成现场机器人控制器 IP。
    std::string robotIp = "172.31.16.227";
    uint16_t cmdPort = 61210;
    uint16_t servoPort = 61211;
    uint16_t statusPort = 61212;

    if (!eriManager.setCommunicationParam(robotIp.c_str(), cmdPort, servoPort, statusPort)) {
        std::cerr << "PC端通信参数设置失败!" << std::endl;
        return -1;
    }

    eriManager.setRobotStatusCallback(onRobotStatus);
    Sleep(50); // 保留短暂等待；状态回调是否到达以 UDP 状态通道连接成功为准。

    char input = '0';
    while (true) {
        std::cout << "输入指令: (3: 打印最新状态回调, q: 退出): ";
        std::cin >> input;

        if (input == 'q') break;
        if (input == '3') printLatestRobotStatus();
    }

    return 0;
}
