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
#include <string>
#include "ERIParamManager.h"

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

void processReadDO(ERIParamManager& eriManager) {
    UINT32 doValue = eriManager.getCurrentDO(4);
    std::cout << "当前系统 DO: " << doValue << std::endl;
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
    Sleep(50); // 保留短暂等待；连接成功以 SDK 三个通道的 connect success 日志为准。

    char input = '0';
    while (true) {
        std::cout << "输入指令: (3: 读取DO, q: 退出): ";
        std::cin >> input;

        if (input == 'q') break;
        if (input == '3') processReadDO(eriManager);
    }

    return 0;
}
