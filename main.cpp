#include "json_udp_server.hpp"
#include "json_msg_builder.hpp"
#include "ne30_control.hpp"
#include <iostream>
#include <fstream>
#include <thread>
#include <unistd.h> // For usleep
#include <termios.h> // For terminal control
#include <fcntl.h> // For file control

#define CONTROL_NE30

using namespace std;

// 函数声明
nlohmann::json get_json_from_file(const string &file_path);
int kbhit(void);
void enableRawMode();
void disableRawMode();

// 全局变量用于终端设置
struct termios orig_termios;

int main() {
    // 设置终端为非阻塞模式
    enableRawMode();
    
    // 创建 JsonUdpServer 对象，传入端口号
    auto ip_config_json = get_json_from_file("../connect_info.json");
    const int port_ = ip_config_json["port"];
    cout << "port: " << port_ << endl;
    JsonUdpServer server(port_);
    // 启动接收线程，接收touch发来的json
    server.start_receive();
    nlohmann::json recv_msg;

#ifdef CONTROL_NE30
    // 初始化机械臂 要等待一段时间机械臂复位
    NE30Control Ne30;
    cout << "initing ..." << endl;
    for (int i = 0; i < 100; i++) {
        cout << i  << "% \r" << flush;
        usleep(100000); // 100ms
    }
    cout << endl;
    auto ne30_pos = Ne30.getPos();
    printf("x: %5.2f y: %5.2f z: %5.2f \n", ne30_pos.x, ne30_pos.y, ne30_pos.z);
    printf("pitch: %8.4f yaw: %8.4f roll: %8.4f\n", ne30_pos.pitch, ne30_pos.yaw, ne30_pos.roll);
    std::cout << "INIT DONE\n";
#endif

    while (true) {
        if (kbhit()) {
            char c = getchar();
            if ('q' == c) {
                break;
            }
        }
        try {
            recv_msg = server.get_json();
            server.clean_json();
            cout<<"recv_msg: "<<recv_msg<<endl;
            if (recv_msg.empty()) {//处理空的json
                usleep(10000); // 10ms
                continue;
            }
            if (recv_msg.contains("cmd") && recv_msg["cmd"] == CMD_UNKNOWN) {//处理未知命令
                usleep(10000); // 10ms
                continue;
            }
        } catch (const nlohmann::json::parse_error &e) {
            std::cerr << "Parse error: " << e.what() << std::endl;
            usleep(10000); // 10ms
            continue;
        }
        auto ne30_last_pos = ne30_pos;
        std::cout << "recv_msg JSON: " << recv_msg.dump(4) << std::endl;
        switch ((JSON_CMD_TYPE) recv_msg["cmd"]) {
            case CMD_MOVE://收到移动的指令后先设置ne30_pos，稍后写入
                cout << "CMD_MOVE" << std::endl;
                ne30_pos.x += (double) recv_msg["data"][0];
                ne30_pos.y += (double) recv_msg["data"][1];
                ne30_pos.z += (double) recv_msg["data"][2];
            // ne30_pos.roll = vec.at(3);
            // ne30_pos.pitch = vec.at(4);
            // ne30_pos.yaw = vec.at(5);
                break;
            case CMD_RESET:
                break;
            default:
                break;
        }

        printf("x: %5.2f y: %5.2f z: %5.2f \n", ne30_pos.x, ne30_pos.y, ne30_pos.z);
        printf("pitch: %8.4f yaw: %8.4f roll: %8.4f\n", ne30_pos.pitch, ne30_pos.yaw, ne30_pos.roll);

        // 控制机械臂
        if (Ne30.setPos(ne30_pos, 0) == FAILED_TO_MOVE) {//控制和报错一起写了，如果setpos出了问题就直接报错
            ne30_pos = ne30_last_pos;
            cerr << "Failed to move\n";
        }

        usleep(10000); // 10ms
    }
    
    // 恢复终端设置
    disableRawMode();
    return 0;
}

nlohmann::json get_json_from_file(const string &file_path) {
    // 打开文件
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Could not open the file: " << file_path << std::endl;
        return 1; // 返回错误码
    }

    // 解析 JSON 文件
    nlohmann::json j;
    try {
        file >> j; // 使用 operator>> 直接读取文件内容到 json 对象
    } catch (const nlohmann::json::parse_error &e) {
        std::cerr << "Parse error: " << e.what() << std::endl;
        return {};
    }
    return j;
}

// 检测是否有键盘输入
int kbhit(void) {
    struct timeval tv;
    fd_set read_fd;
    
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    
    FD_ZERO(&read_fd);
    FD_SET(0, &read_fd);
    
    if(select(1, &read_fd, NULL, NULL, &tv) == -1)
        return 0;
    
    if(FD_ISSET(0, &read_fd))
        return 1;
    
    return 0;
}

// 设置终端为原始模式
void enableRawMode() {
    struct termios raw;
    
    tcgetattr(STDIN_FILENO, &orig_termios);
    
    raw = orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 1;
    
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

// 恢复终端设置
void disableRawMode() {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}