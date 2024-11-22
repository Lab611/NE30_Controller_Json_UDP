#include "json_udp_server.hpp"
#include "json_msg_builder.hpp"
#include "ne30_control.hpp"
#include <conio.h> // For kbhit and getch
#include <iostream>
#include <fstream>
#include <thread>

#define CONTROL_NE30

using namespace std;


nlohmann::json get_json_from_file(const string &file_path);


int main() {
    // 创建 JsonUdpServer 对象，传入端口号
    auto ip_config_json = get_json_from_file("../connect_info.json");
    const int port_ = ip_config_json["port"];
    cout << "port: " << port_ << endl;
    JsonUdpServer server(port_);
    // 启动接收线程
    server.start_receive();
    nlohmann::json recv_msg;

#ifdef CONTROL_NE30
    // 初始化机械臂 要等待一段时间机械臂复位
    NE30Control Ne30;
    cout << "initing ..." << endl;
    for (int i = 0; i < 30; i++) {
        cout << i << "% \n";
        Sleep(100);
    }
    auto ne30_pos = Ne30.getPos();
    auto ne30_pos_init = ne30_pos;
    auto ne30_pos_RESET = ne30_pos;
    printf("x: %5.2f y: %5.2f z: %5.2f \n", ne30_pos.x, ne30_pos.y, ne30_pos.z);
    printf("pitch: %8.4f yaw: %8.4f roll: %8.4f\n", ne30_pos.pitch, ne30_pos.yaw, ne30_pos.roll);
    std::cout << "INIT DONE\n";
#endif

    while (true) {
        if (_kbhit()) {
            if ('q' == _getch()) {
                break;
            }
        }
        try {
            recv_msg = server.get_json();
            server.clean_json();
            if (recv_msg.empty()) {
                this_thread::sleep_for(chrono::milliseconds(10));
                continue;
            }
            if (recv_msg.contains("cmd") && recv_msg["cmd"] == CMD_UNKNOWN) {
                this_thread::sleep_for(chrono::milliseconds(10));
                continue;
            }
        } catch (const nlohmann::json::parse_error &e) {
            std::cerr << "Parse error: " << e.what() << std::endl;
            this_thread::sleep_for(chrono::milliseconds(10));
            continue;
        }
        auto ne30_last_pos = ne30_pos;
        std::cout << "recv_msg JSON: " << recv_msg.dump(4) << std::endl;
        switch ((JSON_CMD_TYPE) recv_msg["cmd"]) {
            case CMD_MOVE:
                cout << "CMD_MOVE" << std::endl;
                ne30_pos.x += (double) recv_msg["data"][0];
                ne30_pos.y += (double) recv_msg["data"][1];
                ne30_pos.z += (double) recv_msg["data"][2];

                // ne30_pos.roll = (double) recv_msg["data"][5];
                // ne30_pos.pitch = (double) recv_msg["data"][3];
                // ne30_pos.yaw = (double) recv_msg["data"][4];
                break;
            case CMD_MOVE_ABS:
                cout << "CMD_MOVE_ABS" << std::endl;
                ne30_pos.x = ne30_pos_init.x + (double) recv_msg["data"][0];
                ne30_pos.y = ne30_pos_init.y + (double) recv_msg["data"][1];
                ne30_pos.z = ne30_pos_init.z + (double) recv_msg["data"][2];
                ne30_pos.roll = (double) recv_msg["data"][5];
                ne30_pos.pitch = (double) recv_msg["data"][4];
                ne30_pos.yaw = (double) recv_msg["data"][3];
                break;
            case CMD_RESET:
                Ne30.returnInitPosition();
                ne30_pos = ne30_pos_RESET;
                break;
            case CMD_SET_ABS:
                cout << "ABS" <<endl;
                ne30_pos_init = Ne30.getPos();
                break;
            default:
                break;
        }

        printf("x: %5.2f y: %5.2f z: %5.2f \n", ne30_pos.x, ne30_pos.y, ne30_pos.z);
        printf("pitch: %8.4f yaw: %8.4f roll: %8.4f\n", ne30_pos.pitch, ne30_pos.yaw, ne30_pos.roll);

        // 控制机械臂
        if (Ne30.setPos(ne30_pos, 0) == FAILED_TO_MOVE) {
            ne30_pos = ne30_last_pos;
            cout << "Failed to move\n";
        }

        this_thread::sleep_for(chrono::milliseconds(10));
    }

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
    json j;
    try {
        file >> j; // 使用 operator>> 直接读取文件内容到 json 对象
    } catch (const nlohmann::json::parse_error &e) {
        std::cerr << "Parse error: " << e.what() << std::endl;
        return {};
    }
    return j;
}
