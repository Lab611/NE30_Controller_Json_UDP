#include "json_udp_server.hpp"
#include "json_msg_builder.hpp"
#include "ne30_control.hpp"
#include <iostream>
#include <fstream>
#include <thread>

// #define CONTROL_NE30

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
    NE30Control Ne30;
#endif

    while (true) {
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

        std::cout << "recv_msg JSON: " << recv_msg.dump(4) << std::endl;
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
