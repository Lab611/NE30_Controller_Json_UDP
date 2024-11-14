#ifndef JSON_UDP_SERVER_HPP
#define JSON_UDP_SERVER_HPP


#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <string>
#include <thread>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class JsonUdpServer {
public:
    JsonUdpServer(int port);

    ~JsonUdpServer();

    void start_receive();

    json get_json() { return json_; };
    void clean_json() { json_ = {}; };

private:
    void receive_data();

    SOCKET sockfd_;
    int port_;
    struct sockaddr_in server_addr_;
    WSADATA wsaData_;
    json json_;
};

JsonUdpServer::JsonUdpServer(int port) : port_(port) {
    // 初始化 Winsock
    if (WSAStartup(MAKEWORD(2, 2), &wsaData_) != 0) {
        std::cerr << "Winsock initialization failed" << std::endl;
        exit(1);
    }

    // 创建 UDP 套接字
    sockfd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd_ == INVALID_SOCKET) {
        std::cerr << "Socket creation failed" << std::endl;
        WSACleanup();
        exit(1);
    }

    // 设置服务器地址
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr_.sin_port = htons(port_);

    // 绑定端口
    if (bind(sockfd_, (struct sockaddr *) &server_addr_, sizeof(server_addr_)) == SOCKET_ERROR) {
        std::cerr << "Bind failed" << std::endl;
        closesocket(sockfd_);
        WSACleanup();
        exit(1);
    }
}

JsonUdpServer::~JsonUdpServer() {
    closesocket(sockfd_);
    WSACleanup();
}

void JsonUdpServer::start_receive() {
    std::thread receiver_thread(&JsonUdpServer::receive_data, this);
    receiver_thread.detach(); // 异步接收数据
}

void JsonUdpServer::receive_data() {
    char buffer[1024]; // 用于存储接收到的数据
    sockaddr_in client_addr;
    int client_addr_len = sizeof(client_addr);

    while (true) {
        // 接收 UDP 数据
        int recv_len = recvfrom(sockfd_, buffer, sizeof(buffer), 0, (struct sockaddr *) &client_addr, &client_addr_len);
        if (recv_len == SOCKET_ERROR) {
            std::cerr << "Receive failed: " << WSAGetLastError() << std::endl;
            continue;
        }

        // 将接收到的字节数据转换为 JSON 字符串
        buffer[recv_len] = '\0'; // 确保字符串以 null 终止
        std::string json_str(buffer);

        try {
            // 解析 JSON
            json_ = json::parse(json_str);
            // std::cout << "Received JSON: " << json_.dump(4) << std::endl;

            // 你可以在这里处理 JSON 数据，比如响应客户端
            // 或者通过 get_json 后单独处理
        } catch (const json::parse_error &e) {
            std::cerr << "JSON parsing error: " << e.what() << std::endl;
        }
    }
}


// #include <iostream>
// #include "JsonUdpServer.hpp"  // 包含你的 JsonUdpServer 类头文件
//
// int main() {
//     // 定义服务器的端口号
//     int port = 8080;
//
//     // 创建 JsonUdpServer 对象，传入端口号
//     JsonUdpServer server(port);
//
//     // 启动接收线程
//     server.start_receive();
//
//     // 这里可以添加其他逻辑，比如主程序等待或处理其他任务
//     std::cout << "Server is running. Press Enter to exit." << std::endl;
//     std::cin.get();  // 按下回车时退出程序
//
//     return 0;
// }

#endif
