# Clion 编译的 NE 30 Controller

理论上来说 环境都齐了 需要使用 msvc 做为编译器

msvc 可以通过下载 vs 安装 也可以直接下载对应版本

# 文件建构

## 3rdparty

所有第三方库

1. `eigen-3.4.0` [Eigen 3.4.0](https://eigen.tuxfamily.org/index.php?title=Main_Page)
2. `innfos` [机械臂 SDK](https://github.com/mintasca/innfos-cpp-sdk)
3. `nlohmann` [nlohmann/json](https://github.com/nlohmann/json)

## include

头文件 这里都用了 hpp 格式 单文件超人

1. `json_msg_builder.hpp` 通信协议
2. `json_udp_server.hpp` UDP 接收消息
3. `ne30_control.hpp` 机械臂控制类
4. `ne30_pos.hpp` 机械臂的位置信息封装为了一个类 

## main.cpp

主函数

## connect_info.json

修改 UDP 连接配置


# 通信协议

```cpp
// ./include/json_msg_builder.hpp

typedef enum {
    DEV_UNKNOWN = -1,
    DEV_TOUCH = 1,
    DEV_KEYBOARD = 2,
    DEV_ELITE = 101,
    DEV_NE30 = 102,
} JSON_DEVICE_TYPE;

typedef enum {
    CMD_UNKNOWN = -1,
    CMD_MOVE = 0,
    CMD_PAUSE = 1,
    CMD_RESET = 2,
    CMD_QUIT = 3,
} JSON_CMD_TYPE;

inline nlohmann::json build_json_from_pos_and_rot(JSON_DEVICE_TYPE device,
                                                  JSON_CMD_TYPE cmd = CMD_MOVE,
                                                  std::vector<double> pos_and_rot={0}) {
    return nlohmann::json{{"device", device}, {"cmd", cmd}, {"data", pos_and_rot}};
}


inline nlohmann::json build_json_from_cmd(JSON_DEVICE_TYPE device,
                                          JSON_CMD_TYPE cmd) {
    return nlohmann::json{{"device", device}, {"cmd", cmd}};
}

```
