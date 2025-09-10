# Linux 移植版本，使用了gcc，理论上来说环境都齐全了，2025.9.10暂未测试

gcc和make之类的东西缺什么补什么，不会弄了优先问ai/百度，一般情况下只需要apt

# linux环境安装
推荐wsl环境，wsl教程详见csdn  
[wsl搭建](https://blog.csdn.net/cepengyuan/article/details/116006481)

好处是不需要再刷双系统/安装virtualbox(Vmware),并且能直接使用主机的网络代理而不需要额外的vpn，直接使用主机的vscode remote就能编辑代码，还有一键git等，不过这些也需要一些简单的配置（比烧系统轻松得多），百度就行

2025.9.10暂未测试能否直接移植到nuc上，理论上可行

# linux使用的简要教程
    如果你是完全的linux新手，那么记住你只需要把报错送给ai，然后按照ai的说的做（修改代码前先进行备份），如果你只是想运行一个代码，那么你需要

* 1.在CMakeLists.txt的 *** 同级目录 *** 下，使用终端命令：

        mkdir build && cd build
* 2.现在你的终端应该显示如下：

        ~xxxx/build:

    输入：

        cmake ..
    这是告诉vscode你要使用CMakeList中的设置来编译
    
    如果cmake报错了，把错误和CMakeList送给ai，让ai帮忙修改，如果你是c++高手那么随意

    成功后，输入：
        
        make -j8
    
    这是在编译你的项目文件， -j后的参数表示make使用的核心数，根据你的cpu而定，一般而言大家的pc用-j8都没问题，嵌入式设备适当减小，比如-j4

    make报错了也是把错误送给ai，如果报错过长就自己先看一下哪里错了，能不能改
    
* 3.成功到这一步后，输入：

        ./你的项目名称
    至于项目名称，在cmake中有写，或者在build命令输出的最后一行会有“Built target 你的项目名称”

    这时候应该就成功运行程序了


# 文件建构

## 3rdparty

所有第三方库

1. `eigen` [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
2. `innfos` [机械臂 SDK](https://github.com/mintasca/innfos-cpp-sdk)
3. `nlohmann` [nlohmann/json](https://github.com/nlohmann/json)

    Note：推荐使用git submodule，然后再在cmake中修改include路径


## include

头文件 这里都用了 hpp 格式 单文件超人

* TODO：如果你要追求精益求精，可以尝试做一个pch，这样编译会快一点，代码看上去也简洁一点

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
