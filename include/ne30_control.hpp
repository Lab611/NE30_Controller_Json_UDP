#ifndef NE30_CONTROL_HPP
#define NE30_CONTROL_HPP

#include <NE30_pos.hpp>
#include <iostream>
#include <fstream>
#include <thread>
#include <signal.h>
#include <string.h>
#include <string>
#include <chrono>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <ctime>
#include <winsock.h>
#include <windows.h>
#include "actuatorcontroller.h"

using namespace std;

/*
*机械臂运动 类
*/
class NE30Control {
public:
    NE30Control();

    ~NE30Control();

    /// <summary>
    /// xyzpyr 设置机械臂末端位姿
    /// </summary>
    /// <param name="pos_x"></param>
    /// <param name="pos_y"></param>
    /// <param name="pos_z"></param>
    /// <param name="pitch">y转动</param>
    /// <param name="yaw">z转动</param>
    /// <param name="roll">x转动</param>
    /// <param name="force_moving">是否强制运动</param>
    /// <returns>失败返回FAILED_TO_MOVE，即 -1 </returns>
    int setPos(double pos_x, double pos_y, double pos_z, double pitch, double yaw, double roll, BOOL force_moving);

    /// <summary>
    /// 结构体设置机械臂末端位姿
    /// </summary>
    /// <param name="Pos">NE30_Pos 结构体</param>
    /// <param name="force_moving">是否强制运动</param>
    /// <returns>失败返回FAILED_TO_MOVE，即 -1</returns>
    int setPos(const NE30Pos &Pos, BOOL force_moving);

    // 旋转平移矩阵控制
    int setPos(double pos_x, double pos_y, double pos_z, const Eigen::Matrix3d &rotation_matrix, BOOL force_moving);

    // 获取电机角度
    std::vector<double> getAngles() const;

    // 设置电机角度
    void setAngles(double r1, double r2, double r3, double r4, double r5, double r6) const;

    void setAngles(const std::vector<double> &angles) const;


    // 设置0位置，更换零位后需要改代码！
    int setInit() const;

    // 返回机械臂当前位姿
    NE30Pos get_init_pos() const;

    // 失效所有执行器
    void setDisable() const;

    // void printAngle();
    Eigen::Matrix4d getPosMatrix() const;

    NE30Pos getPos() const;

private:
    NE30Pos init_pos;

    //定义全局变量
    //初始化执行器
    ActuatorController *pController;
    //定义错误变量ec
    ErrorsDefine ec;
    //查看已连接执行器，并返回UnifiedID数组
    std::vector<ActuatorController::UnifiedID> uIDArray;

    //机械臂DH数据
    //const double d[6 + 1] = { 0, 0.1015, 0, 0, 0.0792, 0.0792, 0.0395 };//第0个不用
    //const double a[6 + 1] = { 0, 0, -0.173, -0.173, 0, 0, 0 };
    const double d[6 + 1] = {0, 0.1205, 0, 0, 0.1165, 0.0792, 0.085}; //第0个不用
    const double a[6 + 1] = {0, 0, -0.2035, -0.173, 0, 0, 0};
    const double alpha[6 + 1] = {0, EIGEN_PI / 2, 0, 0,EIGEN_PI / 2, -EIGEN_PI / 2, 0};
    double theta[8 + 1][6 + 1]{}; //八组解，每组解六个角，第0个不用
    double current[6]{}, velocity[6]{}, position[6]{};
    Eigen::Matrix3d tH, tH_end;
    // 运动到一个合适的初始位置
    vector<double> true_start_encoders = {9, 2, 0, 0, 0, 0};
    vector<double> true_mid_encoders = {6, 2, -6, -6, -3, 0};
    vector<double> true_end_encoders = {9, 3, -11, -11, -5, 0};

    void paramFeedback(const ActuatorController::UnifiedID &uID, uint8_t paramType, double paramValue);

    void set_init_pos(double pos_x, double pos_y, double pos_z, double pitch, double yaw, double roll);

    void setEncoders(double e1, double e2, double e3, double e4, double e5, double e6) const;

    void setEncoders(const std::vector<double> &encoders) const;
};


//
// Created by Lab611-Y7KP on 24-11-13.
//
#include "ne30_control.hpp"


NE30Control::NE30Control() {
    cout << '\n' << __TIME__ << '\n' << __DATE__ << '\n';
    pController = ActuatorController::initController();
    uIDArray = pController->lookupActuators(ec);

    //打印出各执行器id和ip，并使能所有执行器
    if (!uIDArray.empty()) {
        for (const auto &uID: uIDArray) {
            cout << "Actuator ID: " << (int) uID.actuatorID << " IP address: " << uID.ipAddress.c_str() << endl;
        }
    } else {
        cout << "Connected error code:" << hex << ec << endl;
    }
    if (pController->enableAllActuators()) {
        cout << "All actuators have been enabled successfully! " << endl;
    }

    //激活所有执行器Mode_Profile_Pos模式
    pController->activateActuatorModeInBantch(uIDArray, Actuator::Mode_Profile_Pos);
    //system("pause");


    setEncoders(true_start_encoders);
    Sleep(1200);

    setEncoders(true_mid_encoders);
    Sleep(500);

    setEncoders(true_end_encoders);
    Sleep(500);
    // cout << "init" << endl;

    //获取当前末端位置
    Eigen::Matrix4d T06 = getPosMatrix();
    double pos_x = T06(0, 3), pos_y = T06(1, 3), pos_z = T06(2, 3);
    cout << "CURRENT POSITION \n";
    cout << T06 << endl << endl;
    // cout << "x y z\n";
    // cout << pos_x << " " << pos_y << " " << pos_z << endl;

    //pController->addParaRequestCallback(paramFeedback);
    //system("pause");
    getAngles();
}

NE30Control::~NE30Control() {
    setDisable();
}

// xyz 单位mm         pyr 单位弧度
int NE30Control::setPos(const double pos_x,
                        const double pos_y,
                        const double pos_z,
                        const double pitch,
                        const double yaw,
                        const double roll,
                        const BOOL force_moving) {
    //z-y-x
    // Rotation Matrix
    Eigen::Matrix3d tH;
    tH(0, 0) = cos(pitch) * cos(yaw);
    tH(1, 0) = cos(pitch) * sin(yaw);
    tH(2, 0) = -sin(pitch);
    tH(0, 1) = sin(pitch) * cos(yaw) * sin(roll) - cos(roll) * sin(yaw);
    tH(1, 1) = cos(roll) * cos(yaw) + sin(pitch) * sin(yaw) * sin(roll);
    tH(2, 1) = cos(pitch) * sin(roll);
    tH(0, 2) = sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch);
    tH(1, 2) = cos(roll) * sin(pitch) * sin(yaw) - cos(yaw) * sin(roll);
    tH(2, 2) = cos(roll) * cos(pitch);

    //cout << "EULER ANGLE 2 RM\n" << tH << "\n";
    if (setPos(pos_x, pos_y, pos_z, tH, force_moving) == 0) {
        set_init_pos(pos_x, pos_y, pos_z, pitch, yaw, roll);
        return 0;
    }
    return FAILED_TO_MOVE;
}

// 欧拉角结构体 trans_vec 单位是 mm 欧拉角单位是 弧度
int NE30Control::setPos(const NE30Pos &Pos, const BOOL force_moving) {
    return setPos(Pos.x, Pos.y, Pos.z, Pos.pitch, Pos.yaw, Pos.roll, force_moving);
}

// rotation matrix
// 运动实现都依靠这里
// trans_vec 单位是 mm
int NE30Control::setPos(const double pos_x,
                        const double pos_y,
                        const double pos_z,
                        const Eigen::Matrix3d &rotation_matrix,
                        const BOOL force_moving) {
    Eigen::Matrix4d TargetH;
    //z-y-x
    TargetH(0, 0) = rotation_matrix(0, 0);
    TargetH(1, 0) = rotation_matrix(1, 0);
    TargetH(2, 0) = rotation_matrix(2, 0);
    TargetH(0, 1) = rotation_matrix(0, 1);
    TargetH(1, 1) = rotation_matrix(1, 1);
    TargetH(2, 1) = rotation_matrix(2, 1);
    TargetH(0, 2) = rotation_matrix(0, 2);
    TargetH(1, 2) = rotation_matrix(1, 2);
    TargetH(2, 2) = rotation_matrix(2, 2);
    TargetH(0, 3) = pos_x / 1000.0;
    TargetH(1, 3) = pos_y / 1000.0;
    TargetH(2, 3) = pos_z / 1000.0;
    TargetH(3, 0) = 0;
    TargetH(3, 1) = 0;
    TargetH(3, 2) = 0;
    TargetH(3, 3) = 1;

#ifdef _DEBUG_
	cout << "目标旋转平移矩阵为：" << endl;
	cout << TargetH << endl << endl;
#endif

    double nx = TargetH(0, 0);
    double ny = TargetH(1, 0);
    double nz = TargetH(2, 0);
    double ox = TargetH(0, 1);
    double oy = TargetH(1, 1);
    double oz = TargetH(2, 1);
    double ax = TargetH(0, 2);
    double ay = TargetH(1, 2);
    double az = TargetH(2, 2);
    double px = TargetH(0, 3);
    double py = TargetH(1, 3);
    double pz = TargetH(2, 3);

    //求解
    double A, B, C, D, E, F, G, M, N; //用大写字母替代常数
    //注意，由于数组下标从0开始的问题，矩阵第一行第一列的元素是(0,0)
    //theta1
    A = ax * d[6] - px;
    B = ay * d[6] - py;
    C = d[4];
    //第一个解，赋给一到四组
    theta[1][1] = atan2(B, A) - atan2(C, sqrt(A * A + B * B - C * C));
    theta[2][1] = theta[1][1];
    theta[3][1] = theta[1][1];
    theta[4][1] = theta[1][1];
    //第二个解，赋给五到八组
    theta[5][1] = atan2(B, A) - atan2(C, -sqrt(A * A + B * B - C * C));
    theta[6][1] = theta[5][1];
    theta[7][1] = theta[5][1];
    theta[8][1] = theta[5][1];

    //theta5
    //由theta[1][1]产生的第一个解，赋给一到二组
    A = sin(theta[1][1]) * ax - cos(theta[1][1]) * ay;
    theta[1][5] = acos(A);
    theta[2][5] = theta[1][5];
    //由theta[1][1]产生的第二个解，赋给三到四组
    theta[3][5] = -acos(A);
    theta[4][5] = theta[3][5];
    //由theta[5][1]产生的第一个解，赋给五到六组
    A = sin(theta[5][1]) * ax - cos(theta[5][1]) * ay;
    theta[5][5] = acos(A);
    theta[6][5] = theta[5][5];
    //由theta[5][1]产生的第二个解，赋给七到八组
    theta[7][5] = -acos(A);
    theta[8][5] = theta[7][5];

    //theta6
    for (int i = 1; i <= 8; i = i + 2) {
        A = sin(theta[i][1]) * nx - cos(theta[i][1]) * ny;
        B = sin(theta[i][1]) * ox - cos(theta[i][1]) * oy;
        theta[i][6] = atan2(A, B) - atan2(theta[i][5], 0);
        theta[i + 1][6] = theta[i][6];
    }


    //theta3、theta2、theta4
    for (int i = 1; i <= 8; i = i + 2) {
        D = d[5] * (sin(theta[i][6]) * (nx * cos(theta[i][1]) + ny * sin(theta[i][1])) + cos(theta[i][6]) * (
                        ox * cos(theta[i][1]) + oy * sin(theta[i][1])))
            - d[6] * (ax * cos(theta[i][1]) + ay * sin(theta[i][1])) + px * cos(theta[i][1]) + py * sin(theta[i][1]);
        E = pz - d[1] - az * d[6] + d[5] * (oz * cos(theta[i][6]) + nz * sin(theta[i][6]));
        theta[i][3] = acos((D * D + E * E - a[2] * a[2] - a[3] * a[3]) / (2 * a[2] * a[3]));
        theta[i + 1][3] = -acos((D * D + E * E - a[2] * a[2] - a[3] * a[3]) / (2 * a[2] * a[3]));
    }

    for (int i = 1; i <= 8; i = i + 1) {
        D = d[5] * (sin(theta[i][6]) * (nx * cos(theta[i][1]) + ny * sin(theta[i][1])) + cos(theta[i][6]) * (
                        ox * cos(theta[i][1]) + oy * sin(theta[i][1])))
            - d[6] * (ax * cos(theta[i][1]) + ay * sin(theta[i][1])) + px * cos(theta[i][1]) + py * sin(theta[i][1]);
        E = pz - d[1] - az * d[6] + d[5] * (oz * cos(theta[i][6]) + nz * sin(theta[i][6]));
        F = ((a[3] * cos(theta[i][3]) + a[2]) * E - a[3] * sin(theta[i][3]) * D) / (
                a[2] * a[2] + a[3] * a[3] + 2 * a[2] * a[3] * cos(theta[i][3]));
        G = (D + a[3] * sin(theta[i][3]) * F) / (a[3] * cos(theta[i][3]) + a[2]);
        theta[i][2] = atan2(F, G);
    }

    for (int i = 1; i <= 8; i = i + 1) {
        M = -sin(theta[i][6]) * (nx * cos(theta[i][1]) + ny * sin(theta[i][1])) - cos(theta[i][6]) * (
                ox * cos(theta[i][1]) + oy * sin(theta[i][1]));
        N = oz * cos(theta[i][6]) + nz * sin(theta[i][6]);
        theta[i][4] = atan2(M, N) - theta[i][2] - theta[i][3];
    }


    for (int i = 1; i <= 8; i++) {
        //cout << "\noptional degree " << i << "\n";
        // TODO 有些电机是可以动到180-360的位置的，可以按需求更改下面的
        theta[i][1] = N2PI_2PI(theta[i][1]);

        for (int j = 2; j <= 6; j++) {
            theta[i][j] = N2PI_2PI(theta[i][j]);
            theta[i][j] = NPI_PI(theta[i][j]);
            //printf("%5.2lf  ", theta[i][j] * 180.0 / EIGEN_PI);
        }


        //cout << "\nAFTER PROCESS\n";
        //for (int j = 1; j <= 6; j++)
        //{
        //	printf("%5.2lf  ", theta[i][j] * 180.0 / EIGEN_PI);
        //}
    }

    vector<int> num;
    for (int i = 1; i <= 8; i++) {
        if (-155 < theta[i][2] * 180.0 / EIGEN_PI && theta[i][2] * 180.0 / EIGEN_PI < 19
            && -130 < theta[i][3] * 180.0 / EIGEN_PI && theta[i][3] * 180.0 / EIGEN_PI < 130) {
            num.push_back(i);
        }
    }

#ifdef _DEBUG_
	cout << "可用的解组数为：" << endl;

	for (int i = 0; i < num.size(); i++)
	{
		cout << num[i] << "   ";
		for (int j = 1; j <= 6; j++)
		{
			cout << "theta" << j << "=" << theta[num[i]][j] * 180.0 / EIGEN_PI << "    ";
		}
		cout << endl;
	}
	cout << endl;
#endif
    if (!num.empty()) {
        vector<double> sums;
        auto now_angle = getAngles();
        for (auto j = 0; j < num.size(); ++j) {
            double tmp = 0;
            for (int i = 0; i < 6; ++i) {
                //printf("%f ", now_angle[i]);
                tmp += std::abs(now_angle[i] - theta[num[j]][i + 1] * 180.0 / EIGEN_PI);
            }
            sums.push_back(tmp);
            cout << "OPTIONAL RE SUM {" << j << "} = " << tmp << "\n";
        }
        //printf("NOW ANGLE:  ");
        int min_idx = min_element(sums.begin(), sums.end()) - sums.begin();
        //printf("\nTARGET ANGLE:  %f %f %f %f %f %f \n\n", theta[num[0]][1] * 180.0 / EIGEN_PI, theta[num[0]][2] * 180.0 / EIGEN_PI, theta[num[0]][3] * 180.0 / EIGEN_PI, theta[num[0]][4] * 180.0 / EIGEN_PI, theta[num[0]][5] * 180.0 / EIGEN_PI, theta[num[0]][6] * 180.0 / EIGEN_PI);
        cout << "SUM: " << sums[min_idx] << "\n";
        if (sums[min_idx] < 100.0 || force_moving == TRUE) {
            setAngles(theta[num[min_idx]][1], theta[num[min_idx]][2], theta[num[min_idx]][3], theta[num[min_idx]][4],
                      theta[num[min_idx]][5], theta[num[min_idx]][6]);
        } else {
            char tmp;
            cout << "Move greatly, if continue? Y/[n]\n";
            cin >> tmp;
            if (tmp == 'y' || tmp == 'Y') {
                setAngles(theta[num[min_idx]][1], theta[num[min_idx]][2], theta[num[min_idx]][3],
                          theta[num[min_idx]][4], theta[num[min_idx]][5], theta[num[min_idx]][6]);
                return 0;
            }
            return FAILED_TO_MOVE; // 强制移动失败
        }
    }
    //std::this_thread::sleep_for(std::chrono::milliseconds(30));
    //ActuatorController::processEvents();

    //pController->requestCVPValue(uIDArray.at(0).actuatorID);
    //pController->requestCVPValue(uIDArray.at(1).actuatorID);
    //pController->requestCVPValue(uIDArray.at(2).actuatorID);
    //pController->requestCVPValue(uIDArray.at(3).actuatorID);
    //pController->requestCVPValue(uIDArray.at(4).actuatorID);
    //pController->requestCVPValue(uIDArray.at(5).actuatorID);

    //cout << "关节1的电流为：" << current[0] << endl;
    //cout << "关节2的电流为：" << current[1] << endl;
    //cout << "关节3的电流为：" << current[2] << endl;
    //cout << "关节4的电流为：" << current[3] << endl;
    //cout << "关节5的电流为：" << current[4] << endl;
    //cout << "关节6的电流为：" << current[5] << endl;

    //if (abs(current[3]) > 0.7 || abs(current[4]) > 0.7)
    //	break;
    return 0;
}

int NE30Control::setInit() const {
    setEncoders(19.3726, // id = 0
                8.035, // id = 1
                5.94, // id = 2
                4.82, // id = 3
                2.659, // id = 4
                12.16 // id = 5
    );
    return 1;
}

NE30Pos NE30Control::get_init_pos() const {
    return init_pos;
}

void NE30Control::set_init_pos(double pos_x, double pos_y, double pos_z, double pitch, double yaw, double roll) {
    init_pos.x = pos_x;
    init_pos.y = pos_y;
    init_pos.z = pos_z;
    init_pos.pitch = pitch;
    init_pos.yaw = yaw;
    init_pos.roll = roll;
}

Eigen::Matrix4d NE30Control::getPosMatrix() const {
    double theta1_0 = -10 * pController->getPosition(uIDArray.at(0).actuatorID, true, uIDArray.at(0).ipAddress) / 180.0
                      *
                      EIGEN_PI;
    double theta2_0 = -10 * pController->getPosition(uIDArray.at(1).actuatorID, true, uIDArray.at(1).ipAddress) / 180.0
                      *
                      EIGEN_PI;
    double theta3_0 = 10 * pController->getPosition(uIDArray.at(2).actuatorID, true, uIDArray.at(2).ipAddress) / 180.0
                      *
                      EIGEN_PI;
    double theta4_0 = -10 * pController->getPosition(uIDArray.at(3).actuatorID, true, uIDArray.at(3).ipAddress) / 180.0
                      *
                      EIGEN_PI;
    double theta5_0 = -10 * pController->getPosition(uIDArray.at(4).actuatorID, true, uIDArray.at(4).ipAddress) / 180.0
                      *
                      EIGEN_PI;
    double theta6_0 = -10 * pController->getPosition(uIDArray.at(5).actuatorID, true, uIDArray.at(5).ipAddress) / 180.0
                      *
                      EIGEN_PI;

    double theta_input[6 + 1] = {0, theta1_0, theta2_0, theta3_0, theta4_0, theta5_0, theta6_0};
    Eigen::Matrix4d T[6 + 1];
    for (int i = 1; i < 7; i++) {
        T[i](0, 0) = cos(theta_input[i]);
        T[i](0, 1) = -sin(theta_input[i]) * cos(alpha[i]);
        T[i](0, 2) = sin(theta_input[i]) * sin(alpha[i]);
        T[i](0, 3) = a[i] * cos(theta_input[i]);
        T[i](1, 0) = sin(theta_input[i]);
        T[i](1, 1) = cos(theta_input[i]) * cos(alpha[i]);
        T[i](1, 2) = -cos(theta_input[i]) * sin(alpha[i]);
        T[i](1, 3) = a[i] * sin(theta_input[i]);
        T[i](2, 0) = 0;
        T[i](2, 1) = sin(alpha[i]);
        T[i](2, 2) = cos(alpha[i]);
        T[i](2, 3) = d[i];
        T[i](3, 0) = 0;
        T[i](3, 1) = 0;
        T[i](3, 2) = 0;
        T[i](3, 3) = 1;
    }
    Eigen::Matrix4d T06 = T[1] * T[2] * T[3] * T[4] * T[5] * T[6];
    return T06;
}

NE30Pos NE30Control::getPos() const {
    Eigen::Matrix4d pos_mat = getPosMatrix();
    // 提取旋转矩阵
    Eigen::Matrix3d rot_mat = pos_mat.block<3, 3>(0, 0);

    // 提取平移向量
    Eigen::Vector3d trans_vec = pos_mat.block<3, 1>(0, 3);

    // 计算欧拉角 (需要指定旋转顺序)
    Eigen::Vector3d eulerAngles = rot_mat.eulerAngles(2, 1, 0); // ZYX 顺序
    // 输出欧拉角
    // std::cout << "Euler angles (Yaw, Pitch, Roll): " << std::endl;
    // std::cout << "Yaw (Z axis): " << eulerAngles[0] * 180 / M_PI << " degrees" << std::endl;
    // std::cout << "Pitch (Y axis): " << eulerAngles[1] * 180 / M_PI << " degrees" << std::endl;
    // std::cout << "Roll (X axis): " << eulerAngles[2] * 180 / M_PI << " degrees" << std::endl;

    return NE30Pos(1000.0 * trans_vec[0], 1000.0 * trans_vec[1], 1000.0 * trans_vec[2], eulerAngles[1], eulerAngles[0], eulerAngles[2]);
}

void NE30Control::setAngles(const double r1,
                            const double r2,
                            const double r3,
                            const double r4,
                            const double r5,
                            const double r6) const {
    pController->setPosition(uIDArray.at(0).actuatorID, -r1 * 180.0 / (EIGEN_PI * 10), uIDArray.at(0).ipAddress);
    pController->setPosition(uIDArray.at(1).actuatorID, -r2 * 180.0 / (EIGEN_PI * 10), uIDArray.at(1).ipAddress);
    pController->setPosition(uIDArray.at(2).actuatorID, r3 * 180.0 / (EIGEN_PI * 10), uIDArray.at(2).ipAddress);
    pController->setPosition(uIDArray.at(3).actuatorID, -r4 * 180.0 / (EIGEN_PI * 10), uIDArray.at(3).ipAddress);
    pController->setPosition(uIDArray.at(4).actuatorID, -r5 * 180.0 / (EIGEN_PI * 10), uIDArray.at(4).ipAddress);
    pController->setPosition(uIDArray.at(5).actuatorID, -r6 * 180.0 / (EIGEN_PI * 10), uIDArray.at(5).ipAddress);
}

void NE30Control::setAngles(const std::vector<double> &angles) const {
    if (angles.size() != 6) {
        std::cerr << "Wrong number of encoders in NE30Control::setEncoders" << std::endl;
    }
    for (int i = 0; i < angles.size(); i++) {
        double k = -1.0;
        if (i == 2) {
            k = 1.0;
        }
        pController->setPosition(uIDArray.at(i).actuatorID,
                                 k * angles.at(i) * 180.0 / (EIGEN_PI * 10),
                                 uIDArray.at(i).ipAddress);
    }
}

void NE30Control::setEncoders(const double e1,
                              const double e2,
                              const double e3,
                              const double e4,
                              const double e5,
                              const double e6) const {
    vector<double> encoders = {e1, e2, e3, e4, e5, e6};
    setEncoders(encoders);
}

void NE30Control::setEncoders(const std::vector<double> &encoders) const {
    if (encoders.size() != 6) {
        std::cerr << "Wrong number of encoders in NE30Control::setEncoders" << std::endl;
    }
    for (int i = 0; i < encoders.size(); i++) {
        pController->setPosition(uIDArray.at(i).actuatorID, encoders.at(i), uIDArray.at(i).ipAddress);
    }
}

std::vector<double> NE30Control::getAngles() const {
    double theta1 = -10 * pController->getPosition(uIDArray.at(0).actuatorID, true, uIDArray.at(0).ipAddress);
    double theta2 = -10 * pController->getPosition(uIDArray.at(1).actuatorID, true, uIDArray.at(1).ipAddress);
    double theta3 = 10 * pController->getPosition(uIDArray.at(2).actuatorID, true, uIDArray.at(2).ipAddress);
    double theta4 = -10 * pController->getPosition(uIDArray.at(3).actuatorID, true, uIDArray.at(3).ipAddress);
    double theta5 = -10 * pController->getPosition(uIDArray.at(4).actuatorID, true, uIDArray.at(4).ipAddress);
    double theta6 = -10 * pController->getPosition(uIDArray.at(5).actuatorID, true, uIDArray.at(5).ipAddress);
    vector<double> tmp = {theta1, theta2, theta3, theta4, theta5, theta6};
    cout << "\nCurrent motor angles" << endl;
    cout << -theta1 << "   " << -theta2 << "   " << theta3 << "   " << -theta4 << "   " << -theta5 << "   " << -theta6
            << endl << endl;
    return tmp;
}

void NE30Control::paramFeedback(const ActuatorController::UnifiedID &uID, const uint8_t paramType,
                                const double paramValue) {
    switch (paramType) {
        case Actuator::ACTUAL_CURRENT:
            //cout << "Actuator " << (int)uID.actuatorID << " current is " << paramValue << "A" << endl;
            switch ((int) uID.actuatorID) {
                case 1:
                    current[0] = paramValue;
                    break;
                case 2:
                    current[1] = paramValue;
                    break;
                case 3:
                    current[2] = paramValue;
                    break;
                case 4:
                    current[3] = paramValue;
                    break;
                case 5:
                    current[4] = paramValue;
                    break;
                case 6:
                    current[5] = paramValue;
                    break;
                default:
                    break;
            }
            break;
        case Actuator::ACTUAL_POSITION:
            //cout << "Actuator " << (int)uID.actuatorID << " position is " << paramValue << "R" << endl;
            switch ((int) uID.actuatorID) {
                case 1:
                    position[0] = paramValue;
                    break;
                case 2:
                    position[1] = paramValue;
                    break;
                case 3:
                    position[2] = paramValue;
                    break;
                case 4:
                    position[3] = paramValue;
                    break;
                case 5:
                    position[4] = paramValue;
                    break;
                case 6:
                    position[5] = paramValue;
                    break;
                default:
                    break;
            }
            break;
        case Actuator::ACTUAL_VELOCITY:
            //cout << "Actuator " << (int)uID.actuatorID << " velocity is " << paramValue << "RPM" << endl;
            switch ((int) uID.actuatorID) {
                case 1:
                    velocity[0] = paramValue;
                    break;
                case 2:
                    velocity[1] = paramValue;
                    break;
                case 3:
                    velocity[2] = paramValue;
                    break;
                case 4:
                    velocity[3] = paramValue;
                    break;
                case 5:
                    velocity[4] = paramValue;
                    break;
                case 6:
                    velocity[5] = paramValue;
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

void NE30Control::setDisable() const {
    cout << "about to deactivate all" << endl;
    system("pause");

    setEncoders(true_end_encoders);
    Sleep(1000);

    setEncoders(true_mid_encoders);
    Sleep(1000);

    setEncoders(true_start_encoders);
    Sleep(1000);
    // cout << "init" << endl;
    // 失能所有执行器
    if (pController->disableAllActuators()) {
        cout << "all actuators have been disabled successfully! " << endl;
    }
}


#endif // !NE30_CONTROL_HPP
