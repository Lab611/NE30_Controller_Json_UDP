#ifndef NE30_POS_HPP
#define NE30_POS_HPP
#include <fstream>
#include <signal.h>
#include <cmath>
#include <Eigen/Dense>
#include <vector>

//#pragma comment(lib, "ws2_32.lib")
#define FAILED_TO_MOVE -1
#define NPI_PI(x) ((x)<(-EIGEN_PI))?((x)+2*EIGEN_PI):(((x)>EIGEN_PI)?(x-(2*EIGEN_PI)):(x))
#define N2PI_2PI(x) ((x)<(-2*EIGEN_PI))?((x)+2*EIGEN_PI):(((x)>2*EIGEN_PI)?(x-(2*EIGEN_PI)):(x))

using namespace std;
/// <summary>
/// pitch 对应 y
/// yaw 对应 z
/// roll 对应 x
/// </summary>
class NE30Pos {
public:
    // Ne30Pos();
    explicit NE30Pos(double x_ = 0, double y_ = 0, double z_ = 0, double pitch_ = 0, double yaw_ = 0, double roll_ = 0);

    explicit NE30Pos(const vector<double> &pos_);

    // ~Ne30Pos();
    double x, y, z, pitch, yaw, roll;

    bool operator==(const NE30Pos pos) const {
        return (this->x == pos.x &&
                this->y == pos.y &&
                this->z == pos.z &&
                this->pitch == pos.pitch &&
                this->yaw == pos.yaw &&
                this->roll == pos.roll);
    }

    bool operator!=(const NE30Pos pos) const {
        return !(this->x == pos.x &&
                 this->y == pos.y &&
                 this->z == pos.z &&
                 this->pitch == pos.pitch &&
                 this->yaw == pos.yaw &&
                 this->roll == pos.roll);
    }

    NE30Pos operator+(const NE30Pos &pos) const {
        NE30Pos re;
        re.x = this->x + pos.x;
        re.y = this->y + pos.y;
        re.z = this->z + pos.z;
        re.pitch = this->pitch + pos.pitch;
        re.yaw = this->yaw + pos.yaw;
        re.roll = this->roll + pos.roll;
        return re;
    }

    NE30Pos operator-(const NE30Pos &pos) const {
        NE30Pos re;
        re.x = this->x - pos.x;
        re.y = this->y - pos.y;
        re.z = this->z - pos.z;
        re.pitch = this->pitch - pos.pitch;
        re.yaw = this->yaw - pos.yaw;
        re.roll = this->roll - pos.roll;
        return re;
    }

    void printInfo() const {
        printf("x: %5.2f y : %5.2f z : %5.2f \n", x, y, z);
        printf("pitch: %5.2f yaw: %5.2f roll: %5.2f\n", pitch, yaw,roll);
        printf("%7.4f %7.4f %7.4f %7.4f %7.4f %7.4f\n", x, y, z, pitch, yaw, roll);
    }

private:
};

inline NE30Pos::NE30Pos(const double x_,
                        const double y_,
                        const double z_,
                        const double pitch_,
                        const double yaw_,
                        const double roll_) {
    x = x_;
    y = y_;
    z = z_;
    pitch = pitch_;
    yaw = yaw_;
    roll = roll_;
}

inline NE30Pos::NE30Pos(const vector<double> &pos_) {
    if (pos_.size() == 6) {
        x = pos_.at(0);
        y = pos_.at(1);
        z = pos_.at(2);
        pitch = pos_.at(3);
        yaw = pos_.at(4);
        roll = pos_.at(5);
    } else {
        x = 300;
        y = 165;
        z = 300;
        pitch = -0.0698;
        yaw = 0;
        roll = -2.1311;
    }
}

#endif // !NE30_POS_HPP
