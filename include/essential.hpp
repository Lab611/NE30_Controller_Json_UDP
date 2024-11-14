#ifndef ESSENTIAL_HPP
#define ESSENTIAL_HPP
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
#include <conio.h> // Keyborad
//#pragma comment(lib, "ws2_32.lib")
#define FAILED_TO_MOVE -1
#define NPI_PI(x) ((x)<(-EIGEN_PI))?((x)+2*EIGEN_PI):(((x)>EIGEN_PI)?(x-(2*EIGEN_PI)):(x))
#define N2PI_2PI(x) ((x)<(-2*EIGEN_PI))?((x)+2*EIGEN_PI):(((x)>2*EIGEN_PI)?(x-(2*EIGEN_PI)):(x))



#endif // !ESSENTIAL_HPP
