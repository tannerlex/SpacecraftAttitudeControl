#ifndef MAIN_HPP_
#define MAIN_HPP_

#include <mutex>
#include <condition_variable>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

typedef Matrix3d DCM; /* Container to hold a DCM */

const int RCVNUM = 12;
const int SNDNUM = 7;

const int RCVPORT = 12345;
const char* const LOCALIP = "172.31.220.90";

const int SNDPORT = 12346;
const char* const REMOTEIP = "172.31.12.78";

void sendFunction();
void receiveFunction();
void controllerFunction();

extern double sendBuffer[SNDNUM];
extern std::mutex sendMutex;
extern std::condition_variable sendCv;

extern double receiveBuffer[RCVNUM];
extern std::mutex receiveMutex;
extern std::condition_variable receiveCv;

#endif
