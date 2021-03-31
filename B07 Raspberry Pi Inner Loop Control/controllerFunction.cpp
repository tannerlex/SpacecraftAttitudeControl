#include "./main.hpp"
#include "./receiveUdp.hpp"
#include "./sendUdp.hpp"
#include <cstring>
#include <eigen3/Eigen/Dense>
using namespace Eigen;



// static const DCM A_PB((Matrix3d() <<
//    0.008031085662960,  -0.073245535352114,   0.997281601762539,
//    0.012127083086920,   0.997247555958304,   0.073145375732109,
//   -0.999894212163906,   0.011506680067277,   0.008897235242039
// ).finished());

static const DCM A_BP((Matrix3d() <<
   0.008031085662960,   0.012127083086920,  -0.999894212163906,
  -0.073245535352114,   0.997247555958304,   0.011506680067277,
   0.997281601762539,   0.073145375732109,   0.008897235242039
).finished());

static const Vector3d Kd(
  0.2461670000000000,   1.337558000000000,   1.386980000000000
);

static const Vector3d wbi0_B(0,0,0);
static const Vector3d wbistar_B(0,0,0);

void controllerFunction()
{
  Vector3d wbi_B, wbi_P, wbistar_B, wbistar_P, tc_B, tc_P, h_B, tl_B, t_B;
  double time;

  while (true)
  {
    /* lock to protect the buffer from being accessed elsewhere */
    std::unique_lock<std::mutex> receiveLock(receiveMutex);

    /* wait for data to be ready */
    receiveCv.wait(receiveLock);

    /* copy data from buffer */
    wbi_B(0) = receiveBuffer[0];
    wbi_B(1) = receiveBuffer[1];
    wbi_B(2) = receiveBuffer[2];
    wbistar_B(0) = receiveBuffer[3];
    wbistar_B(1) = receiveBuffer[4];
    wbistar_B(2) = receiveBuffer[5];
    time = receiveBuffer[6];

    /* unlock mutex to allow buffer access elsewhere */
    receiveLock.unlock();
  }
}
