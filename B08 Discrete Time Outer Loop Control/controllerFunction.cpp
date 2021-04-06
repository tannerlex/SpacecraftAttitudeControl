#include "./main.hpp"
#include "./receiveUdp.hpp"
#include "./sendUdp.hpp"
#include <cstring>
#include "./Attitudes.hpp"
#include <eigen3/Eigen/Dense>
using namespace Eigen;



static const DCM A_PB((Matrix3d() <<
   0.008031085662960,  -0.073245535352114,   0.997281601762539,
   0.012127083086920,   0.997247555958304,   0.073145375732109,
  -0.999894212163906,   0.011506680067277,   0.008897235242039
).finished());

static const DCM A_BP((Matrix3d() <<
   0.008031085662960,   0.012127083086920,  -0.999894212163906,
  -0.073245535352114,   0.997247555958304,   0.011506680067277,
   0.997281601762539,   0.073145375732109,   0.008897235242039
).finished());

static const DCM J_C_B((Matrix3d() <<
   0.039940934349015,   0.000002112428714,  -0.000264398339716,
   0.000002112428714,   0.038351474350995,   0.002296039610626,
  -0.000264398339716,   0.002296039610626,   0.007260057763141
).finished());

static const Vector3d Kd(
  0.2461670000000000,   1.337558000000000,   1.386980000000000
);

static const Vector3d wbi0_B(0,0,0);
// static const Vector3d wbistar_B(0,0,0);
static const Quat q0_BI(1,0,0,0);

void controllerFunction()
{
  Vector3d wbi_B, wbi_P, wbistar_B, wbistar_P, tc_B, tc_P, h_B, tl_B, t_B;
  Quat q_BI, qstar_BI;
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
    q_BI.s = receiveBuffer[3];
    q_BI.v(0) = receiveBuffer[4];
    q_BI.v(1) = receiveBuffer[5];
    q_BI.v(2) = receiveBuffer[6];
    qstar_BI.s = receiveBuffer[7];
    qstar_BI.v(0) = receiveBuffer[8];
    qstar_BI.v(1) = receiveBuffer[9];
    qstar_BI.v(2) = receiveBuffer[10];
    time = receiveBuffer[11];
    // wbistar_B(0) = receiveBuffer[3];
    // wbistar_B(1) = receiveBuffer[4];
    // wbistar_B(2) = receiveBuffer[5];

    /* unlock mutex to allow buffer access elsewhere */
    receiveLock.unlock();

    /* calculate the control commands */
    wbistar_P = A_PB * wbistar_B;
    wbi_P = A_PB * wbi_B;
    tc_P(0) = Kd(0) * (wbistar_P(0) - wbi_P(0));
    tc_P(1) = Kd(1) * (wbistar_P(1) - wbi_P(1));
    tc_P(2) = Kd(2) * (wbistar_P(2) - wbi_P(2));
    tc_B = A_BP * tc_P;
    h_B = J_C_B * wbi_B;
    tl_B = wbi_B.cross(h_B);
    t_B = tc_B + tl_B;

    /* send the data off */
    std::unique_lock<std::mutex> sendLock(sendMutex);
    sendBuffer[0] = t_B(0);
    sendBuffer[1] = t_B(1);
    sendBuffer[2] = t_B(2);
    sendBuffer[3] = time;
    sendLock.unlock();
    sendCv.notify_one();
  } /* end while (true) loop */
} /* controllerFunction() */
