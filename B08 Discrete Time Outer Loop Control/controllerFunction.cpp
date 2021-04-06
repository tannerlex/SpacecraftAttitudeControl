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

static const double K = 5.138451736541828;
static const double Zd = 0.999939997380546;
static const double Pd = 0.500999395848231;

static const Vector3d wbi0_B(0,0,0);

void controllerFunction()
{
  DCM A_PB_copy(A_PB);
  Quat q_PB(Attitudes::dcm2quat(A_PB_copy));
  double time;
  Eigen::Vector3d wbi_B, wbi_P, wbistar_B, wbistar_P, tc_B, tc_P,
                  h_B, tl_B, t_B, theta, thz1, thz2, wz1, wz2;
  Quat q_BI, qstar_BI, q_IP, qstar_PI, qe;
  EulAxisAngle e;

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

    /* unlock mutex to allow buffer access elsewhere */
    receiveLock.unlock();

    /* calculate the error quaternion */
    q_IP = Attitudes::quatProdX(q_PB,q_BI); /* this is really q_PI */
    q_IP = Attitudes::quatConjugate(q_IP); /* now this is q_IP */
    qstar_PI = Attitudes::quatProdX(qstar_BI, q_PB);
    qe = Attitudes::quatProdX(qstar_PI,q_IP);

    /* calculate the controller input */
    e = Attitudes::quat2eAxisAngle(qe);
    theta = e.angle*e.axis;

    /* implement difference equation of outer loop controller */
    wbistar_P = K*thz1 - K*Zd*thz2 + (1+Pd)*wz1 - Pd*wz2;

    /* calculate the control commands */
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

    /* update the memory values */
    thz2 = thz1;
    thz1 = theta;
    wz2 = wz1;
    wz1 = wbistar_P;
  } /* end while (true) loop */
} /* controllerFunction() */
