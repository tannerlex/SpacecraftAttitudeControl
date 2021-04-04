#include "./main.hpp"
#include <thread>

double sendBuffer[SNDNUM];
std::mutex sendMutex;
std::condition_variable sendCv;

double receiveBuffer[RCVNUM];
std::mutex receiveMutex;
std::condition_variable receiveCv;

int main(int argc, char* argv[])
{
  std::thread sendThread(sendFunction);
  std::thread controllerThread(controllerFunction);
  std::thread receiveThread(receiveFunction);

  sendThread.join();
  receiveThread.join();
  controllerThread.join();

  return EXIT_SUCCESS;
}
