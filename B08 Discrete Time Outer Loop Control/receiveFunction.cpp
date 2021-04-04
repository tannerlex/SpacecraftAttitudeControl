#include "./main.hpp"
#include "./receiveUdp.hpp"
#include <cstring>

void receiveFunction()
{
  double localBuffer[RCVNUM];
  ReceiveUdp receiveUdp; /* instance of object to recieve UDP data */
  receiveUdp.initialize(LOCALIP, RCVPORT);

  while(true)
  {
    /* collect UDP data into local buffer */
    receiveUdp.receiveDoubles(localBuffer, RCVNUM);

    /* protect the buffer from being accessed elsewhere */
    std::unique_lock<std::mutex> receiveLock(receiveMutex);

    /* copy data from local into the global receive buffer */
    std::memcpy(receiveBuffer, localBuffer, sizeof(receiveBuffer));

    receiveLock.unlock(); /* free up the buffer */

    receiveCv.notify_one(); /* tell controller data is available */
  } /* end infinite loop */
} /* recieveFunction() */
