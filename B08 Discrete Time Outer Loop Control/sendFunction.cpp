#include "./main.hpp"
#include "./sendUdp.hpp"
#include <cstring>

void sendFunction()
{
  double localBuffer[SNDNUM];
  SendUdp sendUdp; /* instance of object to send UDP data */
  sendUdp.initialize(LOCALIP, REMOTEIP, SNDPORT);

  while(true)
  {

    /* lock to protect the buffer from being accessed elsewhere */
    std::unique_lock<std::mutex> sendLock(sendMutex);

    /* wait until data is ready to be sent */
    sendCv.wait(sendLock);
    
    /* copy data from global to local send buffer */
    std::memcpy(localBuffer, sendBuffer, sizeof(sendBuffer));

    sendLock.unlock(); /* free up the buffer */

    sendUdp.sendDoubles(localBuffer, SNDNUM); /* send data */

  } /* end infinite loop */
} /* sendFunction() */
