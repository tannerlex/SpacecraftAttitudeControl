/** 
 *  @file    receiveUdp.hpp
 *  @author  Bryan Bingham
 *  @date    03/18/2021
 *  @version 1.0.4
 *  
 *  @brief Receive data using UDP Protocol
 *
 *  @section DESCRIPTION
 *  
 *  This class creates a socket and will wait to receive a specified number of doubles
 *  from the socket
 *
 */

#ifndef RECEIVEUDP_HPP_
#define RECEIVEUDP_HPP_

#include <stdint.h>
#include <string>

/**
*  @brief Class to receive UDP data
*/
class ReceiveUdp
{
public:

  /** 
  *   @brief  Constructor which sets the socket file descriptor to uninitialized
  *
  */
  ReceiveUdp();

  /** 
  *   @brief  Destructor which closes the socket if initialized
  *
  */
  ~ReceiveUdp();

  /** 
  *   @brief  Initializes the socket to receive UDP data on specific IP and port
  *  
  *   @param  localIp is the IP address to receive data on
  *   @param  receivePort is the UDP port to receive data on
  *   @return std::string error message on failure
  */
  std::string initialize(const char* localIp, const uint16_t receivePort);

  /** 
  *   @brief  Waits until specified number of doubles are received
  *  
  *   @param  buffer is a pointer to a buffer where the received data will be placed
  *   @param  number is the number of doubles to receive
  *   @return const char* error message on failure
  */
  std::string receiveDoubles(double* buffer, const int number) const;

private:
  int m_socketFd; ///< File descriptor of the socket
};

#endif