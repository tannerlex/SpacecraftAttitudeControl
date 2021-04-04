/** 
 *  @file    receiveUdp.cpp
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

#include "receiveUdp.hpp"
#include <arpa/inet.h> // sockaddr_in, in_addr, inet_pton, htons
#include <cstring> // memset
#include <cerrno> // errno
#include <unistd.h> // close
#include <sstream> // error stream creation

ReceiveUdp::ReceiveUdp() :
m_socketFd(-1)
{
}

ReceiveUdp::~ReceiveUdp()
{
  if (m_socketFd != -1)
  {
    close(m_socketFd);
  }
}

std::string ReceiveUdp::initialize(const char* localIp, const uint16_t receivePort)
{
  std::stringstream error;

  // Create a datagram socket (TLPI pg 1153)
  m_socketFd = socket(AF_INET, SOCK_DGRAM, 0);
  if (m_socketFd == -1)
  {
    error << "socket(): " << std::strerror(errno);
    return error.str();
  }

  // IPv4 soccket address structure (TLPI pg 1202)
  struct in_addr in_addr;

  // Convert Presentation IP to Network (TLPI pg 1206)
  int retVal = inet_pton(AF_INET, localIp, &in_addr);
  if (retVal == -1)
  {
    error << "inet_pton(): " << std::strerror(errno);
    return error.str();
  }
  else if (retVal == 0)
  {
    error << "inet_pton(): " << localIp << " is not a valid network address";
    return error.str();;
  }

  struct sockaddr_in sockaddr_in; // struct sockaddr for udp socket

  // Setup sockaddr_in structure (TLPI pg 1202)
  memset(&sockaddr_in, 0, sizeof(struct sockaddr_in));
  sockaddr_in.sin_family = AF_INET;
  sockaddr_in.sin_addr = in_addr;
  sockaddr_in.sin_port = htons(receivePort);

  // Bind Socket to Address described by sockaddr_in (TLPI pg 1153)
  if (bind(m_socketFd, (struct sockaddr *) &sockaddr_in, sizeof(struct sockaddr_in)) == -1)
  {
    error << "bind(): " << std::strerror(errno);
    return error.str();
  }
  return error.str();
}

std::string ReceiveUdp::receiveDoubles(double* buffer, const int number) const
{
  if (m_socketFd == -1)
  {
    return "receiveUdp class has not been initialized";
  }

  std::stringstream error;
  int size = 8*number;

  // Receive datagrams from socket (TLPI PG 1161)
  if (recvfrom(m_socketFd, buffer, size, 0, NULL, NULL) != size)
  {
    error << "recvfrom(): " << std::strerror(errno);
    return error.str();
  }

  return error.str();
}
