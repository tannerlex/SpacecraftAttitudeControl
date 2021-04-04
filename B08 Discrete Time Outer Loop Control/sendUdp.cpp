/** 
 *  @file    sendUdp.cpp
 *  @author  Bryan Bingham
 *  @date    03/18/2021
 *  @version 1.0.4
 *  
 *  @brief Send data using UDP Protocol
 *
 *  @section DESCRIPTION
 *  
 *  This class creates a socket and will send a specified number of doubles
 *  to the socket
 *
 */

#include "sendUdp.hpp"
#include <cstring> // memset
#include <cerrno> // errno
#include <unistd.h> // close
#include <sstream> // error stream creation

SendUdp::SendUdp() :
m_socketFd(-1)
{
}

SendUdp::~SendUdp()
{
  if (m_socketFd != -1)
  {
    close(m_socketFd);
  }
}

std::string SendUdp::initialize(const char* localIp, const char* remoteIp, const uint16_t sendPort)
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
    return error.str();
  }

  // Setup sockaddr_in structure (TLPI pg 1202)
  memset(&m_sockaddr_in, 0, sizeof(struct sockaddr_in));
  m_sockaddr_in.sin_family = AF_INET;
  m_sockaddr_in.sin_addr = in_addr;
  m_sockaddr_in.sin_port = htons(sendPort);

  // Bind Socket to Address described by sockaddr_in (TLPI pg 1153)
  if (bind(m_socketFd, (struct sockaddr *) &m_sockaddr_in, sizeof(struct sockaddr_in)) == -1)
  {
    error << "bind(): " << std::strerror(errno);
    return error.str();
  }

  // Convert Presentation IP to Network (TLPI pg 1206)
  retVal = inet_pton(AF_INET, remoteIp, &in_addr);
  if (retVal == -1)
  {
    error << "inet_pton(): " << std::strerror(errno);
    return error.str();
  }
  else if (retVal == 0)
  {
    error << "inet_pton(): " << remoteIp << " is not a valid network address";
    return error.str();
  }

  // Setup sockaddr_in structure (TLPI pg 1202)
  memset(&m_sockaddr_in, 0, sizeof(struct sockaddr_in));
  m_sockaddr_in.sin_family = AF_INET;
  m_sockaddr_in.sin_addr = in_addr;
  m_sockaddr_in.sin_port = htons(sendPort);

  m_len = sizeof(struct sockaddr_in);

  return error.str();
}

std::string SendUdp::sendDoubles(const double* buffer, const int number) const
{
  if (m_socketFd == -1)
  {
    return "sendUdp class has not been initialized";
  }

  std::stringstream error;
  int size = 8*number;

  // Send datagrams to socket (TLPI PG 1161)
  if (sendto(m_socketFd, buffer, size, 0, (struct sockaddr*)&m_sockaddr_in, m_len) != size)
  {
    error << "sendto(): " << std::strerror(errno);
    return error.str();
  }

  return error.str();
}
