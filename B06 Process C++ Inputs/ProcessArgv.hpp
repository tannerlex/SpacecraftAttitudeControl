#ifndef PROCESSARGV_HPP
#define PROCESSARGV_HPP

#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <vector>

class ProcessArgv
{
public:
  ProcessArgv() = default;
  ProcessArgv(const int num): m_size(num) {};
  ProcessArgv(const int argc, const char* argv[]);

  double getDouble(const int idx); /* provides access to m_values */
  std::string getString(const int idx); /* access to m_strings */
  const int m_size; 

private:
  bool inBounds(int idx); /* bounds checking for accessing vector values */
  std::vector <std::string> m_strings; /* container for arg strings */
  std::vector <double> m_values; /* container for input numbers */
};

#endif /* PROCESSARGV_HPP */
