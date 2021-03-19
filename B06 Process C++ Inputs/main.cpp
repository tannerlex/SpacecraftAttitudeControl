#include "ProcessArgv.hpp"
#include <iostream>

int main(const int argc, const char* argv[])
{
  ProcessArgv p(argc, argv);
  std::cout << "Number of arguments: " << p.m_size << std::endl;

  std::cout << std::endl << "Input strings are:" << std::endl;
  for (int i = 0; i < p.m_size; i++)
  {
    std::cout << p.getString(i) << std::endl;
  }
  std::cout << std::endl << "Input numbers are:" << std::endl;
  for (int i = 0; i < p.m_size; i++)
  {
    std::cout << p.getDouble(i) << std::endl;
  }
  std::cout << std::endl;

  std::cout << "Exiting main()" << std::endl;
}
