#include "ProcessArgv.hpp"
#include <cmath>
#include <stdlib.h>

ProcessArgv::ProcessArgv(const int argc, const char* argv[]): m_size(argc)
{
  for (int i = 0; i < m_size; i++) /* for each command line argument */
  {
    std::string str(argv[i]); /* temp string for adding inputs to member vars*/
    m_strings.push_back(str); /* add string to member container */
    m_values.push_back(atof(argv[i])); /* add floating point number to values*/
  }
}

double ProcessArgv::getDouble(const int idx)
{
  if (inBounds(idx)) /* ensure valid index */
  {
    return m_values[idx];
  }
  else
  {
    return nan("0x1"); /* return an invalid number */
  }
}

std::string ProcessArgv::getString(const int idx)
{
  if (inBounds(idx)) /* ensure valid index */
  {
    return m_strings[idx];
  }
  else
  {
    return "ERROR calling getString()";
  }
}

bool ProcessArgv::inBounds(int idx)
{
  if (idx >= m_size)
  {
    /* print out error message */
    std::cout << "ERROR: out of bounds read attempted! Index: ";
    std::cout << idx << " is >= Size: " << m_size << std::endl;
  }
  return idx < m_size;
}
