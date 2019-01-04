

#include "hpp/rbprm/rbprm-profiler.hh"

using std::map;
using std::string;
using std::ostringstream;

RbPrmProfiler& getRbPrmProfiler()
{
  static RbPrmProfiler s(REAL_TIME);   // alternatives are CPU_TIME and REAL_TIME
  return s;
}
