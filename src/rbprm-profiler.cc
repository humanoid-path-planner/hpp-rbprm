

#include "hpp/rbprm/rbprm-profiler.hh"

using std::map;
using std::ostringstream;
using std::string;

RbPrmProfiler& getRbPrmProfiler() {
  static RbPrmProfiler s(REAL_TIME);  // alternatives are CPU_TIME and REAL_TIME
  return s;
}
