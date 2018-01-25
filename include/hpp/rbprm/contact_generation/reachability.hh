#ifndef HPP_RBPRM_REACHABILITY_HH
#define HPP_RBPRM_REACHABILITY_HH

#include <hpp/rbprm/rbprm-limb.hh>

namespace hpp {
  namespace rbprm {
    namespace reachability{


std::pair<MatrixXX, MatrixXX> stackConstraints(const std::pair<MatrixXX, MatrixXX>& Ab,const std::pair<MatrixXX, MatrixXX>& Cd);



}
}
}
#endif // REACHABILITY_HH
