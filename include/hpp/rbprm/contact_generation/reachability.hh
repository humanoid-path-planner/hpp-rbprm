#ifndef HPP_RBPRM_REACHABILITY_HH
#define HPP_RBPRM_REACHABILITY_HH

#include <hpp/rbprm/rbprm-limb.hh>
#include <centroidal-dynamics-lib/centroidal_dynamics.hh>

namespace hpp {
  namespace rbprm {
    namespace reachability{


std::pair<MatrixXX, MatrixXX> stackConstraints(const std::pair<MatrixXX, MatrixXX>& Ab,const std::pair<MatrixXX, MatrixXX>& Cd);

bool intersectionExist(const std::pair<MatrixXX, MatrixXX>& Ab, const fcl::Vec3f& c0,const fcl::Vec3f& c1, fcl::Vec3f c_out);

std::pair<MatrixXX, MatrixXX> computeStabilityConstraints(const centroidal_dynamics::Equilibrium& contactPhase);

}
}
}
#endif // REACHABILITY_HH
