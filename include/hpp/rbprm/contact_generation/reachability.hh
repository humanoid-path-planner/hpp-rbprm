#ifndef HPP_RBPRM_REACHABILITY_HH
#define HPP_RBPRM_REACHABILITY_HH

#include <hpp/rbprm/rbprm-limb.hh>
#include <centroidal-dynamics-lib/centroidal_dynamics.hh>
#include <hpp/rbprm/rbprm-state.hh>
namespace hpp {
  namespace rbprm {

  HPP_PREDEF_CLASS(RbPrmFullBody);
  class RbPrmFullBody;
  typedef boost::shared_ptr <RbPrmFullBody> RbPrmFullBodyPtr_t;


    namespace reachability{


std::pair<MatrixXX, VectorX> stackConstraints(const std::pair<MatrixXX, VectorX> &Ab, const std::pair<MatrixXX, VectorX> &Cd);

bool intersectionExist(const std::pair<MatrixXX, VectorX>& Ab, const fcl::Vec3f& c0,const fcl::Vec3f& c1, fcl::Vec3f c_out);

std::pair<MatrixXX, VectorX> computeStabilityConstraints(const centroidal_dynamics::Equilibrium& contactPhase,const fcl::Vec3f& int_point = fcl::Vec3f(0,0,0));

std::pair<MatrixXX, VectorX> computeStabilityConstraints(const RbPrmFullBodyPtr_t& fullbody, State &state);



}
}
}
#endif // REACHABILITY_HH
