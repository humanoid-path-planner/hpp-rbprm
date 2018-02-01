#ifndef HPP_RBPRM_KINEMATICS_CONSTRAINTS_HH
#define HPP_RBPRM_KINEMATICS_CONSTRAINTS_HH

#include <hpp/rbprm/rbprm-limb.hh>
#include <hpp/rbprm/rbprm-state.hh>
#include <hpp/model/configuration.hh>
#include <hpp/rbprm/rbprm-state.hh>
namespace hpp {
  namespace rbprm {

  HPP_PREDEF_CLASS(RbPrmFullBody);
  class RbPrmFullBody;
  typedef boost::shared_ptr <RbPrmFullBody> RbPrmFullBodyPtr_t;

  namespace reachability{



std::pair<MatrixX3, MatrixX3> loadConstraintsFromObj(const std::string& fileName);

std::pair<MatrixX3, VectorX> computeAllKinematicsConstraints(const RbPrmFullBodyPtr_t& fullBody,const model::ConfigurationPtr_t& configuration);

std::pair<MatrixX3, VectorX> computeKinematicsConstraintsForState(const RbPrmFullBodyPtr_t& fullBody, const State& state);

std::pair<MatrixX3, VectorX> computeKinematicsConstraintsForLimb(const RbPrmFullBodyPtr_t& fullBody, const State& state,const std::string& limbName);

std::pair<MatrixX3, VectorX> getInequalitiesAtTransform(const std::pair<MatrixX3, MatrixX3>& NV, const fcl::Transform3f& transform);

bool verifyKinematicConstraints(const std::pair<MatrixX3, VectorX>& Ab, const fcl::Vec3f& point);

bool verifyKinematicConstraints(const std::pair<MatrixX3, MatrixX3>& NV, const fcl::Transform3f& transform, const fcl::Vec3f& point);


bool verifyKinematicConstraints(const RbPrmFullBodyPtr_t& fullbody,const State& state, fcl::Vec3f point);

}
}
}

#endif // KINEMATICS_CONSTRAINTS_HH
