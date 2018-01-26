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



std::pair<MatrixXX, MatrixXX> loadConstraintsFromObj(const std::string& fileName);

std::pair<MatrixXX, VectorX> computeAllKinematicsConstraints(const RbPrmFullBodyPtr_t& fullBody,const model::ConfigurationPtr_t& configuration);

std::pair<MatrixXX, VectorX> computeKinematicsConstraints(const RbPrmFullBodyPtr_t& fullBody, const State& state);

std::pair<MatrixXX, VectorX> getInequalitiesAtTransform(const std::pair<MatrixXX, MatrixXX>& NV, const fcl::Transform3f& transform);

bool verifyKinematicConstraints(const std::pair<MatrixXX, VectorX>& Ab, const fcl::Vec3f& point);

bool verifyKinematicConstraints(const std::pair<MatrixXX, MatrixXX>& NV, const fcl::Transform3f& transform, const fcl::Vec3f& point);


bool verifyKinematicConstraints(const State& state,const fcl::Vec3f& point);

}
}
}

#endif // KINEMATICS_CONSTRAINTS_HH
