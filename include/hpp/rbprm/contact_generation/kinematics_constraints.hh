#ifndef HPP_RBPRM_KINEMATICS_CONSTRAINTS_HH
#define HPP_RBPRM_KINEMATICS_CONSTRAINTS_HH

#include <hpp/rbprm/rbprm-limb.hh>
#include <hpp/rbprm/rbprm-state.hh>
namespace hpp {
  namespace rbprm {



std::pair<MatrixXX, VectorX> loadConstraintsFromObj(const std::string& fileName);

std::pair<MatrixXX, VectorX> applyTransformToConstraints(const std::pair<MatrixXX, VectorX>& Ab, fcl::Transform3f transform);

bool verifyKinematicConstraints(const std::pair<MatrixXX, VectorX>& Ab, fcl::Vec3f point);

bool verifyKinematicConstraints(const State& state, fcl::Vec3f point);

}
}


#endif // KINEMATICS_CONSTRAINTS_HH
