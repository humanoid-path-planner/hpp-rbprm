// Copyright (c) 2016, LAAS-CNRS
// Authors: Pierre Fernbach (pierre.fernbach@laas.fr)
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_RBPRM_STEERING_METHOD_KINODYNAMIC_HH
#define HPP_RBPRM_STEERING_METHOD_KINODYNAMIC_HH

#include <hpp/rbprm/config.hh>
#include <hpp/core/steering-method/steering-kinodynamic.hh>
#include <hpp/rbprm/planner/rbprm-node.hh>

namespace hpp {
namespace rbprm {

using core::ConfigurationIn_t;
using core::Path;
using core::Problem;

HPP_PREDEF_CLASS(SteeringMethodKinodynamic);
typedef std::shared_ptr<SteeringMethodKinodynamic> SteeringMethodKinodynamicPtr_t;

class HPP_RBPRM_DLLAPI SteeringMethodKinodynamic : public core::steeringMethod::Kinodynamic {
 public:
  core::PathPtr_t operator()(core::ConfigurationIn_t q1, const core::NodePtr_t x) {
    try {
      return impl_compute(q1, x);
    } catch (const core::projection_error& e) {
      hppDout(info, "Could not build path: " << e.what());
    }
    return core::PathPtr_t();
  }

  core::PathPtr_t operator()(const core::NodePtr_t x, core::ConfigurationIn_t q2) {
    try {
      return impl_compute(x, q2);
    } catch (const core::projection_error& e) {
      hppDout(info, "Could not build path: " << e.what());
    }
    return core::PathPtr_t();
  }
  /// Create an instance
  static SteeringMethodKinodynamicPtr_t create(core::ProblemConstPtr_t problem) {
    SteeringMethodKinodynamic* ptr = new SteeringMethodKinodynamic(problem);
    SteeringMethodKinodynamicPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  /// Copy instance and return shared pointer
  static SteeringMethodKinodynamicPtr_t createCopy(const SteeringMethodKinodynamicPtr_t& other) {
    SteeringMethodKinodynamic* ptr = new SteeringMethodKinodynamic(*other);
    SteeringMethodKinodynamicPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }

  /// Copy instance and return shared pointer
  virtual core::SteeringMethodPtr_t copy() const { return createCopy(weak_.lock()); }

  /// create a path between two configurations
  virtual core::PathPtr_t impl_compute(core::ConfigurationIn_t q1, core::ConfigurationIn_t q2) const;

  core::PathPtr_t impl_compute(core::NodePtr_t x, core::ConfigurationIn_t q2);

  core::PathPtr_t impl_compute(core::ConfigurationIn_t q1, core::NodePtr_t x);

  double totalTimeComputed_;
  double totalTimeValidated_;
  int dirValid_;
  int dirTotal_;
  int rejectedPath_;
  const double maxLength_;

 protected:
  /// Constructor
  SteeringMethodKinodynamic(core::ProblemConstPtr_t problem);

  /// Copy constructor
  SteeringMethodKinodynamic(const SteeringMethodKinodynamic& other);

  /// Store weak pointer to itself
  void init(SteeringMethodKinodynamicWkPtr_t weak) {
    core::SteeringMethod::init(weak);
    weak_ = weak;
  }

  /**
   * @brief computeDirection compute the direction that the steering method will choose in order to connect from to to
   * @param from
   * @param to
   * @return
   */
  core::PathPtr_t computeDirection(const core::ConfigurationIn_t from, const core::ConfigurationIn_t to, bool reverse);

  /**
   * @brief setSteeringMethodBounds Compute the maximal acceleration on a direction from near to target,
   *                                and send it to the steering method
   * @param near the node from where we take the the information about contact and position
   * @param target the target configuration
   * @param reverse if true, we compute the acceleration from target to near, with the information from near
   * @return the node casted from near
   */
  core::PathPtr_t setSteeringMethodBounds(const core::RbprmNodePtr_t& near, const core::ConfigurationIn_t target,
                                          bool reverse);

 private:
  core::DeviceWkPtr_t device_;
  centroidal_dynamics::Vector3 lastDirection_;
  centroidal_dynamics::Equilibrium* sEq_;
  bool boundsUpToDate_;
  SteeringMethodKinodynamicWkPtr_t weak_;

};  // class rbprm-kinodynamic
}  // namespace rbprm
}  // namespace hpp

#endif  // HPP_RBPRM_STEERING_METHOD_KINODYNAMIC_HH
