//
// Copyright (c) 2015 CNRS
// Authors: Florent Lamiraux
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

#ifndef HPP_TIME_CONSTRAINT_STEERING_HH
#define HPP_TIME_CONSTRAINT_STEERING_HH

#include <hpp/core/path-validation/discretized.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/rbprm/interpolation/time-constraint-path.hh>
#include <hpp/rbprm/interpolation/time-dependant.hh>

namespace hpp {
namespace rbprm {
namespace interpolation {
/// \addtogroup validation
/// \{
/// Discretized validation of a path for the LimbRRT algorithm
///
/// Apply some configuration validation algorithms at discretized values
/// of the path parameter.
template <class Path_T>
class TimeConstraintSteering : public hpp::core::steeringMethod::Straight {
  typedef Path_T path_t;
  typedef boost::shared_ptr<TimeConstraintSteering> TimeConstraintSteeringPtr_t;
  typedef boost::weak_ptr<TimeConstraintSteering> TimeConstraintSteeringWkPtr_t;

 public:
  /// Create instance and return shared pointer
  static TimeConstraintSteeringPtr_t create(const core::ProblemPtr_t& problem, const std::size_t pathDofRank) {
    TimeConstraintSteering* ptr = new TimeConstraintSteering(problem, pathDofRank);
    TimeConstraintSteeringPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }
  /// Create instance and return shared pointer
  static TimeConstraintSteeringPtr_t create(const core::DevicePtr_t& device,
                                            const core::WeighedDistancePtr_t& distance,
                                            const std::size_t pathDofRank) HPP_CORE_DEPRECATED {
    TimeConstraintSteering* ptr = new TimeConstraintSteering(device, distance, pathDofRank);
    TimeConstraintSteeringPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }
  /// Copy instance and return shared pointer
  static TimeConstraintSteeringPtr_t createCopy(const TimeConstraintSteeringPtr_t& other) {
    TimeConstraintSteering* ptr = new TimeConstraintSteering(*other);
    TimeConstraintSteeringPtr_t shPtr(ptr);
    ptr->init(shPtr);
    return shPtr;
  }
  /// Copy instance and return shared pointer
  virtual core::SteeringMethodPtr_t copy() const { return createCopy(weak_.lock()); }

  /// create a path between two configurations
  virtual core::PathPtr_t impl_compute(core::ConfigurationIn_t q1, core::ConfigurationIn_t q2) const {
    core::value_type length = problem_.distance()->operator()(q1, q2);
    core::ConstraintSetPtr_t c;
    if (constraints() && constraints()->configProjector()) {
      c = HPP_STATIC_PTR_CAST(core::ConstraintSet, constraints()->copy());
      c->configProjector()->rightHandSideFromConfig(q1);
    } else {
      c = constraints();
    }
    core::PathPtr_t path = path_t::create(problem_.robot(), q1, q2, length, c, pathDofRank_, tds_);
    return path;
  }

 protected:
  /// Constructor with robot
  /// Weighed distance is created from robot
  TimeConstraintSteering(const core::ProblemPtr_t& problem, const std::size_t pathDofRank)
      : core::steeringMethod::Straight(*problem), pathDofRank_(pathDofRank), weak_() {}

  /*/// Constructor with weighed distance
  TimeConstraintSteering (const core::DevicePtr_t& device,
              const core::WeighedDistancePtr_t& distance,
                   const std::size_t pathDofRank) :
SteeringMethod (new core::Problem (device)), pathDofRank_(pathDofRank), weak_ ()
  {
    problem_.distance (distance);
  }*/
  /// Copy constructor
  TimeConstraintSteering(const TimeConstraintSteering& other)
      : core::steeringMethod::Straight(other), pathDofRank_(other.pathDofRank_), weak_(), tds_(other.tds_) {}

  /// Store weak pointer to itself
  void init(TimeConstraintSteeringWkPtr_t weak) {
    core::steeringMethod::Straight::init(weak);
    weak_ = weak;
  }

 private:
  const core::PathPtr_t model_;
  const std::size_t pathDofRank_;
  TimeConstraintSteeringWkPtr_t weak_;

 public:
  T_TimeDependant tds_;
};  // SteeringMethodStraight
/// \}
}  // namespace interpolation
}  // namespace rbprm
}  // namespace hpp

#endif  // HPP_TIME_CONSTRAINT_STEERING_HH
