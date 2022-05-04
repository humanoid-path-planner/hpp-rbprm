//
// Copyright (c) 2014 CNRS
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

#ifndef HPP_RBPRM_COM_TRAJECTORY_HH
#define HPP_RBPRM_COM_TRAJECTORY_HH

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/path.hh>
#include <hpp/rbprm/interpolation/time-dependant.hh>

namespace hpp {
namespace rbprm {
namespace interpolation {
HPP_PREDEF_CLASS(ComTrajectory);
typedef shared_ptr<ComTrajectory> ComTrajectoryPtr_t;
/// Linear interpolation between two configurations
///
/// Degrees of freedom are interpolated depending on the type of
/// \link hpp::pinocchio::Joint joint \endlink
/// they parameterize:
///   \li linear interpolation for translation joints, bounded rotation
///       joints, and translation part of freeflyer joints,
///   \li angular interpolation for unbounded rotation joints,
///   \li constant angular velocity for SO(3) part of freeflyer joints.
class HPP_CORE_DLLAPI ComTrajectory : public core::Path {
 public:
  typedef Path parent_t;
  /// Destructor
  virtual ~ComTrajectory() {}

  /// Create instance and return shared pointer
  /// \param device Robot corresponding to configurations
  /// \param init, end Start and end configurations of the path
  /// \param length Distance between the configurations.
  static ComTrajectoryPtr_t create(pinocchio::vector3_t init,
                                   pinocchio::vector3_t end,
                                   pinocchio::vector3_t initSpeed,
                                   pinocchio::vector3_t acceleration,
                                   core::value_type length) {
    ComTrajectory* ptr =
        new ComTrajectory(init, end, initSpeed, acceleration, length);
    ComTrajectoryPtr_t shPtr(ptr);
    ptr->init(shPtr);
    ptr->checkPath();
    return shPtr;
  }

  /// Create copy and return shared pointer
  /// \param path path to copy
  static ComTrajectoryPtr_t createCopy(const ComTrajectoryPtr_t& path) {
    ComTrajectory* ptr = new ComTrajectory(*path);
    ComTrajectoryPtr_t shPtr(ptr);
    ptr->init(shPtr);
    ptr->checkPath();
    return shPtr;
  }

  /// Return a shared pointer to this
  ///
  /// As ComTrajectoryP are immutable, and refered to by shared pointers,
  /// they do not need to be copied.
  virtual core::PathPtr_t copy() const { return createCopy(weak_.lock()); }

  /// Extraction/Reversion of a sub-path
  /// \param subInterval interval of definition of the extract path
  /// If upper bound of subInterval is smaller than lower bound,
  /// result is reversed.
  virtual core::PathPtr_t extract(const core::interval_t& subInterval) const;

  /// Get the initial configuration
  core::Configuration_t initial() const { return initial_; }

  /// Get the final configuration
  core::Configuration_t end() const { return end_; }

  virtual void checkPath() const {}

 protected:
  /// Print path in a stream
  virtual std::ostream& print(std::ostream& os) const {
    os << "ComTrajectory:" << std::endl;
    os << "interval: [ " << timeRange().first << ", " << timeRange().second
       << " ]" << std::endl;
    os << "initial configuration: " << initial_ << std::endl;
    os << "final configuration:   " << end_ << std::endl;
    os << "init speed:   " << initSpeed_ << std::endl;
    os << "acceleration (constant):   " << (acceleration_) << std::endl;
    return os;
  }
  /// Constructor
  ComTrajectory(pinocchio::vector3_t init, pinocchio::vector3_t end,
                pinocchio::vector3_t initSpeed,
                pinocchio::vector3_t acceleration, core::value_type length);

  /// Copy constructor
  ComTrajectory(const ComTrajectory& path);

  void init(ComTrajectoryPtr_t self) {
    parent_t::init(self);
    weak_ = self;
  }

  virtual bool impl_compute(core::ConfigurationOut_t result,
                            core::value_type param) const;

  virtual core::PathPtr_t copy(const core::ConstraintSetPtr_t&) const { throw; }

 public:
  const pinocchio::vector3_t initial_;
  const pinocchio::vector3_t end_;
  const pinocchio::vector3_t initSpeed_;
  const pinocchio::vector3_t half_acceleration_;
  const pinocchio::vector3_t acceleration_;
  const pinocchio::value_type length_;

 private:
  ComTrajectoryWkPtr_t weak_;
};  // class ComTrajectory
}  // namespace interpolation
}  // namespace rbprm
}  // namespace hpp
#endif  // HPP_RBPRM_COM_TRAJECTORY_HH
