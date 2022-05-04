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

#ifndef HPP_RBPRM_TIMECONSTRAINT_PATH_HH
#define HPP_RBPRM_TIMECONSTRAINT_PATH_HH

#include <hpp/core/config.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/path.hh>
#include <hpp/rbprm/interpolation/time-dependant.hh>

namespace hpp {
namespace rbprm {
namespace interpolation {
HPP_PREDEF_CLASS(TimeConstraintPath);
typedef shared_ptr<TimeConstraintPath> TimeConstraintPathPtr_t;
/// Linear interpolation between two configurations
///
/// Degrees of freedom are interpolated depending on the type of
/// \link hpp::pinocchio::Joint joint \endlink
/// they parameterize:
///   \li linear interpolation for translation joints, bounded rotation
///       joints, and translation part of freeflyer joints,
///   \li angular interpolation for unbounded rotation joints,
///   \li constant angular velocity for SO(3) part of freeflyer joints.
class HPP_CORE_DLLAPI TimeConstraintPath : public core::Path {
 public:
  typedef Path parent_t;
  /// Destructor
  virtual ~TimeConstraintPath() {}

  /// Create instance and return shared pointer
  /// \param device Robot corresponding to configurations
  /// \param init, end Start and end configurations of the path
  /// \param length Distance between the configurations.
  static TimeConstraintPathPtr_t create(const core::DevicePtr_t& device,
                                        core::ConfigurationIn_t init,
                                        core::ConfigurationIn_t end,
                                        core::value_type length,
                                        const std::size_t pathDofRank,
                                        const T_TimeDependant& tds) {
    TimeConstraintPath* ptr =
        new TimeConstraintPath(device, init, end, length, pathDofRank, tds);
    TimeConstraintPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    ptr->checkPath();
    return shPtr;
  }

  /// Create instance and return shared pointer
  /// \param device Robot corresponding to configurations
  /// \param init, end Start and end configurations of the path
  /// \param length Distance between the configurations.
  /// \param constraints the path is subject to
  static TimeConstraintPathPtr_t create(const core::DevicePtr_t& device,
                                        core::ConfigurationIn_t init,
                                        core::ConfigurationIn_t end,
                                        core::value_type length,
                                        core::ConstraintSetPtr_t constraints,
                                        const std::size_t pathDofRank,
                                        const T_TimeDependant& tds) {
    TimeConstraintPath* ptr = new TimeConstraintPath(
        device, init, end, length, constraints, pathDofRank, tds);
    TimeConstraintPathPtr_t shPtr(ptr);
    ptr->init(shPtr);
    ptr->checkPath();
    return shPtr;
  }

  /// Create copy and return shared pointer
  /// \param path path to copy
  static TimeConstraintPathPtr_t createCopy(
      const TimeConstraintPathPtr_t& path) {
    TimeConstraintPath* ptr = new TimeConstraintPath(*path);
    TimeConstraintPathPtr_t shPtr(ptr);
    ptr->initCopy(shPtr);
    ptr->checkPath();
    return shPtr;
  }

  /// Create copy and return shared pointer
  /// \param path path to copy
  /// \param constraints the path is subject to
  static TimeConstraintPathPtr_t createCopy(
      const TimeConstraintPathPtr_t& path,
      const core::ConstraintSetPtr_t& constraints) {
    TimeConstraintPath* ptr = new TimeConstraintPath(*path, constraints);
    TimeConstraintPathPtr_t shPtr(ptr);
    ptr->initCopy(shPtr);
    ptr->checkPath();
    return shPtr;
  }

  /// Return a shared pointer to this
  ///
  /// As TimeConstraintPathP are immutable, and refered to by shared pointers,
  /// they do not need to be copied.
  virtual core::PathPtr_t copy() const { return createCopy(weak_.lock()); }

  /// Return a shared pointer to a copy of this and set constraints
  ///
  /// \param constraints constraints to apply to the copy
  /// \precond *this should not have constraints.
  virtual core::PathPtr_t copy(
      const core::ConstraintSetPtr_t& constraints) const {
    return createCopy(weak_.lock(), constraints);
  }

  /// Extraction/Reversion of a sub-path
  /// \param subInterval interval of definition of the extract path
  /// If upper bound of subInterval is smaller than lower bound,
  /// result is reversed.
  virtual core::PathPtr_t extract(const core::interval_t& subInterval) const;

  /// Modify initial configuration
  /// \param initial new initial configuration
  /// \pre input configuration should be of the same size as current initial
  /// configuration
  void initialConfig(core::ConfigurationIn_t initial) {
    assert(initial.size() == initial_.size());
    pinocchio::value_type dof = initial_[pathDofRank_];
    initial_ = initial;
    initial_[pathDofRank_] = dof;
  }

  /// Modify end configuration
  /// \param end new end configuration
  /// \pre input configuration should be of the same size as current end
  /// configuration
  void endConfig(core::ConfigurationIn_t end) {
    assert(end.size() == end_.size());
    pinocchio::value_type dof = end_[pathDofRank_];
    end_ = end;
    end_[pathDofRank_] = dof;
  }

  /// Return the internal robot.
  core::DevicePtr_t device() const;

  /// Get the initial configuration
  core::Configuration_t initial() const { return initial_; }

  /// Get the final configuration
  core::Configuration_t end() const { return end_; }

  virtual void checkPath() const;

 protected:
  /// Print path in a stream
  virtual std::ostream& print(std::ostream& os) const {
    os << "TimeConstraintPath:" << std::endl;
    os << "interval: [ " << timeRange().first << ", " << timeRange().second
       << " ]" << std::endl;
    os << "initial configuration: " << initial_.transpose() << std::endl;
    os << "final configuration:   " << end_.transpose() << std::endl;
    return os;
  }
  /// Constructor
  TimeConstraintPath(const core::DevicePtr_t& robot,
                     core::ConfigurationIn_t init, core::ConfigurationIn_t end,
                     core::value_type length, const std::size_t pathDofRank,
                     const T_TimeDependant& tds);

  /// Constructor with constraints
  TimeConstraintPath(const core::DevicePtr_t& robot,
                     core::ConfigurationIn_t init, core::ConfigurationIn_t end,
                     core::value_type length,
                     core::ConstraintSetPtr_t constraints,
                     const std::size_t pathDofRank, const T_TimeDependant& tds);

  /// Copy constructor
  TimeConstraintPath(const TimeConstraintPath& path);

  /// Copy constructor with constraints
  TimeConstraintPath(const TimeConstraintPath& path,
                     const core::ConstraintSetPtr_t& constraints);

  void init(TimeConstraintPathPtr_t self) {
    parent_t::init(self);
    weak_ = self;
  }

  void initCopy(TimeConstraintPathPtr_t self) {
    parent_t::init(self);
    weak_ = self;
  }

  virtual bool impl_compute(core::ConfigurationOut_t result,
                            core::value_type param) const;

 private:
  void updateConstraints(core::ConfigurationOut_t configuration) const;

 private:
  core::DevicePtr_t device_;
  core::Configuration_t initial_;
  core::Configuration_t end_;

 public:
  const std::size_t pathDofRank_;
  const T_TimeDependant tds_;

 private:
  TimeConstraintPathWkPtr_t weak_;
};  // class TimeConstraintPath
}  // namespace interpolation
}  // namespace rbprm
}  // namespace hpp
#endif  // HPP_RBPRM_TIMECONSTRAINT_PATH_HH
