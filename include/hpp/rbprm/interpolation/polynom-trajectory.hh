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

#ifndef HPP_RBPRM_POLYNOM_TRAJECTORY_HH
#define HPP_RBPRM_POLYNOM_TRAJECTORY_HH

#include <hpp/core/fwd.hh>
#include <hpp/core/config.hh>
#include <hpp/core/path.hh>
#include <hpp/rbprm/interpolation/time-dependant.hh>
#include <ndcurves/curve_abc.h>

namespace hpp {
namespace rbprm {
namespace interpolation {
HPP_PREDEF_CLASS(PolynomTrajectory);
typedef std::shared_ptr<PolynomTrajectory> PolynomTrajectoryPtr_t;
typedef ndcurves::curve_abc<core::value_type, core::value_type, true, Eigen::Vector3d> Polynom;
typedef std::shared_ptr<Polynom> PolynomPtr_t;
/// Linear interpolation between two configurations
///
/// Degrees of freedom are interpolated depending on the type of
/// \link hpp::pinocchio::Joint joint \endlink
/// they parameterize:
///   \li linear interpolation for translation joints, bounded rotation
///       joints, and translation part of freeflyer joints,
///   \li angular interpolation for unbounded rotation joints,
///   \li constant angular velocity for SO(3) part of freeflyer joints.
class HPP_CORE_DLLAPI PolynomTrajectory : public core::Path {
 public:
  typedef Path parent_t;
  /// Destructor
  virtual ~PolynomTrajectory() {}

  /// Create instance and return shared pointer
  /// \param device Robot corresponding to configurations
  /// \param init, end Start and end configurations of the path
  /// \param length Distance between the configurations.
  static PolynomTrajectoryPtr_t create(PolynomPtr_t polynom, core::value_type subSetStart = 0,
                                       core::value_type subSetEnd = 1) {
    PolynomTrajectory* ptr = new PolynomTrajectory(polynom, subSetStart, subSetEnd);
    PolynomTrajectoryPtr_t shPtr(ptr);
    ptr->init(shPtr);
    ptr->checkPath();
    return shPtr;
  }

  /// Create copy and return shared pointer
  /// \param path path to copy
  static PolynomTrajectoryPtr_t createCopy(const PolynomTrajectoryPtr_t& path) {
    PolynomTrajectory* ptr = new PolynomTrajectory(*path);
    PolynomTrajectoryPtr_t shPtr(ptr);
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
  core::Configuration_t initial() const { return polynom_->operator()(subSetStart_); }

  /// Get the final configuration
  core::Configuration_t end() const { return polynom_->operator()(subSetEnd_); }

  virtual void checkPath() const {}

 protected:
  /// Print path in a stream
  virtual std::ostream& print(std::ostream& os) const {
    os << "PolynomTrajectory:" << std::endl;
    os << "interval: [ " << timeRange().first << ", " << timeRange().second << " ]" << std::endl;
    os << "initial configuration: " << initial() << std::endl;
    os << "final configuration:   " << end() << std::endl;
    return os;
  }
  /// Constructor
  PolynomTrajectory(PolynomPtr_t polynom, core::value_type subSetStart, core::value_type subSetEnd);

  /// Copy constructor
  PolynomTrajectory(const PolynomTrajectory& path);

  void init(PolynomTrajectoryPtr_t self) {
    parent_t::init(self);
    weak_ = self;
  }

  virtual bool impl_compute(core::ConfigurationOut_t result, core::value_type param) const;

  virtual core::PathPtr_t copy(const core::ConstraintSetPtr_t&) const { throw; }

 public:
  const PolynomPtr_t polynom_;
  const core::value_type subSetStart_;
  const core::value_type subSetEnd_;
  const core::value_type length_;

 private:
  PolynomTrajectoryWkPtr_t weak_;
};  // class ComTrajectory
}  // namespace interpolation
}  // namespace rbprm
}  // namespace hpp
#endif  // HPP_RBPRM_POLYNOM_TRAJECTORY_HH
