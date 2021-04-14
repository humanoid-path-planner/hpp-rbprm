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

#ifndef HPP_RBPRM_TIME_CONSTRAINT_PATH_VALIDATION_HH
#define HPP_RBPRM_TIME_CONSTRAINT_PATH_VALIDATION_HH

#include <hpp/core/path-validation/discretized.hh>

namespace hpp {
namespace rbprm {
namespace interpolation {
HPP_PREDEF_CLASS(TimeConstraintPathValidation);

class TimeConstraintPathValidation;
typedef std::shared_ptr<TimeConstraintPathValidation> TimeConstraintPathValidationPtr_t;
/// \addtogroup validation
/// \{

/// Discretized validation of a path for the LimbRRT algorithm
///
/// Apply some configuration validation algorithms at discretized values
/// of the path parameter.
class HPP_CORE_DLLAPI TimeConstraintPathValidation : public core::pathValidation::Discretized {
 public:
  static TimeConstraintPathValidationPtr_t create(const pinocchio::DevicePtr_t& robot,
                                                  const pinocchio::value_type& stepSize,
                                                  const std::size_t pathDofRank);

  /// Compute the largest valid interval starting from the path beginning
  /// In the context of the LimbRRT algoritm, a path is only valid if the extra DOF
  /// value of the first configuration of the path is lower than
  /// the one of the last configuration. See the documentation
  /// of interpolateStates for details
  ///
  /// \param path the path to check for validity,
  /// \param reverse if true check from the end,
  /// \retval the extracted valid part of the path, pointer to path if
  ///         path is valid.
  /// \retval report information about the validation process. A report
  ///         is allocated if the path is not valid.
  /// \return whether the whole path is valid.
  virtual bool validate(const core::PathPtr_t& path, bool reverse, core::PathPtr_t& validPart,
                        core::PathValidationReportPtr_t& report);

 public:
  const std::size_t pathDofRank_;

 protected:
  TimeConstraintPathValidation(const pinocchio::DevicePtr_t& robot, const pinocchio::value_type& stepSize,
                               const std::size_t pathDofRank);
};  // class DiscretizedPathValidation
/// \}
}  // namespace interpolation
}  // namespace rbprm
}  // namespace hpp

#endif  // HPP_RBPRM_TIME_CONSTRAINT_PATH_VALIDATION_HH
