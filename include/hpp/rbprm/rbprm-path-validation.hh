// Copyright (c) 2014, LAAS-CNRS
// Authors: Steve Tonneau (steve.tonneau@laas.fr)
//
// This file is part of hpp-rbprm.
// hpp-rbprm is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-rbprm is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-rbprm. If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_RBPRM_PATH_VALIDATION_HH
#define HPP_RBPRM_PATH_VALIDATION_HH

#include <hpp/core/path-validation/discretized.hh>
#include <hpp/rbprm/config.hh>
#include <hpp/rbprm/rbprm-validation.hh>
#include <hpp/util/pointer.hh>

namespace hpp {
namespace rbprm {

// forward declaration
HPP_PREDEF_CLASS(RbPrmPathValidation);
// Planner objects are manipulated only via shared pointers
typedef shared_ptr<RbPrmPathValidation> RbPrmPathValidationPtr_t;

class HPP_RBPRM_DLLAPI RbPrmPathValidation
    : public core::pathValidation::Discretized {
 public:
  /// Create an instance and return a shared pointer to the instance
  static RbPrmPathValidationPtr_t create(const core::DevicePtr_t& robot,
                                         const core::value_type& stepSize);

  /// validate with custom filter for the rom validation
  virtual bool validate(const core::PathPtr_t& path, bool reverse,
                        core::PathPtr_t& validPart,
                        core::PathValidationReportPtr_t& report,
                        const std::vector<std::string>& filter);

  virtual bool validate(const core::PathPtr_t& path, bool reverse,
                        core::PathPtr_t& validPart,
                        core::PathValidationReportPtr_t& report) {
    return core::pathValidation::Discretized::validate(path, reverse, validPart,
                                                       report);
  }

  /// Add a configuration validation object
  virtual void add(const core::ConfigValidationPtr_t& configValidation);

  RbPrmValidationPtr_t getValidator() { return rbprmValidation_; }

 protected:
  /// Protected constructor
  /// Users need to call RbPrmPlanner::create in order to create instances.
  RbPrmPathValidation(const core::DevicePtr_t& robot,
                      const core::value_type& stepSize);

  RbPrmValidationPtr_t rbprmValidation_;

};  // class RbPrmPlanner
}  // namespace rbprm
}  // namespace hpp
#endif  // HPP_RBPRM_PATH_VALIDATION_HH
