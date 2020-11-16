//
// Copyright (c) 2017 CNRS
// Authors: Fernbach Pierre
//
// This file is part of hpp-rbprm
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

#ifndef HPP_RBPRM_DYNAMIC_PATH_VALIDATION_HH
#define HPP_RBPRM_DYNAMIC_PATH_VALIDATION_HH
#include <hpp/rbprm/rbprm-path-validation.hh>
#include <hpp/rbprm/dynamic/dynamic-validation.hh>
namespace hpp {
namespace rbprm {

// forward declaration
HPP_PREDEF_CLASS(DynamicPathValidation);
// Planner objects are manipulated only via shared pointers
typedef boost::shared_ptr<DynamicPathValidation> DynamicPathValidationPtr_t;

class HPP_RBPRM_DLLAPI DynamicPathValidation : public RbPrmPathValidation {
 public:
  /// Create an instance and return a shared pointer to the instance
  static DynamicPathValidationPtr_t create(const core::DevicePtr_t& robot, const core::value_type& stepSize);

  /// validate with custom filter for the rom validation
  virtual bool validate(const core::PathPtr_t& path, bool reverse, core::PathPtr_t& validPart,
                        core::PathValidationReportPtr_t& report, const std::vector<std::string>& filter);

  virtual bool validate(const core::PathPtr_t& path, bool reverse, core::PathPtr_t& validPart,
                        core::PathValidationReportPtr_t& report);

  void addDynamicValidator(const DynamicValidationPtr_t& dynamicValidation) {
    core::pathValidation::Discretized::add(dynamicValidation);
    dynamicValidation_ = dynamicValidation;
  }

 protected:
  /// Protected constructor
  /// Users need to call RbPrmPlanner::create in order to create instances.
  DynamicPathValidation(const core::DevicePtr_t& robot, const core::value_type& stepSize);

 private:
  DynamicValidationPtr_t dynamicValidation_;
};  // class RbPrmPlanner
}  // namespace rbprm
}  // namespace hpp

#endif  // HPP_RBPRM_DYNAMIC_PATH_VALIDATION_HH
