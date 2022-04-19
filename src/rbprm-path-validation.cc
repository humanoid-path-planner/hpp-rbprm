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

#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/config-validation.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/path.hh>
#include <hpp/core/validation-report.hh>
#include <hpp/rbprm/rbprm-path-validation.hh>
#include <hpp/rbprm/rbprm-validation.hh>

namespace hpp {
namespace rbprm {

using core::Configuration_t;
using core::value_type;

RbPrmPathValidationPtr_t RbPrmPathValidation::create(
    const core::DevicePtr_t& robot, const core::value_type& stepSize) {
  RbPrmPathValidation* ptr(new RbPrmPathValidation(robot, stepSize));
  RbPrmPathValidationPtr_t shPtr(ptr);
  return shPtr;
}

RbPrmPathValidation::RbPrmPathValidation(const core::DevicePtr_t& /*robot*/,
                                         const core::value_type& stepSize)
    : core::pathValidation::Discretized(stepSize) {}

void RbPrmPathValidation::add(
    const core::ConfigValidationPtr_t& configValidation) {
  core::pathValidation::Discretized::add(configValidation);
  rbprmValidation_ =
      std::dynamic_pointer_cast<RbPrmValidation>(configValidation);
}

bool RbPrmPathValidation::validate(
    const core::PathPtr_t& path, bool reverse, core::PathPtr_t& validPart,
    core::PathValidationReportPtr_t& validationReport,
    const std::vector<std::string>& filter) {
  core::ValidationReportPtr_t configReport;
  assert(path);
  bool valid = true;
  if (reverse) {
    value_type tmin = path->timeRange().first;
    value_type tmax = path->timeRange().second;
    value_type lastValidTime = tmax;
    value_type t = tmax;
    unsigned finished = 0;
    Configuration_t q(path->outputSize());
    while (finished < 2 && valid) {
      bool success = (*path)(q, t);
      if (!success || !rbprmValidation_->validate(q, configReport, filter)) {
        validationReport = core::CollisionPathValidationReportPtr_t(
            new core::CollisionPathValidationReport(t, configReport));
        valid = false;
      } else {
        lastValidTime = t;
        t -= stepSize_;
      }
      if (t < tmin) {
        t = tmin;
        finished++;
      }
    }
    if (valid) {
      validPart = path;
      return true;
    } else {
      validPart = path->extract(std::make_pair(lastValidTime, tmax));
      return false;
    }
  } else {
    value_type tmin = path->timeRange().first;
    value_type tmax = path->timeRange().second;
    value_type lastValidTime = tmin;
    value_type t = tmin;
    unsigned finished = 0;
    Configuration_t q(path->outputSize());
    while (finished < 2 && valid) {
      bool success = (*path)(q, t);
      if (!success || !rbprmValidation_->validate(q, configReport, filter)) {
        validationReport = core::CollisionPathValidationReportPtr_t(
            new core::CollisionPathValidationReport(t, configReport));
        valid = false;
      } else {
        lastValidTime = t;
        t += stepSize_;
      }
      if (t > tmax) {
        t = tmax;
        finished++;
      }
    }
    if (valid) {
      validPart = path;
      return true;
    } else {
      validPart = path->extract(std::make_pair(tmin, lastValidTime));
      return false;
    }
  }
}

}  // namespace rbprm
}  // namespace hpp
