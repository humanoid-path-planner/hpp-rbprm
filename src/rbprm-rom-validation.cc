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

#include "hpp/rbprm/rbprm-rom-validation.hh"
#include <hpp/fcl/collision.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/rbprm/rbprm-validation-report.hh>
#include "hpp/rbprm/utils/algorithms.h"

namespace hpp {
using namespace core;
namespace rbprm {

RbPrmRomValidationPtr_t RbPrmRomValidation::create(const pinocchio::DevicePtr_t& robot,
                                                   const std::vector<std::string>& affFilters) {
  RbPrmRomValidation* ptr = new RbPrmRomValidation(robot, affFilters);
  return RbPrmRomValidationPtr_t(ptr);
}

RbPrmRomValidation::RbPrmRomValidation(const pinocchio::DevicePtr_t& robot, const std::vector<std::string>& affFilters)
    : hpp::core::CollisionValidation(robot),
      filter_(affFilters),
      unusedReport_(new CollisionValidationReport),
      optional_(false) {}

bool RbPrmRomValidation::validate(const Configuration_t& config) { return validate(config, unusedReport_); }

bool RbPrmRomValidation::validate(const Configuration_t& config, ValidationReportPtr_t& validationReport) {
  ValidationReportPtr_t romReport;

  bool collision = !hpp::core::CollisionValidation::validate(config, romReport);
  // CollisionValidationReportPtr_t reportCast = boost::dynamic_pointer_cast<CollisionValidationReport>(romReport);
  // hppDout(notice,"number of contacts  : "<<reportCast->result.numContacts());
  // hppDout(notice,"contact 1 "<<reportCast->result.getContact(0).pos);
  RbprmValidationReportPtr_t rbprmReport = std::dynamic_pointer_cast<RbprmValidationReport>(validationReport);
  if (rbprmReport) {
    // hppDout(notice,"rbprm-validation-report correctly cast");
    rbprmReport->ROMFilters.insert(std::make_pair(robot_->name(), collision));
  } else {
    hppDout(notice, "Validation report is not a valid rbprm-validation-report instance");
  }
  if (collision) {
    if (rbprmReport) {  // if the report is a correct rbprm report, we add the rom information
      core::CollisionValidationReportPtr_t romCollisionReport =
          HPP_DYNAMIC_PTR_CAST(CollisionValidationReport, romReport);
      rbprmReport->ROMReports.insert(std::make_pair(robot_->name(), romCollisionReport));

      // re arrange the collision pair such that the first one is the pair in collision
      // (allow us to maintain the contact with the same obstacle as long as possible)
      CollisionObjectConstPtr_t obj2 = romCollisionReport->object2;
      bool first(true);
      std::size_t i = 0;
      for (; i < pairs().size(); ++i)
        if (pairs()[i].second == obj2) break;

      if (i != 0 && !pairs().empty()) {
        std::swap(pairs()[0], pairs()[i]);
        std::swap(requests()[0], requests()[i]);
      }
    } else {
      validationReport = romReport;
    }
  }

  if (optional_)
    return true;
  else
    return collision;
}

void RbPrmRomValidation::randomnizeCollisionPairs() {
  for (std::size_t i = 0; i < pairs().size(); ++i) {
    // XXX rand() % N is not uniformly distributed
    std::size_t j = std::rand() % (i + 1);
    if (i != j) {
      std::swap(pairs()[i], pairs()[j]);
      std::swap(requests()[i], requests()[j]);
    }
  }
}

}  // namespace rbprm
}  // namespace hpp
