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

#include "hpp/rbprm/rbprm-validation.hh"
#include "hpp/core/collision-validation.hh"


namespace hpp {
  using namespace core;
  namespace rbprm {

    RbPrmValidationPtr_t RbPrmValidation::create
    (const model::RbPrmDevicePtr_t& robot)
    {
      RbPrmValidation* ptr = new RbPrmValidation (robot);
      return RbPrmValidationPtr_t (ptr);
    }

    RbPrmValidation::RbPrmValidation (const model::RbPrmDevicePtr_t& robot)
        : trunkValidation_(CollisionValidation::create(robot))
        , romValidation_(CollisionValidation::create(robot->robotRom_))
    {
        // NOTHING
    }


    bool RbPrmValidation::validate (const Configuration_t& config,
                    bool throwIfInValid)
    {
        return trunkValidation_->validate(config, throwIfInValid)
             && !romValidation_->validate(config, throwIfInValid);
    }

    bool RbPrmValidation::validate (const Configuration_t& config,
                    ValidationReport& validationReport,
                    bool throwIfInValid)
    {
        return trunkValidation_->validate(config, throwIfInValid)
             && !romValidation_->validate(config, validationReport, throwIfInValid);
    }

    void RbPrmValidation::addObstacle (const CollisionObjectPtr_t& object)
    {
        trunkValidation_->addObstacle(object);
        romValidation_->addObstacle(object);
    }

    void RbPrmValidation::removeObstacleFromJoint
    (const JointPtr_t& joint, const CollisionObjectPtr_t& obstacle)
    {
        trunkValidation_->removeObstacleFromJoint(joint, obstacle);
        romValidation_->removeObstacleFromJoint(joint, obstacle);
    }

  }// namespace rbprm
}// namespace hpp
