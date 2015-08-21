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


namespace hpp {
  using namespace core;
  namespace rbprm {

  RbPrmRomValidationPtr_t RbPrmRomValidation::create
  (const model::DevicePtr_t& robot)
  {
    NormalFilter filter; filter.unConstrained = true;
    RbPrmRomValidation* ptr = new RbPrmRomValidation (robot, filter);
    return RbPrmRomValidationPtr_t (ptr);
  }

    RbPrmRomValidationPtr_t RbPrmRomValidation::create
    (const model::DevicePtr_t& robot, const NormalFilter& normalFilter)
    {
      RbPrmRomValidation* ptr = new RbPrmRomValidation (robot, normalFilter);
      return RbPrmRomValidationPtr_t (ptr);
    }

    RbPrmRomValidation::RbPrmRomValidation (const model::DevicePtr_t& robot
                                      , const NormalFilter& normalFilter)
        : hpp::core::CollisionValidation(robot)
        , filter_(normalFilter)
    {
        if(!normalFilter.unConstrained)
            collisionRequest_.enable_contact = true;
    }


    bool RbPrmRomValidation::validate (const Configuration_t& config,
                    bool throwIfInValid)
    {
        return hpp::core::CollisionValidation::validate(config, throwIfInValid);
    }

    bool RbPrmRomValidation::validate (const Configuration_t& config,
                    ValidationReport& validationReport,
                    bool throwIfInValid)
    {        
        return hpp::core::CollisionValidation::validate(config, validationReport, throwIfInValid);
    }
  }// namespace rbprm
}// namespace hpp
