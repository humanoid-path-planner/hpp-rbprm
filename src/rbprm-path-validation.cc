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

#include <hpp/rbprm/rbprm-path-validation.hh>

namespace hpp{
  namespace rbprm {

    RbPrmPathValidationPtr_t RbPrmPathValidation::create (const core::DevicePtr_t& robot, const core::value_type& stepSize)
    {
      RbPrmPathValidation* ptr (new RbPrmPathValidation(robot, stepSize));
      RbPrmPathValidationPtr_t shPtr (ptr);
      return shPtr;
    }


    RbPrmPathValidation::RbPrmPathValidation(const core::DevicePtr_t &robot, const core::value_type &stepSize) :
      core::DiscretizedCollisionChecking(robot,stepSize){}


  }//namespace rbprm
} //namespace hpp
