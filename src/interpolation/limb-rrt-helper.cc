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

#include <hpp/rbprm/interpolation/limb-rrt-helper.hh>

#include <hpp/model/joint.hh>

namespace hpp {
using namespace core;
using namespace model;
  namespace rbprm {
  namespace interpolation {

  namespace{
    core::DevicePtr_t DeviceFromLimb(const std::string& name, RbPrmLimbPtr_t limb)
    {
        DevicePtr_t limbDevice = Device::create(name);
        limbDevice->rootJoint(limb->limb_->clone());
        JointPtr_t current = limb->limb_, clone = limbDevice->rootJoint();
        while(current->name() != limb->effector_->name())
        {
            current = current->childJoint(0);
            clone->addChildJoint(current->clone());
            clone = clone->childJoint(0);
        }
        limbDevice->setDimensionExtraConfigSpace(1);
        return limbDevice;
    }
    T_LimbDevice DevicesFromLimbs(RbPrmFullBodyPtr_t fullbody)
    {
        T_LimbDevice devices;
        for(T_Limb::const_iterator cit = fullbody->GetLimbs().begin();
            cit != fullbody->GetLimbs().end(); ++cit)
        {
            devices.insert(std::make_pair(cit->first,
                                          DeviceFromLimb(cit->first, cit->second)));
        }
        return devices;
    }
  }

  LimbRRTHelper::LimbRRTHelper(hpp::model::RbPrmDevicePtr_t rootDevice, RbPrmFullBodyPtr_t fullbody)
      : rootDevice_(rootDevice)
      , prototypes_(DevicesFromLimbs(fullbody))
  {

  }
  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp
