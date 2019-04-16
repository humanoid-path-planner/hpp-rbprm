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

#include <hpp/rbprm/interpolation/interpolation-constraints.hh>

using namespace hpp::core;

namespace hpp {
  namespace rbprm {
  namespace interpolation{

  void addContactConstraints(rbprm::RbPrmFullBodyPtr_t fullBody, pinocchio::DevicePtr_t device, core::ConfigProjectorPtr_t projector, const State& state, const std::vector<std::string> active)
  {
      std::vector<bool> cosntraintsR = setMaintainRotationConstraints();
      for(std::vector<std::string>::const_iterator cit = active.begin();
          cit != active.end(); ++cit)
      {
          RbPrmLimbPtr_t limb = fullBody->GetLimbs().at(*cit);
          //const fcl::Vec3f& ppos  = state.contactPositions_.at(*cit);
          pinocchio::Transform3f localFrame(1), globalFrame(1);
          globalFrame.translation(state.contactPositions_.at(*cit));
          //const fcl::Matrix3f& rotation = state.contactRotation_.at(*cit);
          const pinocchio::Frame effectorFrame = device->getFrameByName(limb->effector_.name());
          pinocchio::JointPtr_t effectorJoint = effectorFrame.joint();
          projector->add(constraints::Implicit::create (
                                  constraints::Position::create("",device,
                                                                effectorJoint, effectorFrame.pinocchio().placement * localFrame, globalFrame)));
          if(limb->contactType_ == hpp::rbprm::_6_DOF)
          {
              pinocchio::Transform3f rotation(1);
              rotation.rotation(state.contactRotation_.at(*cit) * effectorFrame.pinocchio().placement.rotation().transpose());
              projector->add(constraints::Implicit::create (constraints::Orientation::create("", device,
                                                                                effectorJoint,
                                                                                rotation,
                                                                                cosntraintsR)));
          }
      }
  }

      std::string getEffectorLimb(const  State &startState, const State &nextState)
    {
        return nextState.contactCreations(startState).front();
    }

    pinocchio::Frame getEffector(RbPrmFullBodyPtr_t fullbody,
                           const  State &startState, const State &nextState)
    {
        std::string effectorVar = getEffectorLimb(startState, nextState);
        return fullbody->device_->getFrameByName(fullbody->GetLimbs().at(effectorVar)->effector_.name());
    }




  } //   namespace interpolation
  } //   namespace rbprm
} // namespace hpp

