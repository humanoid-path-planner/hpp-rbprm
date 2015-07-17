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

#include <hpp/rbprm/rbprm-limb.hh>
#include <hpp/model/joint.hh>

namespace hpp {
  namespace rbprm {

    RbPrmLimbPtr_t RbPrmLimb::create (const model::JointPtr_t limb,
                                      const std::size_t nbSamples, const double resolution)
    {
        RbPrmLimb* rbprmDevice = new RbPrmLimb(limb, nbSamples, resolution);
        RbPrmLimbPtr_t res (rbprmDevice);
        res->init (res);
        return res;
    }

    RbPrmLimb::~RbPrmLimb()
    {
        // NOTHING
    }

    // ========================================================================

    void RbPrmLimb::init(const RbPrmLimbWkPtr_t& weakPtr)
    {
        weakPtr_ = weakPtr;
    }

    model::JointPtr_t GetEffector(const model::JointPtr_t limb)
    {
        model::JointPtr_t current = limb;
        while(current->numberChildJoints() !=0)
        {
            //assert(current->numberChildJoints() ==1);
            current = current->childJoint(0);
        }
        return current;
    }

    RbPrmLimb::RbPrmLimb (const model::JointPtr_t& limb,
                          const std::size_t nbSamples, const double resolution)
        : limb_(limb)
        , effector_(GetEffector(limb))
        , sampleContainer_(limb, nbSamples, resolution)
    {
        // TODO
    }
  } // model
} //hpp
