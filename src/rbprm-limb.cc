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

namespace hpp {
  namespace rbprm {

    RbPrmLimbPtr_t RbPrmLimb::create (const model::JointPtr_t& limb,
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

    RbPrmLimb::RbPrmLimb (const model::JointPtr_t& limb,
                          const std::size_t nbSamples, const double resolution)
        : limb_(limb)
        , sampleContainer_(limb, nbSamples, resolution)
    {
        // TODO
    }
  } // model
} //hpp
