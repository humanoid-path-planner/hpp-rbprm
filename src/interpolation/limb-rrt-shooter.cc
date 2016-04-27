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

#include <hpp/rbprm/interpolation/limb-rrt-shooter.hh>
#include <hpp/rbprm/rbprm-limb.hh>
#include <hpp/rbprm/sampling/sample.hh>

namespace hpp {
using namespace core;
  namespace rbprm {
  namespace interpolation {

    LimbRRTShooterPtr_t LimbRRTShooter::create (hpp::rbprm::RbPrmLimbPtr_t limb,
                                                const hpp::core::PathPtr_t path,
                                                const std::size_t pathDofRank)
    {
        LimbRRTShooter* ptr = new LimbRRTShooter (limb, path, pathDofRank);
        LimbRRTShooterPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
    }

    void LimbRRTShooter::init (const LimbRRTShooterPtr_t& self)
    {
        ConfigurationShooter::init (self);
        weak_ = self;
    }

    LimbRRTShooter::LimbRRTShooter (const hpp::rbprm::RbPrmLimbPtr_t limb,
                                    const hpp::core::PathPtr_t path,
                                    const std::size_t pathDofRank)
    : core::ConfigurationShooter()
    , limb_(limb)
    , path_(path)
    , pathDofRank_(pathDofRank)
    {
        // NOTHING
    }

    hpp::core::ConfigurationPtr_t LimbRRTShooter::shoot () const
    {
        // edit path sampling dof
        value_type a = path_->timeRange().first; value_type b = path_->timeRange().second;
        value_type pathDofVal = (b-a)* rand() / value_type(RAND_MAX) + a;
        ConfigurationPtr_t config (new Configuration_t((*path_)(pathDofVal)));
        (*config) [pathDofRank_] = pathDofVal;
        // choose random limb configuration
        RbPrmLimbPtr_t limb_;
        const sampling::Sample& sample = *(limb_->sampleContainer_.samples_.begin() + (rand() % (int) (limb_->sampleContainer_.samples_.size() -1)));
        sampling::Load(sample,*config);
        //config->segment(0,pathRootLength_) = ((*path_)(pathDofVal)).segment(pathRootRank_, pathRootLength_);
        return config;
    }
  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp
