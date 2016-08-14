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
#include <hpp/rbprm/interpolation/time-constraint-utils.hh>
#include <hpp/rbprm/rbprm-limb.hh>
#include <hpp/rbprm/sampling/sample.hh>

namespace hpp {
using namespace core;
  namespace rbprm {
  namespace interpolation {

    LimbRRTShooterPtr_t LimbRRTShooter::create (const core::DevicePtr_t  /*device*/,
                                                const hpp::core::PathPtr_t path,
                                                const std::size_t pathDofRank,
                                                const hpp::rbprm::RbPrmLimbPtr_t limb)
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
    , configSize_(pathDofRank+1)
    {
        // NOTHING
    }

    hpp::core::ConfigurationPtr_t LimbRRTShooter::shoot () const
    {
        // edit path sampling dof
        value_type a = path_->timeRange().first; value_type b = path_->timeRange().second;
        value_type u = value_type(rand()) / value_type(RAND_MAX);
        value_type pathDofVal = (b-a)* u + a;
        ConfigurationPtr_t config (new Configuration_t(configSize_));
        config->head(configSize_-1) =  (*path_)(pathDofVal);
        (*config) [pathDofRank_] = u;
        // choose random limb configuration
        const sampling::Sample& sample = *(limb_->sampleContainer_.samples_.begin() + (rand() % (int) (limb_->sampleContainer_.samples_.size() -1)));
        sampling::Load(sample,*config);
        return config;
    }

    LimbRRTShooterPtr_t LimbRRTShooterFactory::operator()(const RbPrmFullBodyPtr_t fullBody, const hpp::core::PathPtr_t path,
                    const std::size_t pathDofRank, const hpp::rbprm::State &from, const hpp::rbprm::State &to) const
    {
        const rbprm::T_Limb& limbs = fullBody->GetLimbs();
        std::vector<std::string> variations = to.allVariations(from, extractEffectorsName(limbs));
        return LimbRRTShooter::create(fullBody->device_,path,pathDofRank,limbs.at(*(variations.begin())));
    }
  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp
