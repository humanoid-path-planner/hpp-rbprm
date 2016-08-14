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

#include <hpp/rbprm/interpolation/com-rrt-shooter.hh>
#include <hpp/rbprm/interpolation/time-constraint-utils.hh>
#include <hpp/rbprm/rbprm-limb.hh>
#include <hpp/rbprm/sampling/sample.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>

namespace hpp {
using namespace core;
  namespace rbprm {
  namespace interpolation {

    ComRRTShooterPtr_t ComRRTShooter::create (const core::DevicePtr_t  device,
                                              const hpp::core::PathPtr_t comPath,
                                              const hpp::core::PathPtr_t rootPath,
                                              const std::size_t pathDofRank,
                                              const rbprm::T_Limb freeLimbs)
    {
        ComRRTShooter* ptr = new ComRRTShooter (device, comPath, rootPath, pathDofRank, freeLimbs);
        ComRRTShooterPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
    }

    void ComRRTShooter::init (const ComRRTShooterPtr_t& self)
    {
        core::ConfigurationShooter::init (self);
        weak_ = self;
    }

    ComRRTShooter::ComRRTShooter (const core::DevicePtr_t  device,
                                  const hpp::core::PathPtr_t comPath,
                                  const hpp::core::PathPtr_t rootPath,
                                  const std::size_t pathDofRank, const hpp::rbprm::T_Limb freeLimbs)
    : core::ConfigurationShooter()
    , comPath_(comPath)
    , rootPath_(rootPath)
    , pathDofRank_(pathDofRank)
    , configSize_(pathDofRank+1)
    , device_(device)
    , freeLimbs_(freeLimbs)
    {
        // NOTHING
    }

    hpp::core::ConfigurationPtr_t ComRRTShooter::shoot () const
    {
        // edit path sampling dof
        value_type a = rootPath_->timeRange().first; value_type b = rootPath_->timeRange().second;
        value_type u = value_type(rand()) / value_type(RAND_MAX);
        value_type pathDofVal = (b-a)* u + a;
        ConfigurationPtr_t config (new Configuration_t(configSize_));
        config->head(configSize_-1) =  (*rootPath_)(pathDofVal);
        (*config) [pathDofRank_] = u;
        // choose random limb configuration
        for(rbprm::CIT_Limb cit = freeLimbs_.begin(); cit != freeLimbs_.end(); ++cit)
        {
            const rbprm::RbPrmLimbPtr_t limb = cit->second;
            const sampling::Sample& sample = *(limb->sampleContainer_.samples_.begin() + (rand() % (int) (limb->sampleContainer_.samples_.size() -1)));
            sampling::Load(sample,*config);
        }
        return config;
        /*JointVector_t jv = device_->getJointVector ();
        ConfigurationPtr_t config (new Configuration_t (configSize_));
        for (JointVector_t::const_iterator itJoint = jv.begin ();
             itJoint != jv.end (); itJoint++) {
          std::size_t rank = (*itJoint)->rankInConfiguration ();
          (*itJoint)->configuration ()->uniformlySample (rank, *config);
        }
        // edit path sampling dof
        value_type u = value_type(rand()) / value_type(RAND_MAX);
        (*config) [pathDofRank_] = u;
        value_type a = comPath_->timeRange().first; value_type b = comPath_->timeRange().second;
        value_type pathDofVal = (b-a)* u + a;
        config->head(2) =  (*comPath_)(pathDofVal).head(2); // put it close to com
        config->segment<4>(3) =  (*comPath_)(pathDofVal).segment<4>(3); // put it close to com
        return config;*/
    }


    rbprm::T_Limb GetFreeLimbs(const RbPrmFullBodyPtr_t fullBody, const hpp::rbprm::State &from, const hpp::rbprm::State &to)
    {
        rbprm::T_Limb res;
        std::vector<std::string> fixedContacts = to.fixedContacts(from);
        for(rbprm::CIT_Limb cit = fullBody->GetLimbs().begin();
            cit != fullBody->GetLimbs().end(); ++cit)
        {
            if(std::find(fixedContacts.begin(), fixedContacts.end(), cit->first) == fixedContacts.end())
            {
                res.insert(*cit);
            }
        }
        return res;
    }

    ComRRTShooterPtr_t ComRRTShooterFactory::operator()(const RbPrmFullBodyPtr_t fullBody, const hpp::core::PathPtr_t comPath,
                    const std::size_t pathDofRank, const hpp::rbprm::State &from, const hpp::rbprm::State &to) const
    {
        return ComRRTShooter::create(fullBody->device_,comPath,guidePath_,pathDofRank,GetFreeLimbs(fullBody, from, to));
    }
  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp
