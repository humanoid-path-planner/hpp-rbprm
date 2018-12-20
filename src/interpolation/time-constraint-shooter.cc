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
#include <hpp/pinocchio/joint.hh>

namespace hpp {
using namespace core;
  namespace rbprm {
  namespace interpolation {

    TimeConstraintShooterPtr_t TimeConstraintShooter::create (const core::DevicePtr_t  device,
                                              const hpp::core::PathPtr_t rootPath,
                                              const std::size_t pathDofRank,
                                              const T_TimeDependant& tds,
                                              core::ConfigProjectorPtr_t projector,
                                              const rbprm::T_Limb freeLimbs)
    {
        TimeConstraintShooter* ptr = new TimeConstraintShooter (device, rootPath, pathDofRank, tds, projector, freeLimbs);
        TimeConstraintShooterPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
    }

    void TimeConstraintShooter::init (const TimeConstraintShooterPtr_t& self)
    {
        core::ConfigurationShooter::init (self);
        weak_ = self;
    }

    TimeConstraintShooter::TimeConstraintShooter (const core::DevicePtr_t  device,
                                  const hpp::core::PathPtr_t rootPath,
                                  const std::size_t pathDofRank,
                                  const T_TimeDependant& tds,
                                  core::ConfigProjectorPtr_t projector,
                                  const hpp::rbprm::T_Limb freeLimbs)
    : core::ConfigurationShooter()
    , rootPath_(rootPath)
    , pathDofRank_(pathDofRank)
    , configSize_(pathDofRank+1)
    , device_(device)
    , freeLimbs_(freeLimbs)
    , tds_(tds)
    , projector_(projector)
    {
        // NOTHING
    }

    hpp::core::ConfigurationPtr_t TimeConstraintShooter::shoot () const
    {
        // edit path sampling dof
        value_type a = rootPath_->timeRange().first; value_type b = rootPath_->timeRange().second;
        value_type u = value_type(rand()) / value_type(RAND_MAX);
        value_type pathDofVal = (b-a)* u + a;
        ConfigurationPtr_t config (new Configuration_t(configSize_));
        config->head(configSize_-1) =  (*rootPath_)(pathDofVal);
        (*config) [pathDofRank_] = u;
        /*if(freeLimbs_.empty())
        {
            JointVector_t jv = device_->getJointVector ();
            for (JointVector_t::const_iterator itJoint = jv.begin ();
                 itJoint != jv.end (); itJoint++) {
              std::size_t rank = (*itJoint)->rankInConfiguration ();
              (*itJoint)->configuration ()->uniformlySample (rank, *config);
            }
        }
        else*/
        {
            // choose random limb configuration
            for(rbprm::CIT_Limb cit = freeLimbs_.begin(); cit != freeLimbs_.end(); ++cit)
            {
                const rbprm::RbPrmLimbPtr_t limb = cit->second;
                const int rand_int = (rand() % (int) (limb->sampleContainer_.samples_.size() -1));
                const sampling::Sample& sample = *(limb->sampleContainer_.samples_.begin() + rand_int);
                sampling::Load(sample,*config);
            }
        }
        UpdateConstraints(*config, tds_, pathDofRank_);
        return config;
    }
  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp
