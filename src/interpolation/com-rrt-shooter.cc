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
#include <hpp/rbprm/rbprm-limb.hh>
#include <hpp/rbprm/sampling/sample.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>

namespace hpp {
using namespace core;
  namespace rbprm {
  namespace interpolation {

    ComRRTShooterPtr_t ComRRTShooter::create (const core::DevicePtr_t  device,
                                                const hpp::core::PathPtr_t path,
                                                const std::size_t pathDofRank,
                                                const hpp::rbprm::RbPrmLimbPtr_t /*limb*/)
    {
        ComRRTShooter* ptr = new ComRRTShooter (device, path, pathDofRank);
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
                                  const hpp::core::PathPtr_t path,
                                  const std::size_t pathDofRank)
    : core::ConfigurationShooter()
    , path_(path)
    , pathDofRank_(pathDofRank)
    , configSize_(pathDofRank+1)
    , device_(device)
    {
        // NOTHING
    }

    hpp::core::ConfigurationPtr_t ComRRTShooter::shoot () const
    {
        JointVector_t jv = device_->getJointVector ();
        ConfigurationPtr_t config (new Configuration_t (configSize_));
        for (JointVector_t::const_iterator itJoint = jv.begin ();
             itJoint != jv.end (); itJoint++) {
          std::size_t rank = (*itJoint)->rankInConfiguration ();
          (*itJoint)->configuration ()->uniformlySample (rank, *config);
        }
        // edit path sampling dof
        value_type u = value_type(rand()) / value_type(RAND_MAX);
        (*config) [pathDofRank_] = u;
        value_type a = path_->timeRange().first; value_type b = path_->timeRange().second;
        value_type pathDofVal = (b-a)* u + a;
        config->head(2) =  (*path_)(pathDofVal).head(2); // put it close to com
        config->segment<4>(3) =  (*path_)(pathDofVal).segment<4>(3); // put it close to com
        return config;
    }
  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp
