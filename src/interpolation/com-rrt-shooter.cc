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

    TimeConstraintShooterPtr_t ComRRTShooterFactory::operator()(const RbPrmFullBodyPtr_t fullBody, const hpp::core::PathPtr_t /*comPath*/,
                    const std::size_t pathDofRank, const hpp::rbprm::State &from, const hpp::rbprm::State &to,
                    const T_TimeDependant& tds, core::ConfigProjectorPtr_t projector) const
    {
        return TimeConstraintShooter::create(fullBody->device_,guidePath_,pathDofRank,tds, projector, GetFreeLimbs(fullBody, from, to));
    }

    TimeConstraintShooterPtr_t EffectorRRTShooterFactory::operator()(const RbPrmFullBodyPtr_t fullBody, const hpp::core::PathPtr_t /*comPath*/,
                    const std::size_t pathDofRank, const hpp::rbprm::State &from, const hpp::rbprm::State &to,
                    const T_TimeDependant& tds, core::ConfigProjectorPtr_t projector) const
    {
        rbprm::T_Limb res;
        return TimeConstraintShooter::create(fullBody->device_,guidePath_,pathDofRank,tds, projector, res);
    }
  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp
