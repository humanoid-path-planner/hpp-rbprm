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

  rbprm::T_Limb GetVaryingLimb(const RbPrmFullBodyPtr_t fullBody, const hpp::rbprm::State &from, const hpp::rbprm::State &to)
  {
      rbprm::T_Limb res;
      const rbprm::T_Limb& limbs = fullBody->GetLimbs();
      std::vector<std::string> variations = to.allVariations(from, extractEffectorsName(limbs));
      const std::string limbName = *(variations.begin());
      rbprm::RbPrmLimbPtr_t limb = limbs.at(limbName);
      res.insert(std::make_pair(limbName,limb));
      return res;
  }

    TimeConstraintShooterPtr_t LimbRRTShooterFactory::operator()(const RbPrmFullBodyPtr_t fullBody, const hpp::core::PathPtr_t path,
                    const std::size_t pathDofRank, const hpp::rbprm::State &from, const hpp::rbprm::State &to,
                    const T_TimeDependant& tds, core::ConfigProjectorPtr_t projector) const
    {
        return TimeConstraintShooter::create(fullBody->device_,path,pathDofRank,tds, projector, GetVaryingLimb(fullBody, from, to));
    }
  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp
