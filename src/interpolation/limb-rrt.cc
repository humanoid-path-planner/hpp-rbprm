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

#include <hpp/rbprm/interpolation/limb-rrt.hh>
#include <hpp/rbprm/interpolation/time-constraint-utils.hh>
#include <hpp/core/bi-rrt-planner.hh>

namespace hpp {
using namespace core;
  namespace rbprm {
  namespace interpolation {  

    void SetConfigShooter(LimbRRTHelper& helper, RbPrmLimbPtr_t limb, core::PathPtr_t& rootPath)
    {
        ConfigurationShooterPtr_t limbRRTShooter = LimbRRTShooter::create(limb, rootPath,
                                                                        helper.fullBodyDevice_->configSize()-1);
        helper.rootProblem_.configurationShooter(limbRRTShooter);
    }

    PathVectorPtr_t LimbRRTHelper::Run(const State &from, const State &to)
    {
        PathVectorPtr_t res;
        core::PathPtr_t rootPath = refPath_;
        const rbprm::T_Limb& limbs = fullbody_->GetLimbs();
        // get limbs that moved
        std::vector<std::string> variations = to.allVariations(from, extractEffectorsName(limbs));
        for(std::vector<std::string>::const_iterator cit = variations.begin();
            cit != variations.end(); ++cit)
        {
            SetPathValidation(*this);
            DisableUnNecessaryCollisions(rootProblem_, limbs.at(*cit));
            SetConfigShooter(*this,limbs.at(*cit),rootPath);

            ConfigurationPtr_t start = TimeConfigFromDevice(*this, from, 0.);
            ConfigurationPtr_t end   = TimeConfigFromDevice(*this, to  , 1.);
            rootProblem_.initConfig(start);
            BiRRTPlannerPtr_t planner = BiRRTPlanner::create(rootProblem_);
            ProblemTargetPtr_t target = problemTarget::GoalConfigurations::create (planner);
            rootProblem_.target (target);
            rootProblem_.addGoalConfig(end);
            SetContactConstraints(from, to);
            InitConstraints();

            res = planner->solve();
            rootProblem_.resetGoalConfigs();
        }
        return res;
    }

    void LimbRRTHelper::SetConstraints(const State& /*from*/, const State& /*to*/)
    {
        // TODO
    }
  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp
