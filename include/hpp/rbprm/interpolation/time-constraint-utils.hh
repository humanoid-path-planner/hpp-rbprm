//
// Copyright (c) 2014 CNRS
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
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_RBPRM_TIME_CONSTRAINT_UTILS_HH
# define HPP_RBPRM_TIME_CONSTRAINT_UTILS_HH

# include <hpp/rbprm/config.hh>
# include <hpp/rbprm/tools.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>
# include <hpp/rbprm/rbprm-state.hh>
# include <hpp/rbprm/rbprm-device.hh>
# include <hpp/rbprm/interpolation/time-constraint-steering.hh>
# include <hpp/rbprm/interpolation/time-constraint-helper.hh>
# include <hpp/rbprm/interpolation/time-constraint-path-validation.hh>
# include <hpp/core/problem.hh>

# include <vector>
# include <map>


namespace hpp {
    namespace rbprm {
    namespace interpolation {

    template<class Helper_T>
    void SetPathValidation(Helper_T& helper)
    {
        TimeConstraintPathValidationPtr_t pathVal = TimeConstraintPathValidation::create(
                    helper.fullBodyDevice_, 0.05,helper.fullBodyDevice_->configSize()-1);
        helper.rootProblem_.pathValidation(pathVal);
    }

    template<class Helper_T>
    core::ConfigurationPtr_t TimeConfigFromDevice(const Helper_T& helper, const State& state, const double time)
    {
        core::Configuration_t config(helper.fullBodyDevice_->currentConfiguration());
        config.head(state.configuration_.rows()) = state.configuration_;
        config[config.rows()-1] = time;
        return core::ConfigurationPtr_t(new core::Configuration_t(config));
    }

    inline std::vector<std::string> extractEffectorsName(const rbprm::T_Limb& limbs)
    {
        std::vector<std::string> res;
        for(rbprm::T_Limb::const_iterator cit = limbs.begin(); cit != limbs.end(); ++cit)
        {
            res.push_back(cit->first);
        }
        return res;
    }

    inline void DisableUnNecessaryCollisions(core::Problem& problem, rbprm::RbPrmLimbPtr_t limb)
    {
        // TODO should we really disable collisions for other bodies ?
        tools::RemoveNonLimbCollisionRec<core::Problem>(problem.robot()->rootJoint(),
                                                        "all",
                                                        //limb->limb_->name(),
                                                        problem.collisionObstacles(),problem);

        if(limb->disableEndEffectorCollision_)
        {
            hpp::tools::RemoveEffectorCollision<core::Problem>(problem,
                                                               problem.robot()->getJointByName(limb->effector_->name()),
                                                               problem.collisionObstacles());
        }
    }

    inline core::PathPtr_t generateRootPath(const core::Problem& problem, const State& from, const State& to)
    {
        core::Configuration_t startRootConf(from.configuration_);
        core::Configuration_t endRootConf(to.configuration_);
        return (*(problem.steeringMethod()))(startRootConf, endRootConf);
    }

    }
    }
}
#endif // HPP_RBPRM_TIME_CONSTRAINT_UTILS_HH
