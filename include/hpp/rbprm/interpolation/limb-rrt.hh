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

#ifndef HPP_RBPRM_LIMB_RRT_HH
# define HPP_RBPRM_LIMB_RRT_HH

# include <hpp/rbprm/config.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>
# include <hpp/rbprm/rbprm-state.hh>
# include <hpp/rbprm/rbprm-device.hh>
# include <hpp/rbprm/interpolation/time-constraint-steering.hh>
# include <hpp/rbprm/interpolation/time-constraint-helper.hh>
# include <hpp/core/path.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/config-projector.hh>

# include <vector>
# include <map>


namespace hpp {
    namespace rbprm {
    namespace interpolation {

    typedef TimeConstraintHelper<LimbRRTPath> helper_t;
    class LimbRRTHelper : public helper_t
    {
    public:
        LimbRRTHelper(RbPrmFullBodyPtr_t fullbody,
                      core::ProblemPtr_t referenceProblem,
                      core::PathPtr_t refPath):
            helper_t(fullbody,referenceProblem,refPath) {}
        ~LimbRRTHelper(){}
        virtual void SetConstraints(const State& from, const State& to);
        virtual core::PathVectorPtr_t Run(const State& from, const State& to);
    };

    typedef core::PathPtr_t (*interpolate_states)(rbprm::RbPrmFullBodyPtr_t, core::ProblemPtr_t,const rbprm::CIT_State&,
                                                       const rbprm::CIT_State&,const std::size_t);

    typedef core::PathPtr_t (*interpolate_states_from_path)(rbprm::RbPrmFullBodyPtr_t, core::ProblemPtr_t, const core::PathPtr_t,
                                                       const rbprm::CIT_StateFrame&,const rbprm::CIT_StateFrame&,const std::size_t);

    interpolate_states           const limbRRT = &interpolateStates<LimbRRTHelper, rbprm::CIT_State>;
    interpolate_states_from_path const limbRRTFromPath = &interpolateStatesFromPath<LimbRRTHelper>;
    }
    }
}


#endif // HPP_RBPRM_LIMB_RRT_HH
