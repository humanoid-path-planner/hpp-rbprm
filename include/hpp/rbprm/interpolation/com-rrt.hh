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

#ifndef HPP_RBPRM_COM_RRT_HH
# define HPP_RBPRM_COM_RRT_HH

# include <hpp/rbprm/config.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>
# include <hpp/rbprm/rbprm-state.hh>
# include <hpp/rbprm/rbprm-device.hh>
# include <hpp/rbprm/interpolation/time-constraint-steering.hh>
# include <hpp/rbprm/interpolation/time-constraint-helper.hh>
# include <hpp/rbprm/interpolation/time-constraint-path.hh>
# include <hpp/rbprm/interpolation/com-rrt-shooter.hh>
# include <hpp/core/path.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/config-projector.hh>

# include <vector>
# include <map>


namespace hpp {
    namespace rbprm {
    namespace interpolation {

    typedef TimeConstraintHelper<TimeConstraintPath, ComRRTShooter> com_parent_t;
    class ComRRTHelper : public com_parent_t
    {
    public:
        ComRRTHelper(RbPrmFullBodyPtr_t fullbody,
                      core::ProblemPtr_t referenceProblem,
                      core::PathPtr_t refPath):
            com_parent_t(fullbody,referenceProblem,refPath) {}
        ~ComRRTHelper(){}
        virtual void SetConstraints(const State& from, const State& to);
    };

    //interpolate_states           const comRRT         = &interpolateStates<ComRRTHelper, rbprm::CIT_State>;
    interpolate_states_from_path const comRRT = &interpolateStatesFromPath<ComRRTHelper>;
    }
    }
}


#endif // HPP_RBPRM_COM_RRT_HH
