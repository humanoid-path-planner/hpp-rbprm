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

    struct SetComRRTConstraints;

    typedef TimeConstraintHelper<TimeConstraintPath,ComRRTShooterFactory, SetComRRTConstraints> ComRRTHelper;
    struct SetComRRTConstraints
    {
        void operator ()(ComRRTHelper& helper, const State& from, const State& to) const;
    };

    core::PathPtr_t comRRT(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem, const PathPtr_t comPath,
                           const State &startState, const State &nextState,
                           const  std::size_t numOptimizations,
                           const bool keepExtraDof=false);

    core::PathPtr_t comRRTFromPath(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem, const PathPtr_t comPath,
                const PathPtr_t guidePath, const CIT_StateFrame &startState, const CIT_StateFrame &endState,
                const  std::size_t numOptimizations);

    core::Configuration_t projectOnCom(RbPrmFullBodyPtr_t fullbody,core::ProblemPtr_t referenceProblem, const State& model, const fcl::Vec3f& targetCom, bool& success);

    /*typedef std::vector<model::vector_t,Eigen::aligned_allocator<model::vector_t> > T_Configuration;
    core::PathPtr_t generateComTraj(const T_Configuration& configurations, const model::value_type dt, const model::ConfigurationIn_t & initSpeed, const model::ConfigurationIn_t endSpeed);*/
    }
    }
}


#endif // HPP_RBPRM_COM_RRT_HH
