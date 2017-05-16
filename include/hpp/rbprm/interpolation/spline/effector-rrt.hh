/// Copyright (c) 2015 CNRS
/// Authors: Joseph Mirabel
///
///
// This file is part of hpp-wholebody-step.
// hpp-wholebody-step-planner is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-wholebody-step-planner is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-wholebody-step-planner. If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_SPLINE_EFFECTOR_RRT_HH
# define HPP_SPLINE_EFFECTOR_RRT_HH

# include <hpp/rbprm/config.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>
# include <hpp/rbprm/rbprm-state.hh>
# include <hpp/rbprm/rbprm-device.hh>
# include <hpp/rbprm/interpolation/time-constraint-steering.hh>
# include <hpp/rbprm/interpolation/time-constraint-helper.hh>
# include <hpp/rbprm/interpolation/time-constraint-path.hh>
# include <hpp/rbprm/interpolation/interpolation-constraints.hh>
# include <hpp/rbprm/interpolation/com-rrt.hh>
# include <hpp/core/path.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/config-projector.hh>
# include <spline/exact_cubic.h>
# include <spline/bezier_curve.h>
# include <spline/spline_deriv_constraint.h>

# include <vector>
# include <map>


namespace hpp {
namespace rbprm {
namespace interpolation {
    struct SetEffectorRRTConstraints;

    typedef TimeConstraintHelper<TimeConstraintPath,EffectorRRTShooterFactory, SetEffectorRRTConstraints> EffectorRRTHelper;

    core::PathPtr_t effectorRRT(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem, const PathPtr_t comPath,
                                const  State &startState, const State &nextState,
                                const  std::size_t numOptimizations,
                                const bool keepExtraDof);

    core::PathPtr_t effectorRRT(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem, const PathPtr_t comPath,
                                const  State &startState, const State &nextState,
                                const  std::size_t numOptimizations,
                                const bool keepExtraDof,
                                const std::vector<std::string>& constrainedJointPos = std::vector<std::string>(),
                                const std::vector<std::string>& constrainedLockedJoints = std::vector<std::string>());


    core::PathPtr_t effectorRRTFromPath(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem, const PathPtr_t comPath,
                                        const  State &startState, const State &nextState,
                                        const  std::size_t numOptimizations,
                                        const bool keepExtraDof,
                                        const PathPtr_t refFullBodyPath,
                                        const std::vector<std::string>& constrainedJointPos = std::vector<std::string>(),
                                        const std::vector<std::string>& constrainedLockedJoints = std::vector<std::string>());

    typedef spline::exact_cubic<double, double, 3, true, Eigen::Matrix<value_type, 3, 1> > exact_cubic_t;
    typedef spline::spline_deriv_constraint<double, double, 3, true, Eigen::Matrix<value_type, 3, 1> > spline_deriv_constraint_t;
    typedef boost::shared_ptr<exact_cubic_t> exact_cubic_Ptr;

    struct SetEffectorRRTConstraints
    {
        SetEffectorRRTConstraints(const core::PathPtr_t refCom, const exact_cubic_Ptr refEff, const core::PathPtr_t refFullbody, const model::JointPtr_t  effector,
                                  const std::vector<model::JointPtr_t >& constrainedJointPos,const std::vector<model::JointPtr_t >& constrainedLockedJoints):
            refCom_(refCom), refFullbody_(refFullbody), refEff_ (refEff), effector_(effector),
            constrainedJointPos_(constrainedJointPos), constrainedLockedJoints_(constrainedLockedJoints) {}

        void operator ()(EffectorRRTHelper& helper, const State& from, const State& to) const;
        const core::PathPtr_t   refCom_;
        const core::PathPtr_t   refFullbody_;
        const exact_cubic_Ptr   refEff_;
        const model::JointPtr_t effector_;
        const std::vector<model::JointPtr_t > constrainedJointPos_;
        const std::vector<model::JointPtr_t > constrainedLockedJoints_;
    };


    } // namespace interpolation
  } // namespace rbprm
} // namespace hpp
#endif // HPP_SPLINE_EFFECTOR_TRAJECTORY_HH
