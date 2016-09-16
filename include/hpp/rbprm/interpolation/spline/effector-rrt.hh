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

# include <vector>
# include <map>


namespace hpp {
namespace rbprm {
namespace interpolation {
    struct SetEffectorRRTConstraints;

    typedef TimeConstraintHelper<TimeConstraintPath,EffectorRRTShooterFactory, SetEffectorRRTConstraints> EffectorRRTHelper;

    // assumes extra dof is still present
    core::PathPtr_t effectorRRT(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem, const PathPtr_t comPath,
                                const  State &startState, const State &nextState,
                                const  std::size_t numOptimizations,
                                const bool keepExtraDof);


    typedef spline::bezier_curve<double, double, 1, true, Eigen::Matrix<value_type, 1, 1> > exact_cubic_t;
    typedef boost::shared_ptr<exact_cubic_t> exact_cubic_Ptr;

    struct SetEffectorRRTConstraints
    {
        SetEffectorRRTConstraints(const core::PathPtr_t refCom, const exact_cubic_Ptr refEff, const model::JointPtr_t  effector):
            refCom_(refCom), refEff_ (refEff), effector_(effector){}

        void operator ()(EffectorRRTHelper& helper, const State& from, const State& to) const
        {
            CreateContactConstraints<EffectorRRTHelper>(helper, from, to);
            //CreateComConstraint<EffectorRRTHelper,core::PathPtr_t >(helper, refCom_);
            CreateEffectorConstraint<EffectorRRTHelper, const exact_cubic_Ptr >(helper, refEff_, effector_);
        }
        const core::PathPtr_t   refCom_;
        const exact_cubic_Ptr   refEff_;
        const model::JointPtr_t effector_;
    };


    } // namespace interpolation
  } // namespace rbprm
} // namespace hpp
#endif // HPP_SPLINE_EFFECTOR_TRAJECTORY_HH
