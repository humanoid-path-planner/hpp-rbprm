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

#ifndef HPP_RBPRM_LIMB_RRT_HELPER_HH
# define HPP_RBPRM_LIMB_RRT_HELPER_HH

# include <hpp/rbprm/config.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>
# include <hpp/rbprm/rbprm-state.hh>
# include <hpp/rbprm/rbprm-device.hh>
# include <hpp/rbprm/interpolation/limb-rrt-steering.hh>
# include <hpp/core/path.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/config-projector.hh>

# include <vector>
# include <map>


namespace hpp {
    namespace rbprm {
    namespace interpolation {
    /// Helper struct for applying the LimbRRT algorithm. Maintains a pointer
    /// to a RbPrmFullbody object, and creates a new instance
    /// of a problem for a clone of the associated Device.
    ///
    template<class Path_T>
    struct HPP_CORE_DLLAPI LimbRRTHelper
    {
        /// \param fullbody robot considered for applying LimbRRT. The associated
        /// Device will be cloned to avoid side effects during the planning,
        /// An extra DOF is added to the cloned device, as required by the algorithm.
        /// \param referenceProblem an internal problem will be created,
        /// using this parameter as a reference, for retrieving collision obstacles
         LimbRRTHelper(RbPrmFullBodyPtr_t fullbody,
                       core::ProblemPtr_t referenceProblem,
                       core::PathPtr_t rootPath)
             : fullbody_(fullbody)
             , fullBodyDevice_(fullbody->device_->clone())
             , rootProblem_(fullBodyDevice_)
             , rootPath_(rootPath)
         {
             // adding extra DOF for including time in sampling
             fullBodyDevice_->setDimensionExtraConfigSpace(fullBodyDevice_->extraConfigSpace().dimension()+1);             
             proj_ = core::ConfigProjector::create(rootProblem_.robot(),"proj", 1e-2, 30);
             rootProblem_.collisionObstacles(referenceProblem->collisionObstacles());
             steeringMethod_ = LimbRRTSteering<LimbRRTPath>::create(&rootProblem_,fullBodyDevice_->configSize()-1);
             rootProblem_.steeringMethod(steeringMethod_);
         }

        ~LimbRRTHelper(){}

         RbPrmFullBodyPtr_t fullbody_;
         core::DevicePtr_t fullBodyDevice_;
         core::Problem rootProblem_;
         core::PathPlannerPtr_t planner_;
         core::PathPtr_t rootPath_;
         core::ConfigProjectorPtr_t proj_;
         boost::shared_ptr<LimbRRTSteering<LimbRRTPath> > steeringMethod_;
    };

    /// Runs the LimbRRT to create a kinematic, continuous,
    /// collision free path between the two State (ie, only one effector
    /// position differs between the states). Equilibirium is not
    /// verified along the path.
    /// To achieve this, an oriented RRT is run for the transitioning limb. The root path between
    /// two State is given by the problem steering method defined in the helper parameter.
    /// An extra DOF is used to sample a root position along the normalized path as well
    /// as the limb configuration. To avoid going back and forth, two configurations can thus
    /// only be connected if the first configuration has a DOF value lower than the second one.
    /// The LimbRRT algorithm is a modification of the original algorithm introduced in Qiu et al.
    /// "A Hierarchical Framework for Realizing Dynamically-stable
    /// Motions of Humanoid Robot in Obstacle-cluttered Environments"
    /// If OpenMP is activated, the interpolation between the states is run in parallel
    /// WARNING: At the moment, no more than 100 states can be interpolated simultaneously
    /// TODO: include parametrization of shortcut algorithm
    ///
    /// \param helper holds the problem parameters and the considered device
    /// An extra DOF is added to the cloned device, as required by the algorithm.
    /// The method assumes that the steering method associated with the helper's rootProblem_
    /// produces a collision free path all parts of the Device different that the transitioning limb.
    /// \param from initial state
    /// \param to final state
    /// \return the resulting path vector
    template<class Path_T>
    core::PathVectorPtr_t HPP_RBPRM_DLLAPI interpolateStates(LimbRRTHelper<Path_T>& helper, const State& from, const State& to);

    /// Runs the LimbRRT to create a kinematic, continuous,
    /// collision free path between an ordered State contrainer (Between each consecutive state, only one effector
    /// position differs between the states). Equilibrium is not
    /// verified along the path.
    /// To achieve this, an oriented RRT is run for the transitioning limb. The root path between
    /// two State is given by the problem steering method defined in the helper parameter.
    /// An extra DOF is used to sample a root position along the normalized path as well
    /// as the limb configuration. To avoid going back and forth, two configurations can thus
    /// only be connected if the first configuration has a DOF value lower than the second one.
    /// The LimbRRT algorithm is a modification of the original algorithm introduced in Qiu et al.
    /// "A Hierarchical Framework for Realizing Dynamically-stable
    /// Motions of Humanoid Robot in Obstacle-cluttered Environments"
    /// If OpenMP is activated, the interpolation between the states is run in parallel
    /// WARNING: At the moment, no more than 100 states can be interpolated simultaneously
    /// TODO: include parametrization of shortcut algorithm
    ///
    /// \param helper holds the problem parameters and the considered device
    /// An extra DOF is added to the cloned device, as required by the algorithm.
    /// The method assumes that the steering method associated with the helper's rootProblem_
    /// produces a collision free path all parts of the Device different that the transitioning limb.
    /// Here the steering method of the reference problem will be used to
    /// generate a root path between each consecutive state
    /// with the assumption that the path is valid.
    /// \param iterator to the initial State
    /// \param to iterator to the final State
    /// \param numOptimizations Number of iterations of the shortcut algorithm to apply between each states
    /// \return the resulting path vector, concatenation of all the interpolation paths between the State
    template<class Path_T, typename StateConstIterator>
    core::PathPtr_t HPP_RBPRM_DLLAPI interpolateStates(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem,
                                                             const StateConstIterator& startState,
                                                             const StateConstIterator& endState,
                                                             const std::size_t numOptimizations = 10);
    /// Runs the LimbRRT to create a kinematic, continuous,
    /// collision free path between an ordered State contrainer (Between each consecutive state, only one effector
    /// position differs between the states). Equilibrium is not
    /// verified along the path.
    /// To achieve this, an oriented RRT is run for the transitioning limb. The root path between
    /// two State is given by the problem steering method defined in the helper parameter.
    /// An extra DOF is used to sample a root position along the normalized path as well
    /// as the limb configuration. To avoid going back and forth, two configurations can thus
    /// only be connected if the first configuration has a DOF value lower than the second one.
    /// The LimbRRT algorithm is a modification of the original algorithm introduced in Qiu et al.
    /// "A Hierarchical Framework for Realizing Dynamically-stable
    /// Motions of Humanoid Robot in Obstacle-cluttered Environments"
    /// If OpenMP is activated, the interpolation between the states is run in parallel
    /// WARNING: At the moment, no more than 100 states can be interpolated simultaneously
    /// TODO: include parametrization of shortcut algorithm
    ///
    /// \param helper holds the problem parameters and the considered device
    /// An extra DOF is added to the cloned device, as required by the algorithm.
    /// The method assumes that the steering method associated with the helper's rootProblem_
    /// produces a collision free path all parts of the Device different that the transitioning limb.
    /// \param path reference path for the root
    /// \param iterator to the initial State with its associated keyFrame in the path
    /// \param to iterator to the final State with its associated keyFrame in the path
    /// \param numOptimizations Number of iterations of the shortcut algorithm to apply between each states
    /// \return the resulting path vector, concatenation of all the interpolation paths between the State
    template<class Path_T>
    core::PathPtr_t HPP_RBPRM_DLLAPI interpolateStates(RbPrmFullBodyPtr_t fullbody,
                                                             core::ProblemPtr_t referenceProblem,
                                                             const core::PathPtr_t rootPath,
                                                             const CIT_StateFrame& startState,
                                                             const CIT_StateFrame& endState,
                                                             const std::size_t numOptimizations = 10);

    /// Runs the LimbRRT to create a kinematic, continuous,
    /// collision free path between an ordered State contrainer (Between each consecutive state, only one effector
    /// position differs between the states). Equilibrium is not
    /// verified along the path.
    /// To achieve this, an oriented RRT is run for the transitioning limb. The com trajectory
    /// between the two State is given by the problem steering method defined in the helper parameter.
    /// An extra DOF is used to sample a root position along the normalized path as well
    /// as the limb configuration. To avoid going back and forth, two configurations can thus
    /// only be connected if the first configuration has a DOF value lower than the second one.
    /// The LimbRRT algorithm is a modification of the original algorithm introduced in Qiu et al.
    /// "A Hierarchical Framework for Realizing Dynamically-stable
    /// Motions of Humanoid Robot in Obstacle-cluttered Environments"
    /// If OpenMP is activated, the interpolation between the states is run in parallel
    /// WARNING: At the moment, no more than 100 states can be interpolated simultaneously
    /// TODO: include parametrization of shortcut algorithm
    ///
    /// \param helper holds the problem parameters and the considered device
    /// An extra DOF is added to the cloned device, as required by the algorithm.
    /// The method assumes that the steering method associated with the helper's rootProblem_
    /// produces a collision free path all parts of the Device different that the transitioning limb.
    /// \param comPath reference path for the COM
    /// \param iterator to the initial State with its associated keyFrame in the path
    /// \param to iterator to the final State with its associated keyFrame in the path
    /// \param numOptimizations Number of iterations of the shortcut algorithm to apply between each states
    /// \return the resulting path vector, concatenation of all the interpolation paths between the State
    template<class Path_T>
    core::PathPtr_t HPP_RBPRM_DLLAPI interpolateStatesTrackCOM(RbPrmFullBodyPtr_t fullbody,
                                                             core::ProblemPtr_t referenceProblem,
                                                             const core::PathPtr_t comPath,
                                                             const CIT_StateFrame& startState,
                                                             const CIT_StateFrame& endState,
                                                             const std::size_t numOptimizations = 10);

    } // namespace interpolation
    } // namespace rbprm
  } // namespace hpp


#include <hpp/rbprm/interpolation/limb-rrt-helper.inl>
#endif // HPP_RBPRM_LIMB_RRT_HELPER_HH
