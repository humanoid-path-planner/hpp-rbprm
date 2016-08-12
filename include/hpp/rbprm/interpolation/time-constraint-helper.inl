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

#ifndef HPP_RBPRM_TIME_CONSTRAINT_HELPER_UTILS_HH
# define HPP_RBPRM_TIME_CONSTRAINT_HELPER_UTILS_HH

#include <hpp/rbprm/interpolation/limb-rrt-shooter.hh>
#include <hpp/rbprm/interpolation/time-constraint-path-validation.hh>
#include <hpp/rbprm/interpolation/time-dependant.hh>
#include <hpp/rbprm/interpolation/time-constraint-utils.hh>
#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/problem-target/goal-configurations.hh>
#include <hpp/core/bi-rrt-planner.hh>
#include <hpp/core/random-shortcut.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/constraints/generic-transformation.hh>
#include <hpp/constraints/position.hh>
#include <hpp/constraints/orientation.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/locked-joint.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/subchain-path.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/object-factory.hh>
#include <hpp/rbprm/tools.hh>
#include <hpp/constraints/relative-com.hh>
#include <hpp/constraints/symbolic-calculus.hh>
#include <hpp/constraints/symbolic-function.hh>
#include <vector>

namespace hpp {
using namespace core;
using namespace model;
namespace rbprm {
namespace interpolation {

    namespace{
    inline std::vector<bool> setMaintainRotationConstraints()
    {
        std::vector<bool> res;
        for(std::size_t i =0; i <3; ++i)
            res.push_back(true);
        return res;
    }
    }

    template<class Path_T, class Shooter_T>
    void TimeConstraintHelper<Path_T, Shooter_T>::InitConstraints()
    {
        core::ConstraintSetPtr_t cSet = core::ConstraintSet::create(rootProblem_.robot(),"");
        cSet->addConstraint(proj_);
        rootProblem_.constraints(cSet);
    }

    template<class Path_T, class Shooter_T>
    void TimeConstraintHelper<Path_T, Shooter_T>::SetConfigShooter(const rbprm::RbPrmLimbPtr_t movingLimb)
    {
        rootProblem_.configurationShooter(Shooter_T::create(fullBodyDevice_, refPath_, fullBodyDevice_->configSize()-1, movingLimb));
    }

    template<class Path_T, class Shooter_T>
    void TimeConstraintHelper<Path_T, Shooter_T>::SetContactConstraints(const State& from, const State& to)
    {
        std::vector<bool> cosntraintsR = setMaintainRotationConstraints();
        std::vector<std::string> fixed = to.fixedContacts(from);
        model::DevicePtr_t device = rootProblem_.robot();

        for(std::vector<std::string>::const_iterator cit = fixed.begin();
            cit != fixed.end(); ++cit)
        {
            RbPrmLimbPtr_t limb = fullbody_->GetLimbs().at(*cit);
            const fcl::Vec3f& ppos  = from.contactPositions_.at(*cit);
            const fcl::Matrix3f& rotation = from.contactRotation_.at(*cit);
            JointPtr_t effectorJoint = device->getJointByName(limb->effector_->name());
            proj_->add(core::NumericalConstraint::create (
                                    constraints::deprecated::Position::create("",device,
                                                                  effectorJoint,fcl::Vec3f(0,0,0), ppos)));
            if(limb->contactType_ == hpp::rbprm::_6_DOF)
            {
                proj_->add(core::NumericalConstraint::create (constraints::deprecated::Orientation::create("", device,
                                                                                  effectorJoint,
                                                                                  rotation,
                                                                                  cosntraintsR)));
            }
        }
    }

    template<class Path_T, class Shooter_T>
    PathVectorPtr_t TimeConstraintHelper<Path_T, Shooter_T>::Run(const State &from, const State &to)
    {
        PathVectorPtr_t res;
        const rbprm::T_Limb& limbs = fullbody_->GetLimbs();
        // get limbs that moved
        std::vector<std::string> variations = to.allVariations(from, extractEffectorsName(limbs));
        for(std::vector<std::string>::const_iterator cit = variations.begin();
            cit != variations.end(); ++cit)
        {
            SetPathValidation(*this);
            DisableUnNecessaryCollisions(rootProblem_, limbs.at(*cit));
            SetConfigShooter(limbs.at(*cit));

            ConfigurationPtr_t start = TimeConfigFromDevice(*this, from, 0.);
            ConfigurationPtr_t end   = TimeConfigFromDevice(*this, to  , 1.);
            rootProblem_.initConfig(start);
            BiRRTPlannerPtr_t planner = BiRRTPlanner::create(rootProblem_);
            ProblemTargetPtr_t target = problemTarget::GoalConfigurations::create (planner);
            rootProblem_.target (target);
            rootProblem_.addGoalConfig(end);
            InitConstraints();

            res = planner->solve();
            rootProblem_.resetGoalConfigs();
        }
        return res;
    }


    namespace
    {
        template<class Helper_T>
        PathVectorPtr_t optimize(Helper_T& helper, PathVectorPtr_t partialPath, const std::size_t numOptimizations)
        {
            core::RandomShortcutPtr_t rs = core::RandomShortcut::create(helper.rootProblem_);
            for(std::size_t j=0; j<numOptimizations;++j)
            {
                partialPath = rs->optimize(partialPath);
            }
            return partialPath;
        }

        inline std::size_t checkPath(const std::size_t& distance, bool valid[])
        {
            std::size_t numValid(distance);
            for(std::size_t i = 0; i < distance; ++i)
            {
               if (!valid[i])
               {
                    numValid= i;
                    break;
               }
            }
            if (numValid==0)
                throw std::runtime_error("No path found at state 0");
            else if(numValid != distance)
            {
                std::cout << "No path found at state " << numValid << std::endl;
            }
            return numValid;
        }

        inline PathPtr_t ConcatenateAndResizePath(PathVectorPtr_t res[], std::size_t numValid)
        {
            PathVectorPtr_t completePath = res[0];
            for(std::size_t i = 1; i < numValid; ++i)
            {
                completePath->concatenate(*res[i]);
            }
            // reducing path
            core::SizeInterval_t interval(0, completePath->initial().rows()-1);
            core::SizeIntervals_t intervals;
            intervals.push_back(interval);
            PathPtr_t reducedPath = core::SubchainPath::create(completePath,intervals);
            return reducedPath;
        }
    }

    template<class Helper_T>
    PathPtr_t interpolateStatesFromPath(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem, const PathPtr_t refPath,
                                      const CIT_StateFrame &startState, const CIT_StateFrame &endState, const  std::size_t numOptimizations)
    {
        PathVectorPtr_t res[100];
        bool valid[100];
        std::size_t distance = std::distance(startState,endState);
        assert(distance < 100);
        // treat each interpolation between two states separatly
        // in a different thread
        #pragma omp parallel for
        for(std::size_t i = 0; i < distance; ++i)
        {
            CIT_StateFrame a, b;
            a = (startState+i);
            b = (startState+i+1);
            Helper_T helper(fullbody, referenceProblem,refPath);
                            //refPath->extract(core::interval_t(a->first, b->first)));
            helper.SetConstraints(a->second, b->second);
            PathVectorPtr_t partialPath = helper.Run(a->second, b->second);
            if(partialPath)
            {
                res[i] = optimize(helper,partialPath, numOptimizations);
                valid[i]=true;
            }
            else
            {
                valid[i] = false;
            }
        }
        std::size_t numValid = checkPath(distance, valid);
        return ConcatenateAndResizePath(res, numValid);
    }


    template<class Helper_T, typename StateConstIterator>
    PathPtr_t interpolateStates(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem,
                                      const StateConstIterator &startState, const StateConstIterator &endState, const std::size_t numOptimizations)
    {
        PathVectorPtr_t res[100];
        bool valid[100];
        std::size_t distance = std::distance(startState,endState);
        assert(distance < 100);
        // treat each interpolation between two states separatly
        // in a different thread
        #pragma omp parallel for
        for(std::size_t i = 0; i < distance; ++i)
        {
            StateConstIterator a, b;
            a = (startState+i);
            b = (startState+i+1);
            Helper_T helper(fullbody, referenceProblem, generateRootPath(*referenceProblem, *a, *b));
            helper.SetConstraints(*a, *b);
            PathVectorPtr_t partialPath = helper.Run(*a, *b);
            if(partialPath)
            {
                res[i] = optimize(helper,partialPath, numOptimizations);
                valid[i]=true;
            }
            else
            {
                valid[i] = false;
            }
        }
        std::size_t numValid = checkPath(distance, valid);
        return ConcatenateAndResizePath(res, numValid);
    }

  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp

#endif // HPP_RBPRM_TIME_CONSTRAINT_HELPER_UTILS_HH
