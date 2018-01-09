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
#include <hpp/rbprm/interpolation/time-constraint-utils.hh>
#include <hpp/rbprm/interpolation/time-constraint-path-validation.hh>
#include <hpp/rbprm/interpolation/time-dependant.hh>
#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/problem-target/goal-configurations.hh>
#include <hpp/core/bi-rrt-planner.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/random-shortcut.hh>
#include <hpp/core/path-optimization/partial-shortcut.hh>
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

    template<class Path_T, class ShooterFactory_T, typename ConstraintFactory_T>
    void TimeConstraintHelper<Path_T, ShooterFactory_T, ConstraintFactory_T>::InitConstraints()
    {
        core::ConstraintSetPtr_t cSet = core::ConstraintSet::create(rootProblem_.robot(),"");
        cSet->addConstraint(proj_);
        rootProblem_.constraints(cSet);
    }

    template<class Path_T, class ShooterFactory_T, typename ConstraintFactory_T>
    void TimeConstraintHelper<Path_T, ShooterFactory_T, ConstraintFactory_T>::SetConfigShooter(const hpp::rbprm::State &from, const hpp::rbprm::State &to)
    {
        rootProblem_.configurationShooter(shooterFactory_(fullbody_, refPath_, fullBodyDevice_->configSize()-1, from, to,
                                                          steeringMethod_->tds_, proj_));
    }

namespace
{
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
                                                        //"all",
                                                        limb->limb_->name(),
                                                        problem.collisionObstacles(),problem);

        if(limb->disableEndEffectorCollision_)
        {
            hpp::tools::RemoveEffectorCollision<core::Problem>(problem,
                                                               problem.robot()->getJointByName(limb->effector_->name()),
                                                               problem.collisionObstacles());
        }
    }


    inline void DisableUnNecessaryCollisions(core::Problem& problem, const std::vector<std::string>& variations,  const rbprm::T_Limb& limbs)
    {
        std::vector<std::string> jointNamesVariations;
        for(std::vector<std::string>::const_iterator cit = variations.begin();
                                cit != variations.end(); ++cit)
        {
            rbprm::RbPrmLimbPtr_t limb = limbs.at(*cit);
            jointNamesVariations.push_back(limb->limb_->name());
            if(limb->disableEndEffectorCollision_)
            {
                hpp::tools::RemoveEffectorCollision<core::Problem>(problem,
                                                                   problem.robot()->getJointByName(limb->effector_->name()),
                                                                   problem.collisionObstacles());
            }
        }
        // TODO should we really disable collisions for other bodies ?
        tools::RemoveNonLimbCollisionRec<core::Problem>(problem.robot()->rootJoint(),
                                                        jointNamesVariations,
                                                        problem.collisionObstacles(),problem);
    }

    inline core::PathPtr_t generateRootPath(const core::Problem& problem, const State& from, const State& to)
    {
        core::Configuration_t startRootConf(from.configuration_);
        core::Configuration_t endRootConf(to.configuration_);
        return (*(problem.steeringMethod()))(startRootConf, endRootConf);
    }
}

    template<class Path_T, class Shooter_T, typename ConstraintFactory_T>
    PathVectorPtr_t TimeConstraintHelper<Path_T, Shooter_T, ConstraintFactory_T>::Run(const State &from, const State &to,const size_t maxIterations)
    {
        PathVectorPtr_t res;
        SetPathValidation(*this);
        const rbprm::T_Limb& limbs = fullbody_->GetLimbs();
        // get limbs that moved
        std::vector<std::string> variations = to.allVariations(from, extractEffectorsName(limbs));
        if(!variations.empty())
        {
            DisableUnNecessaryCollisions(rootProblem_, variations, limbs);
        }
        // TODO IS THIS REDUNDANT ?
        /*for(rbprm::CIT_Limb lit = limbs.begin(); lit != limbs.end(); ++lit)
        {
            if(lit->second->disableEndEffectorCollision_ && std::find(variations.begin(), variations.end(),
                                                                 lit->second->limb_->name()) == variations.end())
            {
                hpp::tools::RemoveEffectorCollision<core::Problem>(rootProblem_,
                                                                   rootProblem_.robot()->getJointByName(lit->second->effector_->name()),
                                                                   rootProblem_.collisionObstacles());
            }
        }*/
        SetConfigShooter(from, to);

        ConfigurationPtr_t start = TimeConfigFromDevice(*this, from, 0.);
        ConfigurationPtr_t end   = TimeConfigFromDevice(*this, to  , 1.);
        rootProblem_.initConfig(start);
        BiRRTPlannerPtr_t planner = BiRRTPlanner::create(rootProblem_);
        ProblemTargetPtr_t target = problemTarget::GoalConfigurations::create (planner);
        rootProblem_.target (target);
        rootProblem_.addGoalConfig(end);
        InitConstraints();
        if(maxIterations>0)
            planner->maxIterations(maxIterations);
        boost::posix_time::ptime timeStart(boost::posix_time::microsec_clock::universal_time());
        res = planner->solve();
        hppDout(notice,"TimeConstraintHelper::Run : solved in "<<((boost::posix_time::microsec_clock::universal_time() - timeStart).total_milliseconds())<<" ms");
        hppDout(notice,"With a roadmap of "<<planner->roadmap()->nodes().size()<<" nodes");
        rootProblem_.resetGoalConfigs();
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
                hppDout(notice,"Optimize random shortucut, iter : "<<j);
                partialPath = rs->optimize(partialPath);
            }
            core::pathOptimization::PartialShortcutPtr_t rs2 = core::pathOptimization::PartialShortcut::create(helper.rootProblem_);
            for(std::size_t j=0; j<numOptimizations;++j)
            {
                hppDout(notice,"Optimize partial shortcut, iter : "<<j);
                partialPath = rs2->optimize(partialPath);
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

        inline PathPtr_t ConcatenateAndResizePath(PathVectorPtr_t res[], std::size_t numValid, const bool keepExtraDof)
        {
            PathVectorPtr_t completePath = res[0];
            for(std::size_t i = 1; i < numValid; ++i)
            {
                completePath->concatenate(*res[i]);
            }
            if(keepExtraDof)
            {
                return completePath;
            }
            else
            {
                // reducing path
                core::SizeInterval_t interval(0, completePath->initial().rows()-1);
                core::SizeIntervals_t intervals;
                intervals.push_back(interval);
                PathPtr_t reducedPath = core::SubchainPath::create(completePath,intervals);
                return reducedPath;
            }
        }
    }

    struct GenPath
    {
        GenPath(const core::Problem& problem) : problem_(problem) {}
       ~GenPath() {}
        PathPtr_t operator ()(const CIT_State& from, const CIT_State& to) const
        {
            return interpolation::generateRootPath(problem_, *from, *to);
        }
        const core::Problem& problem_;
    };

    struct ExtractPath
    {
         ExtractPath(PathPtr_t refPath) : refPath_(refPath) {}
        ~ExtractPath() {}
        PathPtr_t operator ()(const CIT_StateFrame& from, CIT_StateFrame& to) const
        {
            return refPath_->extract(core::interval_t(from->first, to->first));
        }
        PathPtr_t refPath_;
    };

    inline const State& get(const CIT_StateFrame& from)
    {
        return from->second;
    }

    inline const State& get(const CIT_State& from)
    {
        return *from;
    }

    template<class Helper_T, class StateIterator_T,class ShooterFactory_T, typename ConstraintFactory_T, class PathGetter_T>
    PathPtr_t interpolateStatesFromPathGetter(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem,
                                              const ShooterFactory_T& shooterFactory, const ConstraintFactory_T& constraintFactory,
                                              const PathGetter_T& pathGetter,
                                              const StateIterator_T &startState, const StateIterator_T &endState,
                                              const std::size_t numOptimizations, const bool keepExtraDof=false,
                                              const model::value_type error_treshold = 0.001, const size_t maxIterations = 0)
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
            StateIterator_T a, b;
            a = (startState+i);
            b = (startState+i+1);
            Helper_T helper(fullbody, shooterFactory, constraintFactory, referenceProblem, pathGetter(a,b),error_treshold);
            helper.SetConstraints(get(a), get(b));
            PathVectorPtr_t partialPath = helper.Run(get(a), get(b),maxIterations);
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
        return ConcatenateAndResizePath(res, numValid, keepExtraDof);
    }

    template<class Helper_T, class ShooterFactory_T, typename ConstraintFactory_T>
    PathPtr_t interpolateStatesFromPath(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem,
                                        const ShooterFactory_T& shooterFactory, const ConstraintFactory_T& constraintFactory,
                                        const PathPtr_t refPath,
                                        const CIT_StateFrame &startState, const CIT_StateFrame &endState,
                                        const std::size_t numOptimizations,
                                        const bool keepExtraDof,
                                        const value_type error_treshold,
                                        const size_t maxIterations)
    {
        ExtractPath extractPath(refPath);
        return interpolateStatesFromPathGetter<Helper_T, CIT_StateFrame, ShooterFactory_T, ConstraintFactory_T, ExtractPath>
                (fullbody,referenceProblem, shooterFactory, constraintFactory, extractPath,startState,endState,numOptimizations,keepExtraDof, error_treshold,maxIterations);
    }


    template<class Helper_T, class ShooterFactory_T, typename ConstraintFactory_T, typename StateConstIterator>
    PathPtr_t interpolateStates(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem,
                                const ShooterFactory_T& shooterFactory, const ConstraintFactory_T& constraintFactory,
                                const StateConstIterator &startState, const StateConstIterator &endState,
                                const std::size_t numOptimizations, const bool keepExtraDof,
                                const value_type error_treshold,
                                const size_t maxIterations)
    {        
        GenPath genPath(*referenceProblem);
        return interpolateStatesFromPathGetter<Helper_T, StateConstIterator, ShooterFactory_T, ConstraintFactory_T, GenPath>
                (fullbody,referenceProblem, shooterFactory, constraintFactory,
                 genPath,startState,endState,numOptimizations,keepExtraDof, error_treshold,maxIterations);
    }

  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp

#endif // HPP_RBPRM_TIME_CONSTRAINT_HELPER_UTILS_HH
