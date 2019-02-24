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

#include <hpp/rbprm/interpolation/com-rrt.hh>
#include <hpp/rbprm/interpolation/limb-rrt.hh>
#include <hpp/rbprm/interpolation/time-constraint-utils.hh>
#include <hpp/rbprm/interpolation/interpolation-constraints.hh>
#include <hpp/rbprm/tools.hh>
#include <hpp/core/bi-rrt-planner.hh>
#include <hpp/core/configuration-shooter/uniform.hh>
#include <hpp/core/path-validation/discretized.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/core/problem-solver.hh>
#ifdef PROFILE
#include "hpp/rbprm/rbprm-profiler.hh"
#endif

namespace hpp {
using namespace core;
  namespace rbprm {
  namespace interpolation {

    void SetComRRTConstraints::operator ()(ComRRTHelper& helper, const State& from, const State& to) const
    {
        CreateContactConstraints<ComRRTHelper>(helper, from, to);
        CreateComConstraint<ComRRTHelper,core::PathPtr_t>(helper, helper.refPath_);
        // add postural task to respect the constraint during all the motion but greatly increase computation time
        // FIXME : can lead to discontinuities ...
      /*  Configuration_t refConfig = helper.fullbody_->referenceConfig();
        CreatePosturalTaskConstraint<ComRRTHelper,Configuration_t>(helper, refConfig);
        helper.proj_->lastIsOptional(true);
        helper.proj_->numOptimize(500);
        helper.proj_->lastAsCost(true);
        helper.proj_->errorThreshold(1e-3);*/
    }

    core::PathPtr_t comRRT(RbPrmFullBodyPtr_t fullbody, ProblemSolverPtr_t problemSolver, const PathPtr_t comPath,
                           const  State &startState, const State &nextState,
                           const  std::size_t numOptimizations,
                           const bool keepExtraDof){

        return comRRT(fullbody,problemSolver->problem(),comPath,startState,nextState,numOptimizations,keepExtraDof);
    }

    core::PathPtr_t comRRT(RbPrmFullBodyPtr_t fullbody, ProblemPtr_t referenceProblem, const PathPtr_t comPath,
                           const  State &startState, const State &nextState,
                           const  std::size_t numOptimizations,
                           const bool keepExtraDof)
    {
        hppDout(notice,"Begin interpolation::comRRT.");
        //check whether there is a contact variations
        std::vector<std::string> variations = nextState.allVariations(startState, extractEffectorsName(fullbody->GetLimbs()));
        core::PathPtr_t guidePath;
        T_State states; states.push_back(startState); states.push_back(nextState);
        T_StateFrame stateFrames;
        stateFrames.push_back(std::make_pair(comPath->timeRange().first, startState));
        stateFrames.push_back(std::make_pair(comPath->timeRange().second, nextState));
        hppDout(notice,"comRRT : comPath length : "<<comPath->length());
        hppDout(notice,"comRRT : startState : r(["<<pinocchio::displayConfig(startState.configuration_)<<"])");
        hppDout(notice,"comRRT : nextState  : r(["<<pinocchio::displayConfig(nextState.configuration_)<<"])");

        if(variations.empty())
        {
            hppDout(notice,"No contact variation, use comRRT.");
            std::vector<std::string> fixed = nextState.fixedContacts(startState);
            pinocchio::DevicePtr_t device = fullbody->device_->clone();
            /*if(keepExtraDof)
            {
                device->setDimensionExtraConfigSpace(device->extraConfigSpace().dimension()+1);
            }*/
            core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(device,"proj", 1e-3, 1000);
            core::ProblemPtr_t rootProblem = core::Problem::create(device);
            for(std::vector<std::string>::const_iterator cit = fixed.begin();
                cit != fixed.end(); ++cit)
            {
            }
            rootProblem->configurationShooter(core::configurationShooter::Uniform::create(device));
            rootProblem->pathValidation(pathValidation::Discretized::create(0.05));
            core::ConstraintSetPtr_t cSet = core::ConstraintSet::create(rootProblem->robot(),"");
            cSet->addConstraint(proj);
            rootProblem->constraints(cSet);
            ConfigurationPtr_t start =  ConfigurationPtr_t(new Configuration_t(startState.configuration_));
            ConfigurationPtr_t end =  ConfigurationPtr_t(new Configuration_t(nextState.configuration_));
            rootProblem->initConfig(start);
            BiRRTPlannerPtr_t planner = BiRRTPlanner::create(*rootProblem);
            ProblemTargetPtr_t target = problemTarget::GoalConfigurations::create (rootProblem);
            rootProblem->target (target);
            rootProblem->addGoalConfig(end);
            hppDout(notice,"Start solve");

            guidePath = planner->solve();
            hppDout(notice,"Solve success");
            //return guidePath;
        }
        else
        {
            hppDout(notice,"Contact variations, use limbRRT.");
            guidePath = limbRRT(fullbody,referenceProblem,states.begin(),states.begin()+1,numOptimizations,10000);
            hppDout(notice,"LimbRRT done.");

        }
        ComRRTShooterFactory shooterFactory(guidePath);
        SetComRRTConstraints constraintFactory;
#ifdef PROFILE
    RbPrmProfiler& watch = getRbPrmProfiler();
    watch.start("com_traj");
#endif
        hppDout(notice,"Start interpolateStatesFromPath");
        core::PathPtr_t resPath = interpolateStatesFromPath<ComRRTHelper, ComRRTShooterFactory, SetComRRTConstraints>
              (fullbody, referenceProblem, shooterFactory, constraintFactory, comPath, stateFrames.begin(), stateFrames.begin()+1, numOptimizations, keepExtraDof,0.05,10000);
        hppDout(notice,"interpolateStatesFromPath end.");
#ifdef PROFILE
    watch.stop("com_traj");
#endif
        return resPath;
    }

    core::PathPtr_t comRRTFromPath(RbPrmFullBodyPtr_t fullbody, core::ProblemSolverPtr_t problemSolver, const PathPtr_t comPath,
                           const PathPtr_t guidePath, const CIT_StateFrame &startState, const CIT_StateFrame &endState,
                           const  std::size_t numOptimizations)
    {
        ComRRTShooterFactory shooterFactory(guidePath);
        SetComRRTConstraints constraintFactory;
        return interpolateStatesFromPath<ComRRTHelper, ComRRTShooterFactory, SetComRRTConstraints>
                (fullbody, problemSolver->problem(), shooterFactory, constraintFactory, comPath, startState, endState, numOptimizations);
    }

    core::Configuration_t projectOnCom(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem, const State& model, const fcl::Vec3f& targetCom, bool &success)
    {
        core::PathPtr_t unusedPath(StraightPath::create(fullbody->device_,model.configuration_, model.configuration_,0));
        ComRRTShooterFactory unusedFactory(unusedPath);
        SetComRRTConstraints constraintFactory;
        ComRRTHelper helper(fullbody, unusedFactory, constraintFactory, referenceProblem,unusedPath, 0.001);
        CreateContactConstraints<ComRRTHelper>(helper,model,model);
        CreateComConstraint<ComRRTHelper,core::PathPtr_t>(helper, helper.refPath_,targetCom);

       // Configuration_t refConfig = fullbody->referenceConfig();
       // CreatePosturalTaskConstraint<ComRRTHelper,Configuration_t>(helper, refConfig);
      //  helper.proj_->lastIsOptional(true);
        helper.proj_->maxIterations(100);
        //helper.proj_->lastAsCost(true);
        helper.proj_->errorThreshold(1e-3);

        Configuration_t res(helper.fullBodyDevice_->configSize());
        res.head(model.configuration_.rows()) = model.configuration_;
        if(helper.proj_->apply(res))
        {
            success = true;
            /*hppDout(notice,"projection successfull, trying to optimize : ");
            CreatePosturalTaskConstraint<ComRRTHelper,ConfigurationPtr_t>(helper, refConfig);
            helper.proj_->lastIsOptional(true);
            helper.proj_->errorThreshold(1e-3);
            hppDout(notice,"before : "<<pinocchio::displayConfig(res));
            bool opSuccess = helper.proj_->optimize(res);
            hppDout(notice,"optimize successfull : "<<opSuccess);
            hppDout(notice,"after : "<<pinocchio::displayConfig(res));*/
        }
        else
        {
            success = false;
        }
        // copy extraDoF from original config :
        hppDout(notice,"projectOnCom end : ");
        hppDout(notice,"original config : "<<pinocchio::displayConfig(model.configuration_));
        hppDout(notice,"project  config : "<<pinocchio::displayConfig(res));
        res.segment((fullbody->device_->configSize() - fullbody->device_->extraConfigSpace().dimension()),fullbody->device_->extraConfigSpace().dimension()) = model.configuration_.tail(fullbody->device_->extraConfigSpace().dimension());
        hppDout(notice,"project  config : "<<pinocchio::displayConfig(res.head(res.rows()-1)));
        return res.head(res.rows()-1);
    }

    /*void generateOneComPath(const pinocchio::ConfigurationIn_t & from, const pinocchio::ConfigurationIn_t & to,
                       const pinocchio::ConfigurationIn_t & initSpeed, const double& acceleration)
    {

    }

    typedef std::vector<pinocchio::vector_t,Eigen::aligned_allocator<pinocchio::vector_t> > T_Configuration;
    core::PathPtr_t generateComPath(pinocchio::DevicePtr_t device, const T_Configuration& configurations, const std::vector<double>& accelerations,
                                    const pinocchio::value_type dt, const pinocchio::ConfigurationIn_t & initSpeed)
    {
        assert(configurations.size() == accelerations.size() +1);
        core::PathVectorPtr_t res = core::PathVector::create(device->configSize(), device->numberDof());
        pinocchio::value_type size_step = 1 /(pinocchio::value_type)(positions.size());
        pinocchio::value_type u = 0.;
        CIT_Configuration pit = positions.begin();
        pinocchio::Configuration_t previous = addRotation(pit, 0., q1, q2, ref), current;
        ++pit;
        for(;pit != positions.end()-1; ++pit, u+=size_step)
        {
            current = addRotation(pit, u, q1, q2, ref);
            res->appendPath(makePath(device,problem, previous, current));
            previous = current;
        }
        // last configuration is exactly q2
        current = addRotation(pit, 1., q1, q2, ref);
        res->appendPath(makePath(device,problem, previous, current));
        return res;

    }*/
  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp
