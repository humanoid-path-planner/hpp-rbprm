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
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/discretized-path-validation.hh>

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
    }

    core::PathPtr_t comRRT(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem, const PathPtr_t comPath,
                           const  State &startState, const State &nextState,
                           const  std::size_t numOptimizations,
                           const bool keepExtraDof)
    {
        //check whether there is a contact variations
        std::vector<std::string> variations = nextState.allVariations(startState, extractEffectorsName(fullbody->GetLimbs()));
        core::PathPtr_t guidePath;
        T_State states; states.push_back(startState); states.push_back(nextState);
        T_StateFrame stateFrames;
        stateFrames.push_back(std::make_pair(comPath->timeRange().first, startState));
        stateFrames.push_back(std::make_pair(comPath->timeRange().second, nextState));
        if(variations.empty())
        {
            std::vector<bool> cosntraintsR = setMaintainRotationConstraints();
            std::vector<std::string> fixed = nextState.fixedContacts(startState);
            model::DevicePtr_t device = fullbody->device_->clone();
            if(keepExtraDof)
            {
                device->setDimensionExtraConfigSpace(device->extraConfigSpace().dimension()+1);
            }
            core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(device,"proj", 1e-4, 1000);
            core::Problem rootProblem(device);
            for(std::vector<std::string>::const_iterator cit = fixed.begin();
                cit != fixed.end(); ++cit)
            {
                /*RbPrmLimbPtr_t limb = fullbody->GetLimbs().at(*cit);
                const fcl::Vec3f& ppos  = startState.contactPositions_.at(*cit);
                const fcl::Matrix3f& rotation = startState.contactRotation_.at(*cit);
                JointPtr_t effectorJoint = device->getJointByName(limb->effector_->name());
                proj->add(core::NumericalConstraint::create (
                                        constraints::deprecated::Position::create("",device,
                                                                      effectorJoint,fcl::Vec3f(0,0,0), ppos)));
                if(limb->contactType_ == hpp::rbprm::_6_DOF)
                {
                    proj->add(core::NumericalConstraint::create (constraints::deprecated::Orientation::create("", device,
                                                                                      effectorJoint,
                                                                                      rotation,
                                                                                      cosntraintsR)));
                }
                tools::RemoveNonLimbCollisionRec<core::Problem>(device->rootJoint(),
                                                                limb->limb_->name(),
                                                                rootProblem.collisionObstacles(),rootProblem);*/
            }
            rootProblem.configurationShooter(core::BasicConfigurationShooter::create(device));
            rootProblem.pathValidation(DiscretizedPathValidation::create(device,0.05));
            core::ConstraintSetPtr_t cSet = core::ConstraintSet::create(rootProblem.robot(),"");
            cSet->addConstraint(proj);
            rootProblem.constraints(cSet);
            ConfigurationPtr_t start =  ConfigurationPtr_t(new Configuration_t(startState.configuration_));
            ConfigurationPtr_t end =  ConfigurationPtr_t(new Configuration_t(nextState.configuration_));
            rootProblem.initConfig(start);
            BiRRTPlannerPtr_t planner = BiRRTPlanner::create(rootProblem);
            ProblemTargetPtr_t target = problemTarget::GoalConfigurations::create (planner);
            rootProblem.target (target);
            rootProblem.addGoalConfig(end);
            guidePath = planner->solve();
            return guidePath;
        }
        else
        {
            guidePath = limbRRT(fullbody,referenceProblem,states.begin(),states.begin()+1,numOptimizations);
        }
        ComRRTShooterFactory shooterFactory(guidePath);
        SetComRRTConstraints constraintFactory;
#ifdef PROFILE
    RbPrmProfiler& watch = getRbPrmProfiler();
    watch.start("com_traj");
#endif
        core::PathPtr_t resPath = interpolateStatesFromPath<ComRRTHelper, ComRRTShooterFactory, SetComRRTConstraints>
              (fullbody, referenceProblem, shooterFactory, constraintFactory, comPath, stateFrames.begin(), stateFrames.begin()+1, numOptimizations, keepExtraDof);
#ifdef PROFILE
    watch.stop("com_traj");
#endif
        return resPath;
    }

    core::PathPtr_t comRRTFromPath(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem, const PathPtr_t comPath,
                           const PathPtr_t guidePath, const CIT_StateFrame &startState, const CIT_StateFrame &endState,
                           const  std::size_t numOptimizations)
    {
        ComRRTShooterFactory shooterFactory(guidePath);
        SetComRRTConstraints constraintFactory;
        return interpolateStatesFromPath<ComRRTHelper, ComRRTShooterFactory, SetComRRTConstraints>
                (fullbody, referenceProblem, shooterFactory, constraintFactory, comPath, startState, endState, numOptimizations);
    }

    core::Configuration_t projectOnCom(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem, const State& model, const fcl::Vec3f& targetCom, bool &success)
    {
        core::PathPtr_t unusedPath(StraightPath::create(fullbody->device_,model.configuration_, model.configuration_,0));
        ComRRTShooterFactory unusedFactory(unusedPath);
        SetComRRTConstraints constraintFactory;
        ComRRTHelper helper(fullbody, unusedFactory, constraintFactory, referenceProblem,unusedPath );
        CreateContactConstraints<ComRRTHelper>(helper,model,model);
        CreateComConstraint<ComRRTHelper,core::PathPtr_t>(helper, helper.refPath_,targetCom);
        Configuration_t res(helper.fullBodyDevice_->configSize());
        res.head(model.configuration_.rows()) = model.configuration_;
        if(helper.proj_->apply(res))
        {
            success = true;
        }
        else
        {
            success = false;
        }
        return res.head(res.rows()-1);
    }
  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp
