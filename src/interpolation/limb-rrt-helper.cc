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

#include <hpp/rbprm/interpolation/limb-rrt-helper.hh>
#include <hpp/rbprm/interpolation/limb-rrt-shooter.hh>
#include <hpp/rbprm/interpolation/limb-rrt-path-validation.hh>
#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/problem-target/goal-configurations.hh>
#include <hpp/core/bi-rrt-planner.hh>
#include <hpp/model/joint.hh>

namespace hpp {
using namespace core;
using namespace model;
    namespace rbprm {
    namespace interpolation {

    namespace{
    core::DevicePtr_t DeviceFromLimb(const std::string& name, RbPrmLimbPtr_t limb)
    {
        DevicePtr_t limbDevice = Device::create(name);
        limbDevice->rootJoint(limb->limb_->clone());
        JointPtr_t current = limb->limb_, clone = limbDevice->rootJoint();
        while(current->name() != limb->effector_->name())
        {
            current = current->childJoint(0);
            clone->addChildJoint(current->clone());
            clone = clone->childJoint(0);
        }
        limbDevice->setDimensionExtraConfigSpace(1);
        return limbDevice;
    }
    /*T_LimbDevice DevicesFromLimbs(RbPrmFullBodyPtr_t fullbody)
    {
        T_LimbDevice devices;
        for(T_Limb::const_iterator cit = fullbody->GetLimbs().begin();
            cit != fullbody->GetLimbs().end(); ++cit)
        {
            devices.insert(std::make_pair(cit->first,
                                          DeviceFromLimb(cit->first, cit->second)));
        }
        //first create path between position
        return devices;
    }*/
    }

    //find contact creation

    LimbRRTHelper::LimbRRTHelper(RbPrmFullBodyPtr_t fullbody, hpp::core::ProblemPtr_t referenceProblem)
        : fullbody_(fullbody)
        , fullBodyDevice_(fullbody->device_->clone())
        , rootProblem_(fullBodyDevice_)
    {
        fullBodyDevice_->setDimensionExtraConfigSpace(fullBodyDevice_->extraConfigSpace().dimension()+1);
        rootProblem_.collisionObstacles(referenceProblem->collisionObstacles());
        // remove end effector collision
        for(ObjectVector_t::const_iterator cit = rootProblem_.collisionObstacles().begin();
            cit != rootProblem_.collisionObstacles().end(); ++cit)
        {
            for(T_Limb::const_iterator lit = fullbody->GetLimbs().begin(); lit !=
                fullbody->GetLimbs().end(); ++lit)
            {
                rootProblem_.removeObstacleFromJoint(
                            fullBodyDevice_->getJointByName(lit->second->effector_->name()), *cit);
            }
        }
        rootProblem_.pathValidation(LimbRRTPathValidation::create(fullBodyDevice_, 0.05,fullBodyDevice_->configSize()-1));
        /*for(hpp::rbprm::T_Limb::const_iterator cit = fullbody->GetLimbs().begin();
            cit != fullbody->GetLimbs().end(); ++cit)
        {
            //create limb device
            core::DevicePtr_t limbDevice = DeviceFromLimb(cit->first, cit->second);
            limbDevices_.push_back(limbDevice);
            //create associated problem solver
            problems_.push_back(Problem(limbDevice));
            Problem& problem = problems_.back();
            problem.collisionObstacles(referenceProblem->collisionObstacles());

        }*/
        //problem_.configurationShooter(LimbRRTShooter::create(fullbody,path,));
    }

    LimbRRTSolver::LimbRRTSolver(core::PathPtr_t rootPath, core::DevicePtr_t limbDevice,
                  const core::Problem& problem)
        : limbDevice_(limbDevice->clone())
        , rootPath_(rootPath->copy ())
        , problem_(problem)
    {
        // NOTHING
    }

    namespace
    {
    core::PathPtr_t generateRootPath(const LimbRRTHelper& helper, const State& from, const State& to)
    {
        Configuration_t startRootConf(from.configuration_);
        Configuration_t endRootConf(to.configuration_);
        return (*(helper.rootProblem_.steeringMethod()))(startRootConf, endRootConf);
    }
    }

    PathVectorPtr_t interpolateStates(LimbRRTHelper& helper, const State& from, const State& to)
    {
        PathVectorPtr_t res;
        core::PathPtr_t rootPath = generateRootPath(helper,from,to);
        // get limbs that moved
        std::vector<std::string> variations = to.variations(from);
        const rbprm::T_Limb& limbs = helper.fullbody_->GetLimbs();
        for(std::vector<std::string>::const_iterator cit = variations.begin();
            cit != variations.end(); ++cit)
        {
            ConfigurationShooterPtr_t limbRRTShooter = LimbRRTShooter::create(limbs.at(*cit),
                                                                                rootPath,
                                                                                helper.fullBodyDevice_->configSize()-1);
            helper.rootProblem_.configurationShooter(limbRRTShooter);
            Configuration_t start(helper.fullBodyDevice_->currentConfiguration());
            start.head(from.configuration_.rows()) = from.configuration_;
            start[start.rows()-1] = rootPath->timeRange().first;
            Configuration_t end  (start);
            end.head(from.configuration_.rows()) = to.configuration_;
            end[start.rows()-1] = rootPath->timeRange().second;
            helper.rootProblem_.initConfig(ConfigurationPtr_t(new Configuration_t(start)));
            BiRRTPlannerPtr_t planner = BiRRTPlanner::create(helper.rootProblem_);
            ProblemTargetPtr_t target = problemTarget::GoalConfigurations::create (planner);
            helper.rootProblem_.target (target);
            helper.rootProblem_.addGoalConfig(ConfigurationPtr_t(new Configuration_t(end)));
            res = planner->solve();
            helper.rootProblem_.resetGoalConfigs();
            break;
        }
        return res;
    }
  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp
