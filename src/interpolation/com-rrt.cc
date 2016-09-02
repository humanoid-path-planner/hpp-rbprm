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
#include <hpp/rbprm/tools.hh>
#include <hpp/core/bi-rrt-planner.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/discretized-path-validation.hh>
#include <hpp/constraints/relative-com.hh>
#include <hpp/constraints/symbolic-calculus.hh>
#include <hpp/constraints/symbolic-function.hh>

namespace hpp {
using namespace core;
  namespace rbprm {
  namespace interpolation {  

    typedef constraints::PointCom PointCom;
    typedef constraints::SymbolicFunction<PointCom> PointComFunction;
    typedef constraints::SymbolicFunction<PointCom>::Ptr_t PointComFunctionPtr_t;



    struct ComRightSide : public RightHandSideFunctor
    {
         ComRightSide (const core::PathPtr_t comPath) : comPath_(comPath){}
        ~ComRightSide(){}
        virtual void operator() (constraints::vectorOut_t output,
                               const constraints::value_type& normalized_input, model::ConfigurationOut_t /*conf*/) const
        {
          const interval_t& tR (comPath_->timeRange());
          constraints::value_type unNormalized = (tR.second-tR.first)* normalized_input + tR.first;
          output = comPath_->operator ()(unNormalized).head(3);
        }
        const core::PathPtr_t comPath_;
    };

    void CreateComConstraint(ComRRTHelper& helper, const fcl::Vec3f& initTarget=fcl::Vec3f())
    {
        model::DevicePtr_t device = helper.rootProblem_.robot();
        core::ComparisonTypePtr_t equals = core::Equality::create ();
        core::ConfigProjectorPtr_t& proj = helper.proj_;
        /************/
        // Create the time varying equation for COM
        model::CenterOfMassComputationPtr_t comComp = model::CenterOfMassComputation::
          create (device);
        comComp->add (device->rootJoint());
        comComp->computeMass ();
        PointComFunctionPtr_t comFunc = PointComFunction::create ("COM-walkgen",
            device, PointCom::create (comComp));
        NumericalConstraintPtr_t comEq = NumericalConstraint::create (comFunc, equals);
        comEq->nonConstRightHandSide() = initTarget;
        proj->add(comEq);
        proj->updateRightHandSide();
        helper.steeringMethod_->tds_.push_back(TimeDependant(comEq, boost::shared_ptr<ComRightSide>(new ComRightSide(helper.refPath_))));
       /***************/
        //create identity joint
        //model::ObjectFactory ofac;
        //const model::Transform3f tr;
        //model::JointPtr_t j (ofac.createJointAnchor(tr));
        //constraints::RelativeComPtr_t cons = constraints::RelativeCom::create(device,j,fcl::Vec3f(0,0,0));
        //fcl::Transform3f localFrame, globalFrame;
        //constraints::PositionPtr_t cons (constraints::Position::create("",device,
        //                                                               device->rootJoint(),
        //                                                               globalFrame,
        //                                                               localFrame));
        //core::NumericalConstraintPtr_t nm(core::NumericalConstraint::create (cons,core::Equality::create()));
        //proj->add(nm);
        //helper.steeringMethod_->tds_.push_back(TimeDependant(nm,boost::shared_ptr<ComRightSide>(new ComRightSide(helper.refPath_,j))));
    }

    void SetComRRTConstraints::operator ()(ComRRTHelper& helper, const State& from, const State& to)
    {
        helper.SetContactConstraints(from, to);
        CreateComConstraint(helper);
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
        ComRRTShooterFactory factory(guidePath);
        return interpolateStatesFromPath<ComRRTHelper, ComRRTShooterFactory>
                (fullbody, referenceProblem, factory, comPath, stateFrames.begin(), stateFrames.begin()+1, numOptimizations, keepExtraDof);
    }

    core::PathPtr_t comRRTFromPath(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem, const PathPtr_t comPath,
                           const PathPtr_t guidePath, const CIT_StateFrame &startState, const CIT_StateFrame &endState,
                           const  std::size_t numOptimizations)
    {
        ComRRTShooterFactory factory(guidePath);
        return interpolateStatesFromPath<ComRRTHelper, ComRRTShooterFactory>
                (fullbody, referenceProblem, factory, comPath, startState, endState, numOptimizations);
    }

    core::Configuration_t projectOnCom(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem, const State& model, const fcl::Vec3f& targetCom)
    {
        core::PathPtr_t unusedPath(StraightPath::create(fullbody->device_,model.configuration_, model.configuration_,0));
        ComRRTShooterFactory unusedFactory(unusedPath);
        ComRRTHelper helper(fullbody, unusedFactory, referenceProblem,unusedPath );
        helper.SetContactConstraints(model,model);
        CreateComConstraint(helper,targetCom);
        Configuration_t res(helper.fullBodyDevice_->configSize());
        res.head(model.configuration_.rows()) = model.configuration_;
        if(helper.proj_->apply(res))
        {
            return res.head(res.rows()-1);
        }
        else
        {
            throw std::runtime_error("could not project state on COM constraint");
        }
    }
  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp
