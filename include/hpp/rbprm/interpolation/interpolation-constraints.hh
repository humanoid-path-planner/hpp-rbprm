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

#ifndef HPP_RBPRM_INTERPOLATION_CONSTRAINTS_HH
# define HPP_RBPRM_INTERPOLATION_CONSTRAINTS_HH

# include <hpp/rbprm/interpolation/time-dependant.hh>
# include <hpp/rbprm/interpolation/time-constraint-utils.hh>
# include <hpp/core/path.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/config-projector.hh>
# include <hpp/constraints/relative-com.hh>
# include <hpp/constraints/symbolic-calculus.hh>
# include <hpp/constraints/symbolic-function.hh>

namespace hpp {
namespace rbprm {
namespace interpolation {

    // declaration of available constraints

    template<class Helper_T>
    void CreateContactConstraints(const State& from, const State& to);

    template<class Helper_T, typename Reference>
    void CreateComConstraint(Helper_T& helper, const Reference& ref, const fcl::Vec3f& initTarget=fcl::Vec3f());

    template<class Helper_T, typename Reference>
    void CreateEffectorConstraint(Helper_T& helper, const Reference& ref,  const JointPtr_t effectorJoint, const fcl::Vec3f& initTarget=fcl::Vec3f());

    template<class Helper_T, typename Reference>
    void Create6DEffectorConstraint(Helper_T& helper, const Reference& ref,  const JointPtr_t effectorJoint, const fcl::Transform3f& initTarget=fcl::Transform3f());

    // Implementation

    template<class Reference>
    struct VecRightSide : public RightHandSideFunctor
    {
         VecRightSide (const Reference ref, const int dim = 3) : ref_(ref), dim_(dim){}
        ~VecRightSide(){}
        virtual void operator() (constraints::vectorOut_t output,
                               const constraints::value_type& normalized_input, model::ConfigurationOut_t /*conf*/) const
        {
          const std::pair<core::value_type, core::value_type>& tR (ref_->timeRange());
          constraints::value_type unNormalized = (tR.second-tR.first)* normalized_input + tR.first;
          output = ref_->operator ()(unNormalized).head(dim_);
        }
        const Reference ref_;
        const int dim_;
    };

    typedef constraints::PointCom PointCom;
    typedef constraints::SymbolicFunction<PointCom> PointComFunction;
    typedef constraints::SymbolicFunction<PointCom>::Ptr_t PointComFunctionPtr_t;

    template<class Helper_T, typename Reference>
    void CreateComConstraint(Helper_T& helper, const Reference &ref, const fcl::Vec3f& initTarget=fcl::Vec3f())
    {
        model::DevicePtr_t device = helper.rootProblem_.robot();
        core::ComparisonTypePtr_t equals = core::Equality::create ();
        core::ConfigProjectorPtr_t& proj = helper.proj_;
        model::CenterOfMassComputationPtr_t comComp = model::CenterOfMassComputation::
          create (device);
        comComp->add (device->rootJoint());
//comComp->add (device->getJointByName("romeo/base_joint_xyz"));
        comComp->computeMass ();
        PointComFunctionPtr_t comFunc = PointComFunction::create ("COM-walkgen",
            device, PointCom::create (comComp));
        NumericalConstraintPtr_t comEq = NumericalConstraint::create (comFunc, equals);
        comEq->nonConstRightHandSide() = initTarget;
        proj->add(comEq);
        proj->updateRightHandSide();
        helper.steeringMethod_->tds_.push_back(TimeDependant(comEq, boost::shared_ptr<VecRightSide<Reference> >(new VecRightSide<Reference> (ref, 3))));
    }

    inline constraints::PositionPtr_t createPositionMethod(model::DevicePtr_t device, const fcl::Vec3f& initTarget, JointPtr_t effector)
    {
        //std::vector<bool> mask; mask.push_back(false); mask.push_back(false); mask.push_back(true);
        std::vector<bool> mask; mask.push_back(true); mask.push_back(true); mask.push_back(true);
        fcl::Transform3f localFrame, globalFrame;
        globalFrame.setTranslation(initTarget);
        return constraints::Position::create("",device,
                                             effector,
                                             localFrame,
                                             globalFrame,
                                             mask);
    }

    inline constraints::OrientationPtr_t createOrientationMethod(model::DevicePtr_t device, const fcl::Transform3f& initTarget, JointPtr_t effector)
    {
        //std::vector<bool> mask; mask.push_back(false); mask.push_back(false); mask.push_back(true);
        std::vector<bool> mask; mask.push_back(true); mask.push_back(true); mask.push_back(true);
        const fcl::Matrix3f& rotation = initTarget.getRotation();
        return constraints::Orientation::create("", device,
                                                    effector,
                                                    rotation,
                                                    mask);
    }


    template<class Helper_T, typename Reference>
    void CreateEffectorConstraint(Helper_T& helper, const Reference &ref,  const JointPtr_t effectorJoint, const fcl::Vec3f& initTarget)
    {
        model::DevicePtr_t device = helper.rootProblem_.robot();
        core::ComparisonTypePtr_t equals = core::Equality::create ();
        core::ConfigProjectorPtr_t& proj = helper.proj_;
        JointPtr_t effector = device->getJointByName(effectorJoint->name());
        NumericalConstraintPtr_t effEq = core::NumericalConstraint::create (
                                    createPositionMethod(device,initTarget, effector), equals);
        effEq->nonConstRightHandSide()[0] = initTarget[2];
        proj->add(effEq);
        proj->updateRightHandSide();
        helper.steeringMethod_->tds_.push_back(
                    TimeDependant(effEq, boost::shared_ptr<VecRightSide<Reference> >(new VecRightSide<Reference>(ref, 3))));
    }

    template<typename Reference, typename fun>
    struct funEvaluator : public RightHandSideFunctor
    {
        funEvaluator(const Reference& ref, const fun& method) : ref_(ref), method_(method),
        dim_(method_->inputSize ()){}
        const std::pair<core::value_type, core::value_type> timeRange ()
        {
            return ref_->timeRange ();
        }
        void operator ()(constraints::vectorOut_t output,
                         const constraints::value_type& normalized_input, model::ConfigurationOut_t /*conf*/) const
        {
            const std::pair<core::value_type, core::value_type>& tR (ref_->timeRange());
            constraints::value_type unNormalized = (tR.second-tR.first)* normalized_input + tR.first;
            method_->operator ()(output, (ref_->operator ()(unNormalized)));
        }

        const Reference ref_;
        const fun method_;
        const std::size_t dim_;
    };

    template<class Helper_T, typename Reference>
    void Create6DEffectorConstraint(Helper_T& helper, const Reference &ref,  const JointPtr_t effectorJoint, const fcl::Transform3f& initTarget)
    {
        //CreateEffectorConstraint(helper, ref, effectorJoint, initTarget.getTranslation());
        model::DevicePtr_t device = helper.rootProblem_.robot();
        // reduce dof if reference path is of lower dimension
        if(ref->operator()(0).rows() < device->configSize())
        {
            device = device->clone();
            device->setDimensionExtraConfigSpace(device->extraConfigSpace().dimension()-1);
        }
        core::ComparisonTypePtr_t equals = core::Equality::create ();
        core::ConfigProjectorPtr_t& proj = helper.proj_;
        JointPtr_t effector = device->getJointByName(effectorJoint->name());
        constraints::OrientationPtr_t orCons = createOrientationMethod(device,initTarget, effector);
        NumericalConstraintPtr_t effEq = core::NumericalConstraint::create (orCons, equals);
        proj->add(effEq);
        proj->updateRightHandSide();
        boost::shared_ptr<funEvaluator<Reference, constraints::OrientationPtr_t> > orEv
                (new funEvaluator<Reference, constraints::OrientationPtr_t>(ref, orCons));
        helper.steeringMethod_->tds_.push_back(
                    TimeDependant(effEq, orEv));
    }


    template<class Helper_T>
    void CreateContactConstraints(Helper_T& helper, const State& from, const State& to)
    {
        std::vector<bool> cosntraintsR = setMaintainRotationConstraints();
        std::vector<std::string> fixed = to.fixedContacts(from);
        model::DevicePtr_t device = helper.rootProblem_.robot();

        for(std::vector<std::string>::const_iterator cit = fixed.begin();
            cit != fixed.end(); ++cit)
        {
            RbPrmLimbPtr_t limb = helper.fullbody_->GetLimbs().at(*cit);
            const fcl::Vec3f& ppos  = from.contactPositions_.at(*cit);
            const fcl::Matrix3f& rotation = from.contactRotation_.at(*cit);
            JointPtr_t effectorJoint = device->getJointByName(limb->effector_->name());
            helper.proj_->add(core::NumericalConstraint::create (
                                    constraints::deprecated::Position::create("",device,
                                                                  effectorJoint,fcl::Vec3f(0,0,0), ppos)));
            if(limb->contactType_ == hpp::rbprm::_6_DOF)
            {
                helper.proj_->add(core::NumericalConstraint::create (constraints::deprecated::Orientation::create("", device,
                                                                                  effectorJoint,
                                                                                  rotation,
                                                                                  cosntraintsR)));
            }
        }
    }


    } // namespace interpolation
  } // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_INTERPOLATION_CONSTRAINTS_HH
