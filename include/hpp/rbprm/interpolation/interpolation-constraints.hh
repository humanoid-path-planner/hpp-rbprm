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
# include <hpp/constraints/configuration-constraint.hh>
# include <hpp/model/configuration.hh>
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
         VecRightSide (const Reference ref, const int dim = 3, const bool times_ten = false) : ref_(ref), dim_(dim), times_ten_(times_ten)
         {}
        ~VecRightSide(){}
        virtual void operator() (constraints::vectorOut_t output,
                               const constraints::value_type& normalized_input, model::ConfigurationOut_t /*conf*/) const
        {
          const std::pair<core::value_type, core::value_type>& tR (ref_->timeRange());
          constraints::value_type unNormalized = (tR.second-tR.first)* normalized_input + tR.first;
          if(times_ten_)
          {
              output = ref_->operator ()(unNormalized).head(dim_); // * (10000) ;
          }
          else
          {
            output = ref_->operator ()(unNormalized).head(dim_) ;
          }
        }
        const Reference ref_;
        const int dim_;
        const bool times_ten_;
    };

    typedef constraints::PointCom PointCom;
    typedef constraints::CalculusBaseAbstract<PointCom::ValueType_t, PointCom::JacobianType_t> s_t;
    typedef constraints::SymbolicFunction<s_t> PointComFunction;
    typedef constraints::SymbolicFunction<s_t>::Ptr_t PointComFunctionPtr_t;

    template<class Helper_T, typename Reference>
    void CreateComConstraint(Helper_T& helper, const Reference &ref, const fcl::Vec3f& initTarget=fcl::Vec3f())
    {
        model::DevicePtr_t device = helper.rootProblem_.robot();
        core::ComparisonTypePtr_t equals = core::Equality::create ();
        core::ConfigProjectorPtr_t& proj = helper.proj_;
        model::CenterOfMassComputationPtr_t comComp = model::CenterOfMassComputation::
          create (device);
        comComp->add (device->rootJoint());
        comComp->computeMass ();
        PointComFunctionPtr_t comFunc = PointComFunction::create ("COM-walkgen",
            device, /*10000 **/ PointCom::create (comComp));
        NumericalConstraintPtr_t comEq = NumericalConstraint::create (comFunc, equals);
        comEq->nonConstRightHandSide() = initTarget; // * 10000;
        proj->add(comEq);
        proj->updateRightHandSide();
        helper.steeringMethod_->tds_.push_back(TimeDependant(comEq, boost::shared_ptr<VecRightSide<Reference> >(new VecRightSide<Reference> (ref, 3, true))));
    }

    template<class Helper_T, typename Reference>
    void CreatePosturalTaskConstraint(Helper_T& helper, const Reference &ref){
      model::DevicePtr_t device = helper.rootProblem_.robot();
      core::ComparisonTypePtr_t equals = core::Equality::create ();
      core::ConfigProjectorPtr_t& proj = helper.proj_;
      hppDout(notice,"create postural task, ref config = "<<model::displayConfig(*ref));
      std::vector <bool> mask (device->configSize(),true);
      // mask : 0 for the freeflyer and the extraDoFs :
      for(size_t i = 0 ; i < 7 ; i++)
        mask[i]=false;
      for(size_t i = device->configSize()-1 ; i >= (device->configSize() - device->extraConfigSpace().dimension()) ; i-- )
        mask[i]=false;

      std::ostringstream oss;
      for (size_type i=0; i < mask.size (); ++i) {
        oss << mask [i] << ",";
      }

      hppDout(notice,"mask = "<<oss.str());
      constraints::ConfigurationConstraintPtr_t postFunc = constraints::ConfigurationConstraint::create("Postural_Task",device,*ref,mask);
      NumericalConstraintPtr_t posturalTask = NumericalConstraint::create (postFunc, equals);
      proj->add(posturalTask,SizeIntervals_t (0),1);
      proj->updateRightHandSide();
    //  helper.steeringMethod_->tds_.push_back(TimeDependant(posturalTask, boost::shared_ptr<VecRightSide<Reference> >(new VecRightSide<Reference> (ref, 3, true))));
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


    void addContactConstraints(rbprm::RbPrmFullBodyPtr_t fullBody, model::DevicePtr_t device, core::ConfigProjectorPtr_t projector, const State& state, const std::vector<std::string> active);

    template<class Helper_T>
    void CreateContactConstraints(Helper_T& helper, const State& from, const State& to)
    {
        model::DevicePtr_t device = helper.rootProblem_.robot();
        std::vector<std::string> fixed = to.fixedContacts(from);
        addContactConstraints(helper.fullbody_, device, helper.proj_,from,fixed);
    }


    } // namespace interpolation
  } // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_INTERPOLATION_CONSTRAINTS_HH
