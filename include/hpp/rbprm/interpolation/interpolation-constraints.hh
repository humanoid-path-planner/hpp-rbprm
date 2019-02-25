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
# include <hpp/pinocchio/configuration.hh>
# include <hpp/pinocchio/frame.hh>
# include <pinocchio/multibody/frame.hpp>
namespace hpp {
namespace rbprm {
namespace interpolation {

    // declaration of available constraints

    template<class Helper_T>
    void CreateContactConstraints(const State& from, const State& to);

    //template<class Helper_T, typename Reference>
    //void CreateComConstraint(Helper_T& helper, const Reference& ref, const fcl::Vec3f& initTarget=fcl::Vec3f());

    template<class Helper_T, typename Reference>
    void CreateEffectorConstraint(Helper_T& helper, const Reference& ref,  const pinocchio::Frame effectorJoint, const fcl::Vec3f& initTarget=fcl::Vec3f());

    template<class Helper_T, typename Reference>
    void Create6DEffectorConstraint(Helper_T& helper, const Reference& ref,  const pinocchio::Frame effectorJoint, const fcl::Transform3f& initTarget=fcl::Transform3f());

    template<class Helper_T, typename Reference>
    void CreateOrientationConstraint(Helper_T& helper, const Reference &ref,  const pinocchio::Frame effector, const pinocchio::DevicePtr_t endEffectorDevice, const fcl::Transform3f& initTarget=fcl::Transform3f());

    // Implementation

    template<class Reference>
    struct VecRightSide : public RightHandSideFunctor
    {
         VecRightSide (const Reference ref, const int dim = 3, const bool times_ten = false) : ref_(ref), dim_(dim), times_ten_(times_ten)
         {}
        ~VecRightSide(){}
        virtual void operator() (constraints::vectorOut_t output,
                               const constraints::value_type& normalized_input, pinocchio::ConfigurationOut_t /*conf*/) const
        {
          const std::pair<core::value_type, core::value_type>& tR (ref_->timeRange());
          constraints::value_type unNormalized = (tR.second-tR.first)* normalized_input + tR.first;
          bool success;
          if(times_ten_)
          {
              output = ref_->operator ()(unNormalized,success).head(dim_); // * (10000) ;
              assert(success && "path operator () did not succeed");
          }
          else
          {
            output = ref_->operator ()(unNormalized,success).head(dim_) ;
            assert(success && "path operator () did not succeed");
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
        pinocchio::DevicePtr_t device = helper.rootProblem_->robot();
        //constraints::ComparisonType equals = constraints::Equality;
        core::ConfigProjectorPtr_t& proj = helper.proj_;
        pinocchio::CenterOfMassComputationPtr_t comComp = pinocchio::CenterOfMassComputation::
          create (device);
        comComp->add (device->rootJoint());
        comComp->compute ();
        PointComFunctionPtr_t comFunc = PointComFunction::create ("COM-constraint",
            device, /*10000 **/ PointCom::create (comComp));
        constraints::ComparisonTypes_t equals (3, constraints::Equality);
        constraints::ImplicitPtr_t comEq = constraints::Implicit::create(comFunc, equals);
        proj->add(comEq);
        proj->rightHandSide(comEq,initTarget);
        helper.steeringMethod_->tds_.push_back(TimeDependant(comEq, boost::shared_ptr<VecRightSide<Reference> >(new VecRightSide<Reference> (ref, 3, true))));
    }

    template<class Helper_T, typename Reference>
    void CreatePosturalTaskConstraint(Helper_T& helper, const Reference &ref){
      pinocchio::DevicePtr_t device = helper.rootProblem_->robot();
      //core::ComparisonTypePtr_t equals = core::Equality::create ();
      core::ConfigProjectorPtr_t& proj = helper.proj_;
      hppDout(notice,"create postural task, ref config = "<<pinocchio::displayConfig(ref));
      std::vector <bool> mask (device->numberDof(),false);
      Configuration_t weight(device->numberDof());

      // mask : 0 for the freeflyer and the extraDoFs :
      for(size_t i = 0 ; i < 3 ; i++){
        mask[i]=false;
        weight[i] = 0.;
      }
      for(size_type i = device->numberDof()-7 ; i < device->numberDof() ; i++){
        mask[i]=false;
        weight[i] = 0.;
      }


      std::ostringstream oss;
      for (std::size_t i=0; i < mask.size (); ++i){
        oss << mask [i] << ",";
      }
      hppDout(notice,"mask = "<<oss.str());

      // create a weight vector
      for(size_type i = 3 ; i < device->numberDof()-7 ; ++i){
        weight[i] = 1.;
      }
      // TODO : retrieve it from somewhere, store it in fullbody ?
      // value here for hrp2, from Justin
      // : chest
      weight[6]= 10.;
      //head :
      for(size_type i = 7 ; i <= 9 ; i++)
        weight[i] = 1.;

      // root's orientation
      for(size_type i = 3 ; i < 6 ; i++){
        weight[i] = 50.;
      }

      for(size_t i = 3 ; i <= 9 ; i++){
        mask[i] = true;
      }
    //  mask[5] = false; // z root rotation ????




      // normalize weight array :
      double moy =0;
      int num_active = 0;
      for (size_type i = 0 ; i < weight.size() ; i++){
        if(mask[i]){
          moy += weight[i];
          num_active++;
        }
      }
      moy = moy/num_active;
      for (size_type i = 0 ; i < weight.size() ; i++)
        weight[i] = weight[i]/moy;


      constraints::ConfigurationConstraintPtr_t postFunc = constraints::ConfigurationConstraint::create("Postural_Task",device,ref,weight);
      ComparisonTypes_t comps; comps.push_back(constraints::Equality);
      const constraints::ImplicitPtr_t posturalTask = constraints::Implicit::create (postFunc, comps);
      proj->add(posturalTask,segments_t(0),1);
      //proj->updateRightHandSide();
    }

    inline constraints::PositionPtr_t createPositionMethod(pinocchio::DevicePtr_t device, const fcl::Vec3f& initTarget, const pinocchio::Frame effectorFrame)
    {
        //std::vector<bool> mask; mask.push_back(false); mask.push_back(false); mask.push_back(true);
        std::vector<bool> mask; mask.push_back(true); mask.push_back(true); mask.push_back(true);
        pinocchio::Transform3f localFrame, globalFrame;
        localFrame = localFrame.Identity(); globalFrame = globalFrame.Identity();
        globalFrame.translation(initTarget);
        pinocchio::JointPtr_t effectorJoint = effectorFrame.joint();
        return constraints::Position::create("",device,
                                             effectorJoint,
                                             effectorFrame.positionInParentFrame() * localFrame,
                                             globalFrame,
                                             mask);
    }

    inline constraints::OrientationPtr_t createOrientationMethod(pinocchio::DevicePtr_t device, const fcl::Transform3f& initTarget, const pinocchio::Frame effectorFrame)
    {
        //std::vector<bool> mask; mask.push_back(false); mask.push_back(false); mask.push_back(true);
        std::vector<bool> mask; mask.push_back(true); mask.push_back(true); mask.push_back(true);
        pinocchio::JointPtr_t effectorJoint = effectorFrame.joint();
        pinocchio::Transform3f rotation(1);
        rotation.rotation(effectorFrame.positionInParentFrame().rotation()*initTarget.getRotation());
        return constraints::Orientation::create("", device,
                                                    effectorJoint,
                                                    rotation,
                                                    mask);
    }


    template<class Helper_T, typename Reference>
    void CreateEffectorConstraint(Helper_T& helper, const Reference &ref,  const pinocchio::Frame effectorFr, const fcl::Vec3f& initTarget)
    {
        pinocchio::DevicePtr_t device = helper.rootProblem_->robot();
        ComparisonTypes_t comps; comps.push_back(constraints::Equality);
        core::ConfigProjectorPtr_t& proj = helper.proj_;

        pinocchio::Frame effectorFrame = device->getFrameByName(effectorFr.name());
        constraints::ImplicitPtr_t effEq = constraints::Implicit::create (
                                    createPositionMethod(device,initTarget, effectorFrame), comps);
        effEq->nonConstRightHandSide()[0] = initTarget[2];
        proj->add(effEq);
        proj->rightHandSide(effEq->nonConstRightHandSide());
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
                         const constraints::value_type& normalized_input, pinocchio::ConfigurationOut_t /*conf*/) const
        {
            const std::pair<core::value_type, core::value_type>& tR (ref_->timeRange());
            bool success;
            constraints::value_type unNormalized = (tR.second-tR.first)* normalized_input + tR.first;
            output = method_->operator ()((ref_->operator ()(unNormalized,success))).vector();
            assert(success && "path operator () did not succeed");
        }

        const Reference ref_;
        const fun method_;
        const std::size_t dim_;
    };


    template<class Helper_T, typename Reference>
    void CreateOrientationConstraint(Helper_T& helper, const Reference &ref,  const pinocchio::Frame effectorFr,const pinocchio::DevicePtr_t endEffectorDevice, const fcl::Transform3f& initTarget){
        pinocchio::DevicePtr_t device = helper.rootProblem_->robot();
        ComparisonTypes_t equals; equals.push_back(constraints::Equality);
        core::ConfigProjectorPtr_t& proj = helper.proj_;
        pinocchio::Frame effectorFrame = device->getFrameByName(effectorFr.name());
        constraints::OrientationPtr_t orCons = createOrientationMethod(device,initTarget, effectorFrame);
        constraints::OrientationPtr_t orConsRef = createOrientationMethod(endEffectorDevice,initTarget, endEffectorDevice->getFrameByName(endEffectorDevice->rootJoint()->childJoint(0)->name())); // same orientation constraint but for a freeflyer device that represent the end effector (same dim as the ref path)
        constraints::ImplicitPtr_t effEq = constraints::Implicit::create (orCons, equals);
        proj->add(effEq);
        //proj->updateRightHandSide();
        boost::shared_ptr<funEvaluator<Reference, constraints::OrientationPtr_t> > orEv
                (new funEvaluator<Reference, constraints::OrientationPtr_t>(ref, orConsRef));
        helper.steeringMethod_->tds_.push_back(
                    TimeDependant(effEq, orEv));
    }


    template<class Helper_T, typename Reference>
    void Create6DEffectorConstraint(Helper_T& helper, const Reference &ref,  const pinocchio::Frame effectorJoint, const fcl::Transform3f& initTarget)
    {
        //CreateEffectorConstraint(helper, ref, effectorJoint, initTarget.getTranslation());
        pinocchio::DevicePtr_t device = helper.rootProblem_->robot();
        // reduce dof if reference path is of lower dimension
        bool success;
        if(ref->operator()(0,success).rows() < device->configSize())
        {
            device = device->clone();
            device->setDimensionExtraConfigSpace(device->extraConfigSpace().dimension()-1);
        }
        ComparisonTypes_t equals; equals.push_back(constraints::Equality);
        core::ConfigProjectorPtr_t& proj = helper.proj_;
        pinocchio::Frame effector = device->getFrameByName(effectorJoint.name());
        constraints::OrientationPtr_t orCons = createOrientationMethod(device,initTarget, effector);
        constraints::ImplicitPtr_t effEq = constraints::Implicit::create (orCons, equals);
        proj->add(effEq);
        //proj->updateRightHandSide();
        boost::shared_ptr<funEvaluator<Reference, constraints::OrientationPtr_t> > orEv
                (new funEvaluator<Reference, constraints::OrientationPtr_t>(ref, orCons));
        helper.steeringMethod_->tds_.push_back(
                    TimeDependant(effEq, orEv));
    }


    void addContactConstraints(rbprm::RbPrmFullBodyPtr_t fullBody, pinocchio::DevicePtr_t device, core::ConfigProjectorPtr_t projector, const State& state, const std::vector<std::string> active);

    template<class Helper_T>
    void CreateContactConstraints(Helper_T& helper, const State& from, const State& to)
    {
        pinocchio::DevicePtr_t device = helper.rootProblem_->robot();
        std::vector<std::string> fixed = to.fixedContacts(from);
        addContactConstraints(helper.fullbody_, device, helper.proj_,from,fixed);
    }


    } // namespace interpolation
  } // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_INTERPOLATION_CONSTRAINTS_HH
