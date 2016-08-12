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
#include <hpp/rbprm/interpolation/time-constraint-utils.hh>
#include <hpp/rbprm/tools.hh>
#include <hpp/core/bi-rrt-planner.hh>
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
          //const interval_t& tR (comPath_->timeRange());
          //constraints::value_type unNormalized = (tR.second-tR.first)* normalized_input + tR.first;
          output = comPath_->operator ()(normalized_input).head(3);
        }
        const core::PathPtr_t comPath_;
    };

    void CreateComConstraint(ComRRTHelper& helper)
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
        proj->add(comEq);
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

    void ComRRTHelper::SetConstraints(const State& from, const State& to)
    {
        CreateComConstraint(*this);
        SetContactConstraints(from,to);
    }
  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp
