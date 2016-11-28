// Copyright (c) 2016, LAAS-CNRS
// Authors: Pierre Fernbach (pierre.fernbach@laas.fr)
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

# include <hpp/rbprm/planner/rbprm-steering-kinodynamic.hh>
# include <hpp/model/device.hh>
# include <hpp/model/joint.hh>
# include <hpp/model/configuration.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/weighed-distance.hh>
# include <hpp/core/kinodynamic-path.hh>
# include <hpp/rbprm/planner/rbprm-node.hh>

namespace hpp{
  namespace rbprm{


    SteeringMethodKinodynamic::SteeringMethodKinodynamic (const core::ProblemPtr_t& problem) :
      core::steeringMethod::Kinodynamic (problem), device_ (problem->robot ()), weak_ ()
    {
    }

    /// Copy constructor
    SteeringMethodKinodynamic::SteeringMethodKinodynamic (const SteeringMethodKinodynamic& other) :
      core::steeringMethod::Kinodynamic (other), device_ (other.device_)
    {
    }


    core::PathPtr_t SteeringMethodKinodynamic::impl_compute (core::ConfigurationIn_t q1,
                                         core::ConfigurationIn_t q2) const
    {
      return core::steeringMethod::Kinodynamic::impl_compute(q1,q2);
    }

    core::PathPtr_t SteeringMethodKinodynamic::impl_compute (core::NodePtr_t x,
                                         core::ConfigurationIn_t q2) const
    {
      // get kinodynamic path from core::steeringMethod::Kinodynamic
      core::RbprmNodePtr_t node =  static_cast<core::RbprmNodePtr_t>(x);
      assert (node && "Rbprm-steering-kinodynamic impl_compute(node,configuration) should be called with a rbprmNode only");
      core::PathPtr_t path = core::steeringMethod::Kinodynamic::impl_compute(*x->configuration(),q2);
      core::KinodynamicPathPtr_t kinoPath = boost::dynamic_pointer_cast<core::KinodynamicPath>(path);
      assert (path && "Error while casting path shared ptr"); // really usefull ? should never happen
      core::size_type configSize = problem_->robot()->configSize() - problem_->robot()->extraConfigSpace().dimension ();

      // check if acceleration is valid after each sign change :
      core::vector_t t1 = kinoPath->getT1();
      core::vector_t tv = kinoPath->getTv();
      core::vector_t t2 = kinoPath->getT2();
      double t=0;
      core::ConfigurationPtr_t q(new core::Configuration_t(problem_->robot()->configSize()));
      core::vector3_t a;
      hppDout(info,"## start checking intermediate accelerations");
      for(size_t ijoint = 0 ; ijoint < 3 ; ijoint++){
        hppDout(info,"for joint "<<ijoint);
        t = t1[ijoint];
        (*kinoPath)(*q,t);
        hppDout(info,"q = "<<model::displayConfig(*q));
        a = (*q).segment<3>(configSize+3);
        hppDout(info,"a = "<<a);
        //TODO check a
        if(tv[ijoint] > 0){
          t += tv[ijoint];
          (*kinoPath)(*q,t);
          hppDout(info,"q = "<<model::displayConfig(*q));
          a = (*q).segment<3>(configSize+3);
          hppDout(info,"a = "<<a);
          //TODO check a
        }
        t += t2[ijoint];
        (*kinoPath)(*q,t);
        hppDout(info,"q = "<<model::displayConfig(*q));
        a = (*q).segment<3>(configSize+3);
        hppDout(info,"a = "<<a);
        //TODO check a
      }

    }

    core::PathPtr_t SteeringMethodKinodynamic::impl_compute (core::ConfigurationIn_t q1,core::NodePtr_t x) const
    {
      // get kinodynamic path from core::steeringMethod::Kinodynamic
      core::RbprmNodePtr_t node =  static_cast<core::RbprmNodePtr_t>(x);
      assert (node && "Rbprm-steering-kinodynamic impl_compute(node,configuration) should be called with a rbprmNode only");
      core::PathPtr_t path = core::steeringMethod::Kinodynamic::impl_compute(q1,*x->configuration());
      core::KinodynamicPathPtr_t kinoPath = boost::dynamic_pointer_cast<core::KinodynamicPath>(path);
      assert (path && "Error while casting path shared ptr"); // really usefull ? should never happen
      core::size_type configSize = problem_->robot()->configSize() - problem_->robot()->extraConfigSpace().dimension ();

      // check if acceleration is valid after each sign change :
      core::vector_t t1 = kinoPath->getT1();
      core::vector_t tv = kinoPath->getTv();
      core::vector_t t2 = kinoPath->getT2();
      double t=0;
      core::ConfigurationPtr_t q(new core::Configuration_t(problem_->robot()->configSize()));
      core::vector3_t a;
      hppDout(info,"## start checking intermediate accelerations");
      for(size_t ijoint = 0 ; ijoint < 3 ; ijoint++){
        hppDout(info,"for joint "<<ijoint);
        t = t1[ijoint];
        (*kinoPath)(*q,t);
        hppDout(info,"q = "<<model::displayConfig(*q));
        a = (*q).segment<3>(configSize+3);
        hppDout(info,"a = "<<a);
        //TODO check a
        if(tv[ijoint] > 0){
          t += tv[ijoint];
          (*kinoPath)(*q,t);
          hppDout(info,"q = "<<model::displayConfig(*q));
          a = (*q).segment<3>(configSize+3);
          hppDout(info,"a = "<<a);
          //TODO check a
        }
        t += t2[ijoint];
        (*kinoPath)(*q,t);
        hppDout(info,"q = "<<model::displayConfig(*q));
        a = (*q).segment<3>(configSize+3);
        hppDout(info,"a = "<<a);
        //TODO check a
      }

      return kinoPath;

    }

  }//rbprm
}//hpp
