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
# include <robust-equilibrium-lib/static_equilibrium.hh>

namespace hpp{
  namespace rbprm{

    using robust_equilibrium::Vector3;
    using robust_equilibrium::MatrixXX;

    SteeringMethodKinodynamic::SteeringMethodKinodynamic (const core::ProblemPtr_t& problem) :
      core::steeringMethod::Kinodynamic (problem),
      sEq_(new robust_equilibrium::StaticEquilibrium(problem_->robot()->name(), problem_->robot()->mass(),4,robust_equilibrium::SOLVER_LP_QPOASES,true,10,false)),
      device_ (problem->robot ()), weak_ ()
    {
    }

    /// Copy constructor
    SteeringMethodKinodynamic::SteeringMethodKinodynamic (const SteeringMethodKinodynamic& other) :
      core::steeringMethod::Kinodynamic (other),
      sEq_(new robust_equilibrium::StaticEquilibrium(problem_->robot()->name(), problem_->robot()->mass(),4,robust_equilibrium::SOLVER_LP_QPOASES,true,10,false)),
      device_ (other.device_)
    {
    }


    core::PathPtr_t SteeringMethodKinodynamic::impl_compute (core::ConfigurationIn_t q1,
                                         core::ConfigurationIn_t q2) const
    {
      return core::steeringMethod::Kinodynamic::impl_compute(q1,q2);
    }

    core::PathPtr_t SteeringMethodKinodynamic::impl_compute (core::NodePtr_t x,
                                         core::ConfigurationIn_t q2)
    {
      // get kinodynamic path from core::steeringMethod::Kinodynamic
      core::RbprmNodePtr_t node =  setSteeringMethodBounds(x,q2,false);
      //return core::steeringMethod::Kinodynamic::impl_compute(*x->configuration(),q2);
      core::PathPtr_t path = core::steeringMethod::Kinodynamic::impl_compute(*x->configuration(),q2);
      core::KinodynamicPathPtr_t kinoPath = boost::dynamic_pointer_cast<core::KinodynamicPath>(path);
      assert (path && "Error while casting path shared ptr"); // really usefull ? should never happen
      core::size_type configSize = problem_->robot()->configSize() - problem_->robot()->extraConfigSpace().dimension ();

      // check if acceleration is valid after each sign change :
      core::vector_t t1 = kinoPath->getT1();
      core::vector_t tv = kinoPath->getTv();
      double t=0;
      core::ConfigurationPtr_t q(new core::Configuration_t(problem_->robot()->configSize()));
      core::vector3_t a;
      bool aValid;
      double maxT = kinoPath->length();
      hppDout(info,"## start checking intermediate accelerations");
      t = 0.0001;
      (*kinoPath)(*q,t);
      hppDout(info,"q(t="<<t<<") = "<<model::displayConfig(*q));
      a = (*q).segment<3>(configSize+3);
      hppDout(info,"a = "<<a);
      aValid = sEq_->checkAdmissibleAcceleration(node->getG(),node->getH(),node->geth(),a);
      hppDout(info,"a valid : "<<aValid);
      if(!aValid && t < maxT)
        maxT = t;
      for(size_t ijoint = 0 ; ijoint < 3 ; ijoint++){
        if(t1[ijoint] > 0){
          hppDout(info,"for joint "<<ijoint);
          t = t1[ijoint] + 0.0001; // add an epsilon to get the value after the sign change
          (*kinoPath)(*q,t);
          hppDout(info,"q(t="<<t<<") = "<<model::displayConfig(*q));
          a = (*q).segment<3>(configSize+3);
          hppDout(info,"a = "<<a);
          aValid = sEq_->checkAdmissibleAcceleration(node->getG(),node->getH(),node->geth(),a);
          hppDout(info,"a valid : "<<aValid);
          if(!aValid && t < maxT)
            maxT = t;
        }
        if(tv[ijoint] > 0){
          t += tv[ijoint];
          (*kinoPath)(*q,t);
          hppDout(info,"q(t="<<t<<") = "<<model::displayConfig(*q));
          a = (*q).segment<3>(configSize+3);
          hppDout(info,"a = "<<a);
          aValid = sEq_->checkAdmissibleAcceleration(node->getG(),node->getH(),node->geth(),a);
          hppDout(info,"a valid : "<<aValid);
          if(!aValid && t < maxT)
            maxT = t;
        }
      }

      hppDout(info, "t = "<<kinoPath->length()<<" maxT = "<<maxT);
      if(maxT < kinoPath->length()){
        core::PathPtr_t extracted = kinoPath->extract(core::interval_t(kinoPath->timeRange().first,kinoPath->timeRange().first + maxT));
        hppDout(notice,"extracted path : end = "<<model::displayConfig((extracted->end())));
        return extracted;
      }
      return kinoPath;
    }

    // reverse (from q1 to x, but end() should always be x)
    core::PathPtr_t SteeringMethodKinodynamic::impl_compute (core::ConfigurationIn_t q1,core::NodePtr_t x)
    {
      core::RbprmNodePtr_t node =  setSteeringMethodBounds(x,q1,true);
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
      bool aValid;
      double minT = kinoPath->timeRange().second;
      hppDout(info,"## start checking intermediate accelerations");
      for(size_t ijoint = 0 ; ijoint < 3 ; ijoint++){
        hppDout(info,"for joint "<<ijoint);
        if(t1[ijoint] > 0){
          t = t1[ijoint] + 0.0001;
          (*kinoPath)(*q,t);
          hppDout(info,"q = "<<model::displayConfig(*q));
          a = (*q).segment<3>(configSize+3);
          hppDout(info,"a = "<<a);
          //TODO check a :
          aValid = sEq_->checkAdmissibleAcceleration(node->getG(),node->getH(),node->geth(),a);
          hppDout(info,"a valid : "<<aValid);
          if(aValid && t < minT)
            minT = t;
        }
        if(tv[ijoint] > 0){
          t += tv[ijoint];
          (*kinoPath)(*q,t);
          hppDout(info,"q = "<<model::displayConfig(*q));
          a = (*q).segment<3>(configSize+3);
          hppDout(info,"a = "<<a);
          //TODO check a :
          aValid = sEq_->checkAdmissibleAcceleration(node->getG(),node->getH(),node->geth(),a);
          hppDout(info,"a valid : "<<aValid);
          if(aValid && t < minT)
            minT = t;
        }
      }
      hppDout(info, "t = "<<kinoPath->length()<<" minT = "<<minT);
      if(minT == kinoPath->timeRange().second)
        return core::PathPtr_t();
      if(minT > kinoPath->timeRange().first)
        return kinoPath->extract(core::interval_t(minT,kinoPath->timeRange().second));
      return kinoPath;
    }

    core::RbprmNodePtr_t SteeringMethodKinodynamic::setSteeringMethodBounds(const core::NodePtr_t& near, const core::ConfigurationIn_t target,bool reverse) {
      //TODO compute the maximal acceleration from near, on a direction from near to target (oposite if revezrse == true)
      core::RbprmNodePtr_t node = static_cast<core::RbprmNodePtr_t>(near);
      assert(node && "Unable to cast near node to rbprmNode");
      const model::size_type indexECS =problem_->robot()->configSize() - problem_->robot()->extraConfigSpace().dimension (); // ecs index
      // compute direction (v) :

      double alpha0=1.; // main variable of our LP problem
      Vector3 toP,fromP,dPosition;
      Vector3 toV,fromV,dVelocity;
      Vector3 direction;

      if(reverse){
        toP = near->configuration()->head(3);
        fromP = target.head(3);
        toV = near->configuration()->segment<3>(indexECS);
        fromV = target.segment<3>(indexECS);
      }else{
        fromP = near->configuration()->head(3);
        toP = target.head(3);
        fromV = near->configuration()->segment<3>(indexECS);
        toV = target.segment<3>(indexECS);
      }
      dPosition = (toP - fromP);
     // dPosition.normalize();
      dVelocity = (toV - fromV);
    //  dVelocity.normalize();
      hppDout(info, "delta position  = "<<dPosition.transpose());
      hppDout(info, "delta velocity  = "<<dVelocity.transpose());
      direction = dPosition + dVelocity;
      direction.normalize();
      hppDout(info, "direction  = "<<direction.transpose());
      // define LP problem : with m+1 variables and 6 constraints
      int m = node->getNumberOfContacts() * 4;

      MatrixXX A = MatrixXX::Zero(6, m+1);
      // build A : [ -G (Hv)^T] :
      A.topLeftCorner(6,m) = - node->getG();
      MatrixXX Hv = (node->getH() * direction);
      assert(Hv.rows() == 6 && Hv.cols()==1 && "Hv should be a vector 6");
      A.topRightCorner(6,1) = Hv;
      hppDout(info,"H = \n"<<node->getH());
      hppDout(info," Hv^T = "<<Hv.transpose());
      hppDout(info,"A = \n"<<A);

      // call to robust_equilibrium_lib :
      sEq_->findMaximumAcceleration(A, node->geth(),alpha0);

      hppDout(info,"Amax found : "<<alpha0);
      if(alpha0 <= 0 )
        alpha0 = aMaxFixed_;
      else
        alpha0 = std::min(alpha0,aMaxFixed_);
      hppDout(info,"Amax after min : "<<alpha0);
      setAmax(alpha0*direction);
      hppDout(info,"Amax vector : "<<aMax_.transpose());
      //setVmax(2*Vector3::Ones(3)); //FIXME: read it from somewhere ?
      return node;
    }

  }//rbprm
}//hpp
