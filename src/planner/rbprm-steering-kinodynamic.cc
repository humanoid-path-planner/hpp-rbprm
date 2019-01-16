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
# include <hpp/pinocchio/device.hh>
# include <hpp/core/problem.hh>
# include <hpp/core/weighed-distance.hh>
# include <hpp/core/kinodynamic-path.hh>
# include <hpp/rbprm/planner/rbprm-node.hh>
#include <centroidal-dynamics-lib/centroidal_dynamics.hh>
# include <hpp/util/debug.hh>
# include <hpp/util/timer.hh>

#define ignore_acc_bound 0 // testing and debug only

namespace hpp{
  namespace rbprm{

    using centroidal_dynamics::Vector3;
    using centroidal_dynamics::MatrixXX;

    SteeringMethodKinodynamic::SteeringMethodKinodynamic (const core::Problem& problem) :
      core::steeringMethod::Kinodynamic (problem),
      totalTimeComputed_(0),totalTimeValidated_(0),dirValid_(0),dirTotal_(0),rejectedPath_(0),maxLength_(50),device_ (problem.robot ()),lastDirection_(),
       sEq_(new centroidal_dynamics::Equilibrium(problem_.robot()->name(), problem_.robot()->mass(),4,centroidal_dynamics::SOLVER_LP_QPOASES,true,10,false)),boundsUpToDate_(false), weak_ ()
    {
    }

    /// Copy constructor
    SteeringMethodKinodynamic::SteeringMethodKinodynamic (const SteeringMethodKinodynamic& other) :
      core::steeringMethod::Kinodynamic (other),
      totalTimeComputed_(0),totalTimeValidated_(0),dirValid_(0),dirTotal_(0),rejectedPath_(0),maxLength_(50),device_ (other.device_),lastDirection_(),
      sEq_(new centroidal_dynamics::Equilibrium(problem_.robot()->name(), problem_.robot()->mass(),4,centroidal_dynamics::SOLVER_LP_QPOASES,true,10,false)),boundsUpToDate_(false),weak_ ()
    {
    }


    core::PathPtr_t SteeringMethodKinodynamic::impl_compute (core::ConfigurationIn_t q1,
                                         core::ConfigurationIn_t q2) const
    {
      hppDout(notice,"Old prototype called !!!");
      std::cout<<"derecated prototype of steering method called"<<std::endl;
      return core::steeringMethod::Kinodynamic::impl_compute(q1,q2);
    }

    core::PathPtr_t SteeringMethodKinodynamic::impl_compute (core::NodePtr_t x,
                                         core::ConfigurationIn_t q2)
    {
      core::RbprmNodePtr_t node = static_cast<core::RbprmNodePtr_t>(x);
      assert(node && "Unable to cast near node to rbprmNode");
      if(!node)
        return core::PathPtr_t();
      // get kinodynamic path from core::steeringMethod::Kinodynamic
      hppStartBenchmark(FIND_A_MAX);
      core::PathPtr_t unboundedPath =  setSteeringMethodBounds(node,q2,false);
      hppDout(notice,"end setBounds");
      hppStopBenchmark(FIND_A_MAX);
      hppDisplayBenchmark(FIND_A_MAX);
      if((std::fabs(aMax_[0])+std::fabs(aMax_[1])) <= 0)
        return core::PathPtr_t();
      if(boundsUpToDate_)
          return unboundedPath;
      //return core::steeringMethod::Kinodynamic::impl_compute(*x->configuration(),q2);
      hppStartBenchmark(steering_kino);
      core::PathPtr_t path = core::steeringMethod::Kinodynamic::impl_compute(*x->configuration(),q2);
      hppStopBenchmark(steering_kino);
      hppDisplayBenchmark(steering_kino);
      if(!path)
        return core::PathPtr_t();
      core::KinodynamicPathPtr_t kinoPath = boost::dynamic_pointer_cast<core::KinodynamicPath>(path);
      if(kinoPath->length() > maxLength_){
        rejectedPath_ ++;
        return core::PathPtr_t();
      }
      Vector3 direction = kinoPath->getA1();
      direction.normalize();
      dirTotal_++;
      if(std::fabs(direction.dot(lastDirection_))>=0.8)
        dirValid_++;

      totalTimeComputed_+= kinoPath->length();
      hppDout(notice,"TotaltimeComputed = "<<totalTimeComputed_);


      assert (path && "Error while casting path shared ptr"); // really usefull ? should never happen
      core::size_type configSize = problem_.robot()->configSize() - problem_.robot()->extraConfigSpace().dimension ();

      #if !ignore_acc_bound
      // check if acceleration is valid after each sign change :
      hppStartBenchmark(INTERMEDIATE_ACCELERATION_CHECKS);
      core::vector_t t0 = kinoPath->getT0();
      core::vector_t t1 = kinoPath->getT1();
      core::vector_t tv = kinoPath->getTv();
      double t=0;
      core::ConfigurationPtr_t q(new core::Configuration_t(problem_.robot()->configSize()));
      core::vector3_t a;
      bool aValid;
      double maxT = kinoPath->length();
      hppDout(info,"## start checking intermediate accelerations");
      double epsilon = 0.0001;
      t = epsilon;
      (*kinoPath)(*q,t);
      hppDout(info,"q(t="<<t<<") = "<<pinocchio::displayConfig(*q));
      a = (*q).segment<3>(configSize+3);
      hppDout(info,"a = "<<a);
      sEq_->setG(node->getG());
      aValid = sEq_->checkAdmissibleAcceleration(node->getH(),node->geth(),a);
      hppDout(info,"a valid : "<<aValid);
      if(!aValid){
        return core::PathPtr_t();
      }
      for(size_t ijoint = 0 ; ijoint < 3 ; ijoint++){
        t=epsilon;
        if(t0[ijoint] > 0){
          hppDout(info,"for joint "<<ijoint);
          t = t0[ijoint] + epsilon; // add an epsilon to get the value after the sign change
          (*kinoPath)(*q,t);
          hppDout(info,"q(t="<<t<<") = "<<pinocchio::displayConfig(*q));
          a = (*q).segment<3>(configSize+3);
          hppDout(info,"a = "<<a);
          aValid = sEq_->checkAdmissibleAcceleration(node->getH(),node->geth(),a);
          hppDout(info,"a valid : "<<aValid);
          if(!aValid && t < maxT)
            maxT = t;
        }
        if(t1[ijoint] > 0){
          hppDout(info,"for joint "<<ijoint);
          t += t1[ijoint]; // add an epsilon to get the value after the sign change
          (*kinoPath)(*q,t);
          hppDout(info,"q(t="<<t<<") = "<<pinocchio::displayConfig(*q));
          a = (*q).segment<3>(configSize+3);
          hppDout(info,"a = "<<a);
          aValid = sEq_->checkAdmissibleAcceleration(node->getH(),node->geth(),a);
          hppDout(info,"a valid : "<<aValid);
          if(!aValid && t < maxT)
            maxT = t;
        }
        if(tv[ijoint] > 0){
          t += tv[ijoint];
          (*kinoPath)(*q,t);
          hppDout(info,"q(t="<<t<<") = "<<pinocchio::displayConfig(*q));
          a = (*q).segment<3>(configSize+3);
          hppDout(info,"a = "<<a);
          aValid = sEq_->checkAdmissibleAcceleration(node->getH(),node->geth(),a);
          hppDout(info,"a valid : "<<aValid);
          if(!aValid && t < maxT)
            maxT = t;
        }
      }

      hppDout(info, "t = "<<kinoPath->length()<<" maxT = "<<maxT);
      if(maxT < kinoPath->length()){
        maxT -= epsilon;
        totalTimeValidated_ += maxT;
        hppDout(notice,"totalTimeValidated = "<<totalTimeValidated_);
        core::PathPtr_t extracted = kinoPath->extract(core::interval_t(kinoPath->timeRange().first,kinoPath->timeRange().first + maxT));
        hppDout(notice,"extracted path : end = "<<pinocchio::displayConfig((extracted->end())));
        return extracted;
      }
      totalTimeValidated_ += kinoPath->length();
      hppDout(notice,"totalTimeValidated = "<<totalTimeValidated_);
      hppStopBenchmark(INTERMEDIATE_ACCELERATION_CHECKS);
      hppDisplayBenchmark(INTERMEDIATE_ACCELERATION_CHECKS);
      #endif
      return kinoPath;
    }

    // reverse (from q1 to x, but end() should always be x)
    core::PathPtr_t SteeringMethodKinodynamic::impl_compute (core::ConfigurationIn_t q1,core::NodePtr_t x)
    {
      core::RbprmNodePtr_t node = static_cast<core::RbprmNodePtr_t>(x);
      assert(node && "Unable to cast near node to rbprmNode");
      if(!node)
        return core::PathPtr_t();
      hppStartBenchmark(FIND_A_MAX);
      core::PathPtr_t unboundedPath =  setSteeringMethodBounds(node,q1,true);
      hppStopBenchmark(FIND_A_MAX);
      hppDisplayBenchmark(FIND_A_MAX);
      if((std::fabs(aMax_[0])+std::fabs(aMax_[1])) <= 0)
        return core::PathPtr_t();
      if(boundsUpToDate_)
          return unboundedPath;
      hppStartBenchmark(steering_kino);
      core::PathPtr_t path = core::steeringMethod::Kinodynamic::impl_compute(q1,*x->configuration());
      hppStopBenchmark(steering_kino);
      hppDisplayBenchmark(steering_kino);

      if(!path)
        return core::PathPtr_t();
      core::KinodynamicPathPtr_t kinoPath = boost::dynamic_pointer_cast<core::KinodynamicPath>(path);
      if(kinoPath->length() > maxLength_){
        rejectedPath_ ++;
        return core::PathPtr_t();
      }
      totalTimeComputed_+= kinoPath->length();
      Vector3 direction = kinoPath->getA1();
      direction.normalize();
      dirTotal_++;
      if(std::fabs(direction.dot(lastDirection_))>=0.8)
        dirValid_++;


      hppStartBenchmark(INTERMEDIATE_ACCELERATION_CHECKS);
      hppDout(notice,"TotaltimeComputed = "<<totalTimeComputed_);
      assert (path && "Error while casting path shared ptr"); // really usefull ? should never happen
      core::size_type configSize = problem_.robot()->configSize() - problem_.robot()->extraConfigSpace().dimension ();
      // check if acceleration is valid after each sign change :
      core::vector_t t0 = kinoPath->getT0();
      core::vector_t t1 = kinoPath->getT1();
      core::vector_t tv = kinoPath->getTv();
      double t=0;
      core::ConfigurationPtr_t q(new core::Configuration_t(problem_.robot()->configSize()));
      core::vector3_t a;
      bool aValid;
      double minT = 0;

      hppDout(info,"## start checking intermediate accelerations");
      double epsilon = 0.0001;
      t = kinoPath->length() - epsilon;
      (*kinoPath)(*q,t);
      hppDout(info,"q(t="<<t<<") = "<<pinocchio::displayConfig(*q));
      a = (*q).segment<3>(configSize+3);
      hppDout(info,"a = "<<a);
      sEq_->setG(node->getG());
      aValid = sEq_->checkAdmissibleAcceleration(node->getH(),node->geth(),a);
      hppDout(info,"a valid : "<<aValid);
      if(!aValid){
        return core::PathPtr_t();
      }
      for(size_t ijoint = 0 ; ijoint < 3 ; ijoint++){
        hppDout(info,"for joint "<<ijoint);
        t=-epsilon;
        if(t0[ijoint] > 0){
          hppDout(info,"for joint "<<ijoint);
          t = t0[ijoint] - epsilon; // add an epsilon to get the value after the sign change
          (*kinoPath)(*q,t);
          hppDout(info,"q(t="<<t<<") = "<<pinocchio::displayConfig(*q));
          a = (*q).segment<3>(configSize+3);
          hppDout(info,"a = "<<a);
          aValid = sEq_->checkAdmissibleAcceleration(node->getH(),node->geth(),a);
          hppDout(info,"a valid : "<<aValid);
          if(!aValid && t >minT)
            minT = t;
        }
        if(t1[ijoint] > 0){
          t += t1[ijoint];
          (*kinoPath)(*q,t);
          hppDout(info,"q(t="<<t<<") = "<<pinocchio::displayConfig(*q));
          a = (*q).segment<3>(configSize+3);
          hppDout(info,"a = "<<a);
          aValid = sEq_->checkAdmissibleAcceleration(node->getH(),node->geth(),a);
          hppDout(info,"a valid : "<<aValid);
          if(!aValid && t > minT)
            minT = t;
        }
        if(tv[ijoint] > 0){
          t += tv[ijoint];
          (*kinoPath)(*q,t);
          hppDout(info,"q(t="<<t<<") = "<<pinocchio::displayConfig(*q));
          a = (*q).segment<3>(configSize+3);
          hppDout(info,"a = "<<a);
          aValid = sEq_->checkAdmissibleAcceleration(node->getH(),node->geth(),a);
          hppDout(info,"a valid : "<<aValid);
          if(!aValid && t > minT)
            minT = t;
        }
      }
      hppDout(info, "t = "<<kinoPath->length()<<" minT = "<<minT);
      if(minT > 0){
        minT += epsilon;
        totalTimeValidated_ += (kinoPath->length() - minT);
        hppDout(notice,"totalTimeValidated = "<<totalTimeValidated_);
        core::PathPtr_t extracted = kinoPath->extract(core::interval_t(minT,kinoPath->timeRange().second));
        hppDout(notice,"extracted path : end = "<<pinocchio::displayConfig((extracted->end())));
        return extracted;
      }
      totalTimeValidated_ += kinoPath->length();
      hppDout(notice,"totalTimeValidated = "<<totalTimeValidated_);
      hppStopBenchmark(INTERMEDIATE_ACCELERATION_CHECKS);
      hppDisplayBenchmark(INTERMEDIATE_ACCELERATION_CHECKS);

      return kinoPath;
    }

    core::PathPtr_t SteeringMethodKinodynamic::computeDirection(const core::ConfigurationIn_t from, const core::ConfigurationIn_t to,bool reverse){
      setAmax(Vector3::Ones(3)*aMaxFixed_);
      hppDout(notice,"Compute direction ");
      core::PathPtr_t path;
      if(reverse)
        path = core::steeringMethod::Kinodynamic::impl_compute(to,from);
      else
        path = core::steeringMethod::Kinodynamic::impl_compute(from,to);

      Vector3 direction;
      direction = Vector3(0,0,0);
      if(path){
        core::KinodynamicPathPtr_t kinoPath = boost::dynamic_pointer_cast<core::KinodynamicPath>(path);
        if(kinoPath){
          direction = kinoPath->getA1();
          direction.normalize();
        }
      }
      lastDirection_=direction;
      return path;
    }

    core::PathPtr_t SteeringMethodKinodynamic::setSteeringMethodBounds(const core::RbprmNodePtr_t& node, const core::ConfigurationIn_t target,bool reverse) {
      Vector3 aMax ;

      // ###################################
#if ignore_acc_bound
      aMax = Vector3::Ones(3)*aMaxFixed_;
      setAmax(aMax);
      return node;
#endif
      // ####################################

      hppDout(notice,"Set bounds between : ");
      if(reverse){
        hppDout(notice,"target : "<<pinocchio::displayConfig(target));
        hppDout(notice,"node   : "<<pinocchio::displayConfig(*(node->configuration())));
      }else{
        hppDout(notice,"node   : "<<pinocchio::displayConfig(*(node->configuration())));
        hppDout(notice,"target : "<<pinocchio::displayConfig(target));
      }



      double alpha0=1.; // main variable of our LP problem
 /*     Vector3 toP,fromP,dPosition;
      Vector3 toV,fromV,dVelocity;
      const pinocchio::size_type indexECS =problem_.robot()->configSize() - problem_.robot()->extraConfigSpace().dimension (); // ecs index

      hppDout(notice,"near = "<<pinocchio::displayConfig((*(node->configuration()))));
      hppDout(notice,"target = "<<pinocchio::displayConfig(target));
      if(reverse){
        toP = node->configuration()->head(3);
        fromP = target.head(3);
        toV = node->configuration()->segment<3>(indexECS);
        fromV = target.segment<3>(indexECS);
      }else{
        fromP = node->configuration()->head(3);
        toP = target.head(3);
        fromV = node->configuration()->segment<3>(indexECS);
        toV = target.segment<3>(indexECS);
      }
      dPosition = (toP - fromP);
     // dPosition.normalize();
      dVelocity = (toV - fromV);
    //  dVelocity.normalize();
      hppDout(info, "delta position  = "<<dPosition.transpose());
      hppDout(info, "delta velocity  = "<<dVelocity.transpose());
      //direction = dPosition + dVelocity;
      direction = dPosition;
      direction.normalize();
*/
      core::PathPtr_t path = computeDirection(*(node->configuration()),target,reverse);

      if(lastDirection_.norm() <= std::numeric_limits<double>::epsilon()){
        assert(false && "ComputeDirection returned a vector of norm null");
        return core::PathPtr_t();
      }

      hppDout(info, "direction  = "<<lastDirection_.transpose());
      hppDout(info,"vector = ["<<(*(node->configuration()))[0]<<","<<(*(node->configuration()))[1]<<","<<(*(node->configuration()))[2]<<","<<lastDirection_[0]<<","<<lastDirection_[1]<<","<<lastDirection_[2]<<",0]");
      hppDout(notice,"number of contacts :  "<<node->getNumberOfContacts());

      // call to centroidal_dynamics_lib :
      sEq_->setG(node->getG());
      centroidal_dynamics::LP_status lpStatus = sEq_->findMaximumAcceleration(node->getH(), node->geth(),lastDirection_,alpha0);
      if(lpStatus==centroidal_dynamics::LP_STATUS_UNBOUNDED){
        hppDout(notice,"Primal LP problem is unbounded : "<<(lpStatus));
      }
      else if(lpStatus==centroidal_dynamics::LP_STATUS_OPTIMAL)
      {
        hppDout(notice,"Primal LP correctly solved: "<<(lpStatus));
      }
      else if(lpStatus==centroidal_dynamics::LP_STATUS_INFEASIBLE){
        hppDout(notice,"Primal LP problem could not be solved: "<<(lpStatus));
      }else{
        hppDout(notice,"Unknown error in LP : "<<lpStatus);
      }

      hppDout(info,"Amax found : "<<alpha0);
      if (alpha0 <= aMaxFixed_){
        alpha0 -= 0.01; // FIXME : hardcoded "robustness" value to avoid hitting the bounds
        hppDout(info,"Amax after min : "<<alpha0);
        aMax = alpha0*lastDirection_;
        boundsUpToDate_ = false;
      }
      else{
        alpha0 = aMaxFixed_;
        boundsUpToDate_=true;
      }

      for(size_t i = 0 ; i < 3 ; ++i)
        aMax[i] = fabs(aMax[i]); // aMax store the amplitude


      if((aMax[2] < aMaxFixed_Z_))
        aMax[2] = aMaxFixed_Z_;
      setAmax(aMax);
      hppDout(info,"Amax vector : "<<aMax_.transpose());
      //setVmax(2*Vector3::Ones(3)); //FIXME: read it from somewhere ?

      return path;
    }

  }//rbprm
}//hpp
