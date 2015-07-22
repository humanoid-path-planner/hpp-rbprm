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

#include <hpp/rbprm/rbprm-fullbody.hh>
#include <hpp/model/joint.hh>
#include <hpp/rbprm/tools.hh>
#include <hpp/rbprm/stability/stability.hh>

#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/locked-joint.hh>
#include <hpp/constraints/position.hh>
#include <hpp/constraints/orientation.hh>

#include <hpp/fcl/BVH/BVH_model.h>

namespace hpp {
  namespace rbprm {

    RbPrmFullBodyPtr_t RbPrmFullBody::create (const model::DevicePtr_t &device)
    {
        RbPrmFullBody* fullBody = new RbPrmFullBody(device);
        RbPrmFullBodyPtr_t res (fullBody);
        res->init (res);
        return res;
    }

    RbPrmFullBody::~RbPrmFullBody()
    {
        // NOTHING
    }


    void RbPrmFullBody::AddLimb(const std::string& name, const fcl::Vec3f &offset,
                                const fcl::Vec3f &normal,const double x, const double y,
                                const model::ObjectVector_t &collisionObjects,
                                const std::size_t nbSamples, const double resolution)
    {
        rbprm::T_Limb::const_iterator cit = limbs_.find(name);
        if(cit != limbs_.end())
        {
            throw std::runtime_error ("Impossible to add limb for joint "
                                      + name + " to robot; limb already exists");
        }
        else
        {
            model::JointPtr_t joint = device_->getJointByName(name);
            rbprm::RbPrmLimbPtr_t limb = rbprm::RbPrmLimb::create(joint, offset,normal,x,y, nbSamples,resolution);
            // adding collision validation
            //core::CollisionValidationPtr_t colVal = core::CollisionValidation::create(device_);
            for(model::ObjectVector_t::const_iterator cit = collisionObjects.begin();
                cit != collisionObjects.end(); ++cit)
            {
                collisionValidation_->addObstacle(*cit);
                //remove effector collision
                collisionValidation_->removeObstacleFromJoint(limb->effector_, *cit);
            }
            limbs_.insert(std::make_pair(name, limb));
        }
    }

    void RbPrmFullBody::init(const RbPrmFullBodyWkPtr_t& weakPtr)
    {
        weakPtr_ = weakPtr;
    }

    RbPrmFullBody::RbPrmFullBody (const model::DevicePtr_t& device)
        : device_(device)
        , collisionValidation_(core::CollisionValidation::create(device))
        , weakPtr_()
    {
        // NOTHING
    }

    void LockJointRec(const std::string& limb, const model::JointPtr_t joint, core::ConfigProjectorPtr_t& projector )
    {
        if(joint->name() == limb) return;
        const core::Configuration_t& c = joint->robot()->currentConfiguration();
        core::size_type rankInConfiguration (joint->rankInConfiguration ());
        projector->add(core::LockedJoint::create(joint,c.segment(rankInConfiguration, joint->configSize())));
        for(std::size_t i=0; i< joint->numberChildJoints(); ++i)
        {
            LockJointRec(limb,joint->childJoint(i), projector);
        }
    }

    // first step
    State MaintainPreviousContacts(const State& previous, const hpp::rbprm::RbPrmFullBodyPtr_t& body,
                                  core::CollisionValidationPtr_t validation, model::ConfigurationIn_t configuration)
    {
        // iterate over every existing contact and try to maintain them
        State current;
        core::ConfigurationIn_t save = body->device_->currentConfiguration();
        current.configuration_ = configuration;
        body->device_->currentConfiguration(configuration);
        for(T_Limb::const_iterator lit = body->GetLimbs().begin(); lit != body->GetLimbs().end(); ++lit)
        {
            const RbPrmLimbPtr_t limb = lit->second;
            const std::string& name = limb->limb_->name();
            std::map<std::string,bool>::const_iterator contactIt = previous.contacts_.find(name);
            if(contactIt != previous.contacts_.end() && contactIt->second == true)
            {
                const fcl::Vec3f& z= limb->normal_;
                // try to maintain contact
                const fcl::Vec3f& ppos  =previous.contactPositions_.at(name);
                const fcl::Vec3f& pnorm  =previous.contactNormals_.at(name);
                core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(body->device_,"proj", 1e-4, 20);
                LockJointRec(limb->limb_->name(), body->device_->rootJoint(), proj);
                proj->add(core::NumericalConstraint::create (constraints::Position::create(body->device_, limb->effector_,fcl::Vec3f(0,0,0), ppos - limb->offset_)));
                proj->add(core::NumericalConstraint::create (constraints::Orientation::create(body->device_,
                                                                                              limb->effector_,
                                                                                              tools::GetRotationMatrix(z,pnorm),
                                                                                              boost::assign::list_of (true)(true)(true))));
                if(proj->apply(current.configuration_) && validation->validate(current.configuration_))
                {
                    // stable?
                    current.contacts_[name] = true;
                    current.contactPositions_[name] = previous.contactPositions_.at(name);
                    current.contactNormals_[name] = previous.contactNormals_.at(name);
                    ++current.nbContacts;
                    std::cout << "\t maintaining contact " << name << std::endl;
                }
                else
                {
                    std::cout << "\t breaking contact " << name << std::endl;
                }
            }
            else // TODO find collision free position
            {
                //current.contacts_[name] = false;
            }
        }
        // reload previous configuration
        body->device_->currentConfiguration(save);
        return current;
    }

    bool ComputeContact(const hpp::rbprm::RbPrmFullBodyPtr_t& body,
                        core::CollisionValidationPtr_t validation,
                        const hpp::rbprm::RbPrmLimbPtr_t& limb, model::ConfigurationOut_t configuration,
                        const model::ObjectVector_t &collisionObjects, const fcl::Vec3f& direction, fcl::Vec3f& position, fcl::Vec3f& normal)
    {
      sampling::T_OctreeReport finalSet;
      fcl::Transform3f transform = limb->limb_->robot()->rootJoint()->currentTransformation (); // get root transform from configuration
      std::vector<sampling::T_OctreeReport> reports(collisionObjects.size());
      std::size_t i (0);
      //#pragma omp parallel for
      // request samples which collide with each of the collision objects
      for(model::ObjectVector_t::const_iterator oit = collisionObjects.begin();
          oit != collisionObjects.end(); ++oit, ++i)
      {
          sampling::GetCandidates(limb->sampleContainer_, transform, *oit, direction, reports[i]);
      }
      // order samples according to EFORT
      for(std::vector<sampling::T_OctreeReport>::const_iterator cit = reports.begin();
          cit != reports.end(); ++cit)
      {
          finalSet.insert(cit->begin(), cit->end());
      }
      // pick first sample which is collision free
      bool found_sample(false);
      sampling::T_OctreeReport::const_iterator it = finalSet.begin();
      for(;!found_sample && it!=finalSet.end(); ++it)
      {

          const sampling::OctreeReport& bestReport = *it;
          sampling::Load(*bestReport.sample_, configuration);

          {
              position = bestReport.contact_.pos;
              // the normal is given by the normal of the contacted object
              const fcl::Vec3f& z= limb->normal_;
              normal = bestReport.normal_;
normal = z;
              // Add constraints to resolve Ik
              core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(body->device_,"proj", 1e-4, 20);
              LockJointRec(limb->limb_->name(), body->device_->rootJoint(), proj);
              proj->add(core::NumericalConstraint::create (constraints::Position::create(body->device_, limb->effector_,fcl::Vec3f(0,0,0), position - limb->offset_)));
              proj->add(core::NumericalConstraint::create (constraints::Orientation::create(body->device_,
                                                                                            limb->effector_,
                                                                                            tools::GetRotationMatrix(z,normal),
                                                                                            boost::assign::list_of (true)(true)(true))));

              if(proj->apply(configuration))
              {
                if(validation->validate(configuration))
                {
                    found_sample = true;
                }
              }
          }
      }
      return found_sample;
    }

    bool ComputeStableContact(const hpp::rbprm::RbPrmFullBodyPtr_t& body,
                              State& current,
                              core::CollisionValidationPtr_t validation,
                              const hpp::rbprm::RbPrmLimbPtr_t& limb, model::ConfigurationOut_t configuration,
                              const model::ObjectVector_t &collisionObjects, const fcl::Vec3f& direction, fcl::Vec3f& position, fcl::Vec3f& normal)
    {
      sampling::T_OctreeReport finalSet;
      fcl::Transform3f transform = limb->limb_->robot()->rootJoint()->currentTransformation (); // get root transform from configuration
      std::vector<sampling::T_OctreeReport> reports(collisionObjects.size());
      std::size_t i (0);
      //#pragma omp parallel for
      // request samples which collide with each of the collision objects
      for(model::ObjectVector_t::const_iterator oit = collisionObjects.begin();
          oit != collisionObjects.end(); ++oit, ++i)
      {
          sampling::GetCandidates(limb->sampleContainer_, transform, *oit, direction, reports[i]);
      }
      // order samples according to EFORT
      for(std::vector<sampling::T_OctreeReport>::const_iterator cit = reports.begin();
          cit != reports.end(); ++cit)
      {
          finalSet.insert(cit->begin(), cit->end());
      }
      // pick first sample which is collision free
      bool found_sample(false);
      bool unstableContact(false); //set to true in case no stable contact is found
      core::Configuration_t unstableContactConfiguration;
      sampling::T_OctreeReport::const_iterator it = finalSet.begin();
      for(;!found_sample && it!=finalSet.end(); ++it)
      {

          const sampling::OctreeReport& bestReport = *it;
          sampling::Load(*bestReport.sample_, configuration);

          {
              position = bestReport.contact_.pos;
              // the normal is given by the normal of the contacted object
              const fcl::Vec3f& z= limb->normal_;
              normal = bestReport.normal_;
normal = z;
              // Add constraints to resolve Ik
              core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(body->device_,"proj", 1e-4, 20);
              LockJointRec(limb->limb_->name(), body->device_->rootJoint(), proj);
              proj->add(core::NumericalConstraint::create (constraints::Position::create(body->device_, limb->effector_,fcl::Vec3f(0,0,0), position - limb->offset_)));
              proj->add(core::NumericalConstraint::create (constraints::Orientation::create(body->device_,
                                                                                            limb->effector_,
                                                                                            tools::GetRotationMatrix(z,normal),
                                                                                            boost::assign::list_of (true)(true)(true))));

              if(proj->apply(configuration))
              {
                if(validation->validate(configuration))
                {
                    // stabgle
                    // create new state
                    State tmp (current);
                    tmp.contacts_[limb->limb_->name()] = true;
                    tmp.contactPositions_[limb->limb_->name()] = position;
                    tmp.contactNormals_[limb->limb_->name()] = normal;
                    tmp.configuration_ = configuration;
                    ++tmp.nbContacts;
                    if(stability::IsStablePoly(body,tmp))
                    {
                        tmp.print();
                        found_sample = true;
                    }
                    else if(!unstableContact)
                    {
                        unstableContact = true;
                        unstableContactConfiguration = configuration;
                    }
                    // if no stable candidate is found, select best contact
                    // anyway
                }
              }
          }
      }
      if(found_sample)
      {
          std::cout << "stable contact found for " << limb->limb_->name() << std::endl;
      }
      else if(!found_sample && unstableContact)
      {
          std::cout << "no stable contact found, chose one anyway " << limb->limb_->name() << std::endl;
          found_sample = true;
          configuration = unstableContactConfiguration;
      }
      else
      {
          std::cout << "did not find any contact"  << limb->limb_->name() << std::endl;
      }
      if(found_sample)
      {
          const std::string& name = limb->limb_->name();
          current.contacts_[name] = true;
          current.contactNormals_[name] = normal;
          current.contactPositions_[name] = position;
          current.configuration_ = configuration;
          ++current.nbContacts;
          current.print();
          std::cout << "state stable ? " << stability::IsStablePoly(body,current) << std::endl;
      }
      return found_sample;
    }

    hpp::rbprm::State ComputeContacts(const hpp::rbprm::RbPrmFullBodyPtr_t& body, model::ConfigurationIn_t configuration,
                                    const model::ObjectVector_t& collisionObjects, const fcl::Vec3f& direction)
    {
    const T_Limb& limbs = body->GetLimbs();
    State result;
    // save old configuration
    core::ConfigurationIn_t save = body->device_->currentConfiguration();
    result.configuration_ = configuration;
    body->device_->currentConfiguration(configuration);
    for(T_Limb::const_iterator lit = limbs.begin(); lit != limbs.end(); ++lit)
    {
        fcl::Vec3f normal, position;
        if(ComputeContact(body, body->collisionValidation_, lit->second, result.configuration_, collisionObjects, direction, position, normal))
        {
            result.contacts_[lit->first] = true;
            result.contactNormals_[lit->first] = normal;
            result.contactPositions_[lit->first] = position;
            ++result.nbContacts;
        }
        else // TODO find collision free position
        {
            //result.contacts_[lit->first] = false;
        }
    }
    // reload previous configuration
    body->device_->currentConfiguration(save);
    return result;
    }

    hpp::rbprm::State ComputeContacts(const hpp::rbprm::State& previous, const hpp::rbprm::RbPrmFullBodyPtr_t& body, model::ConfigurationIn_t configuration,
                                    const model::ObjectVector_t& collisionObjects, const fcl::Vec3f& direction)
    {
    const T_Limb& limbs = body->GetLimbs();
    // save old configuration
    core::ConfigurationIn_t save = body->device_->currentConfiguration();
    body->device_->currentConfiguration(configuration);
    State result = MaintainPreviousContacts(previous,body,body->collisionValidation_, configuration);
    core::Configuration_t config = configuration;
    for(T_Limb::const_iterator lit = limbs.begin(); lit != limbs.end(); ++lit)
    {
        // V0 only recreate non existing contacts
        fcl::Vec3f normal, position;
        if(result.contacts_.find(lit->first) == result.contacts_.end())
        {
            ComputeStableContact(body, result, body->collisionValidation_, lit->second, config, collisionObjects, direction, position, normal);
        }
        else
        {
            //result.contacts_[lit->first] = false;
        }
    }
    // reload previous configuration
    static int id = 0;
    std::cout << "state " << ++id << " stable ? " << stability::IsStablePoly(body,result) << std::endl;
    body->device_->currentConfiguration(save);
    result.print();
    return result;
    }
  } // rbprm
} //hpp
