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

#include <stack>

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


    void RbPrmFullBody::AddLimb(const std::string& name, const std::string &effectorName,
                                const fcl::Vec3f &offset,const fcl::Vec3f &normal, const double x,
                                const double y,
                                const model::ObjectVector_t &collisionObjects, const std::size_t nbSamples, const double resolution)
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
            rbprm::RbPrmLimbPtr_t limb = rbprm::RbPrmLimb::create(joint, effectorName, offset,normal,x,y, nbSamples,resolution);
            // adding collision validation
            //core::CollisionValidationPtr_t colVal = core::CollisionValidation::create(device_);
            for(model::ObjectVector_t::const_iterator cit = collisionObjects.begin();
                cit != collisionObjects.end(); ++cit)
            {
                if(limbs_.empty())
                {
                    collisionValidation_->addObstacle(*cit);
                }
                //remove effector collision
                model::JointPtr_t collisionFree = limb->effector_;
                while(collisionFree)
                {
                    collisionValidation_->removeObstacleFromJoint(collisionFree, *cit);
                    collisionFree = collisionFree->numberChildJoints()>0 ? collisionFree->childJoint(0) : 0;
                }
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


    // assumes unit direction
    std::vector<bool> setRotationConstraints(const fcl::Vec3f& direction)
    {
        std::vector<bool> res;
        for(std::size_t i =0; i <3; ++i)
        {
            res.push_back(std::abs(direction[i]) < 2*std::numeric_limits<double>::epsilon());
            //res.push_back(true);
        }
        return res;
    }


    std::vector<bool> setTranslationConstraints(const fcl::Vec3f& normal)
    {
        std::vector<bool> res;
        for(std::size_t i =0; i <3; ++i)
        {
            res.push_back(std::abs(normal[i]) > 2*std::numeric_limits<double>::epsilon());
        }
        return res;
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
        // iterate over contact filo list
        std::queue<std::string> previousStack = previous.contactOrder_;
        while(!previousStack.empty())
        {
            const std::string name = previousStack.front();
            previousStack.pop();
            const RbPrmLimbPtr_t limb = body->GetLimbs().at(name);
            const fcl::Vec3f& z= limb->normal_;
            // try to maintain contact
            const fcl::Vec3f& ppos  =previous.contactPositions_.at(name);
            const fcl::Vec3f& pnorm  =previous.contactNormals_.at(name);
            core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(body->device_,"proj", 1e-4, 20);
            LockJointRec(limb->limb_->name(), body->device_->rootJoint(), proj);
            fcl::Matrix3f rotation = tools::GetRotationMatrix(z,pnorm);
            proj->add(core::NumericalConstraint::create (constraints::Position::create(body->device_, limb->effector_,fcl::Vec3f(0,0,0), ppos - rotation * limb->offset_)));
            proj->add(core::NumericalConstraint::create (constraints::Orientation::create(body->device_,
                                                                                          limb->effector_,
                                                                                          rotation,
                                                                                          setRotationConstraints(pnorm))));
                                                                                          //boost::assign::list_of (true)(true)(true))));
            if(proj->apply(current.configuration_) && validation->validate(current.configuration_))
            {
                // stable?
                current.contacts_[name] = true;
                current.contactPositions_[name] = previous.contactPositions_.at(name);
                current.contactNormals_[name] = previous.contactNormals_.at(name);
                ++current.nbContacts;
                current.contactOrder_.push(name);
                std::cout << "\t maintaining contact " << name << std::endl;
            }
            else
            {
                std::cout << "\t breaking contact " << name << std::endl;
            }
        }
        // reload previous configuration
        body->device_->currentConfiguration(save);
        return current;
    }

    bool ComputeCollisionFreeConfiguration(const hpp::rbprm::RbPrmFullBodyPtr_t& body,
                              State& current,
                              core::CollisionValidationPtr_t validation,
                              const hpp::rbprm::RbPrmLimbPtr_t& limb, model::ConfigurationOut_t configuration)
    {
        for(std::deque<sampling::Sample>::const_iterator cit = limb->sampleContainer_.samples_.begin();
            cit != limb->sampleContainer_.samples_.end(); ++cit)
        {
            sampling::Load(*cit, configuration);
            if(validation->validate(configuration) && stability::IsStablePoly(body,current))
            {
                current.configuration_ = configuration;
                std::cout << "found collision free non contact for " << limb->limb_->name() << std::endl;
                return true;
            }
        }
        std::cout << "no collision free configuration to maintain balance" << limb->limb_->name() << std::endl;
        return false;
    }

    bool ComputeStableContact(const hpp::rbprm::RbPrmFullBodyPtr_t& body,
                              State& current,
                              core::CollisionValidationPtr_t validation,
                              const hpp::rbprm::RbPrmLimbPtr_t& limb, model::ConfigurationOut_t configuration,
                              const model::ObjectVector_t &collisionObjects, const fcl::Vec3f& direction, fcl::Vec3f& position, fcl::Vec3f& normal,
                              bool contactIfFails = true, bool stableForOneContact = true)
    {
      // state already stable just find collision free configuration
      if(current.stable)
      {
         if (ComputeCollisionFreeConfiguration(body, current, validation, limb, configuration)) return true;
      }
      sampling::T_OctreeReport finalSet;
      fcl::Transform3f transform = limb->limb_->robot()->rootJoint()->childJoint(0)->currentTransformation (); // get root transform from configuration
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
normal = fcl::Vec3f(0,0,1);
              // Add constraints to resolve Ik
              core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(body->device_,"proj", 1e-4, 20);
              fcl::Matrix3f rotation = tools::GetRotationMatrix(z,normal);
              LockJointRec(limb->limb_->name(), body->device_->rootJoint(), proj);
              proj->add(core::NumericalConstraint::create (constraints::Position::create(body->device_,
                                                                                         limb->effector_,
                                                                                         fcl::Vec3f(0,0,0),
                                                                                         position - rotation * limb->offset_, //)));
                                                                                         model::matrix3_t::getIdentity(),
                                                                                         setTranslationConstraints(normal))));//

              proj->add(core::NumericalConstraint::create (constraints::Orientation::create(body->device_,
                                                                                            limb->effector_,
                                                                                            rotation,
                                                                                            setRotationConstraints(z))));
              if(proj->apply(configuration))
              {
                //  std::cout << "rotation " << rotation << " \n current rotation \n" << limb->effector_->currentTransformation().getRotation() << std::endl;
                if(validation->validate(configuration))
                {
                    // stabgle
                    // create new state
                    State tmp (current);
                    tmp.contacts_[limb->limb_->name()] = true;
                    tmp.contactPositions_[limb->limb_->name()] = limb->effector_->currentTransformation().getTranslation();
                    tmp.contactNormals_[limb->limb_->name()] = normal;
                    tmp.configuration_ = configuration;
                    ++tmp.nbContacts;
                    if((tmp.nbContacts == 1 && !stableForOneContact) || stability::IsStablePoly(body,tmp))
                    //if(stability::IsStable(body,tmp))
                    {
                        tmp.print();
                        found_sample = true;
                        std::cout << "handled offset " << position - limb->offset_ << std::endl;
                    }
                    else if(!unstableContact && contactIfFails)
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
      if(found_sample && (stableForOneContact || current.nbContacts > 0 ))
      {
          std::cout << "stable contact found for " << limb->limb_->name() << std::endl;
          current.stable = true;
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
          current.contactOrder_.push(name);
          current.print();
          std::cout << "state stable ? " << stability::IsStablePoly(body,current) << std::endl;
      }
      return found_sample;
    }

    void RepositionContacts(State& result, const hpp::rbprm::RbPrmFullBodyPtr_t& body,core::CollisionValidationPtr_t validation,
                            model::ConfigurationOut_t config,
                            const model::ObjectVector_t &collisionObjects, const fcl::Vec3f& direction)
    {
        // replace existing contacts
        // start with older contact created
        std::stack<std::string> poppedContacts;
        std::queue<std::string> oldOrder = result.contactOrder_;
        std::queue<std::string> newOrder;
        core::Configuration_t savedConfig = config;
        std::string nContactName ="";
        while(!result.stable &&  !oldOrder.empty())
        {
            std::string current = oldOrder.front();
            oldOrder.pop();
            fcl::Vec3f normal, position;
            if(ComputeStableContact(body, result, validation, body->GetLimbs().at(current), config, collisionObjects, direction, position, normal, false))
            {
                std::cout << "state saved with ? " << current << std::endl;
                result.stable = true;
                nContactName = current;
            }
            else
            {
                config = savedConfig;
                poppedContacts.push(current);
            }
        }
        while(!poppedContacts.empty())
        {
            newOrder.push(poppedContacts.top());
            poppedContacts.pop();
        }
        while(!oldOrder.empty())
        {
            newOrder.push(oldOrder.front());
            oldOrder.pop();
        }
        if(result.stable)
        {
            newOrder.push(nContactName);
        }
        result.contactOrder_ = newOrder;
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
            ComputeStableContact(body, result, body->collisionValidation_, lit->second, result.configuration_, collisionObjects, direction, position, normal, true, false);
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
        fcl::Vec3f normal, position;
        if(result.contacts_.find(lit->first) == result.contacts_.end())
        {
            ComputeStableContact(body, result, body->collisionValidation_, lit->second, config, collisionObjects, direction, position, normal);
        }
    }
    // reload previous configuration
    static int id = 0;
    // no stable contact was found / limb maintained
    if(!result.stable)
    {
        std::cout << "replanning unstable state " << id+1 << std::endl;
        RepositionContacts(result, body, body->collisionValidation_, config, collisionObjects, direction);
    }
    std::cout << "state " << ++id << " stable ? " << stability::IsStablePoly(body,result) << std::endl;
    body->device_->currentConfiguration(save);
    result.print();
    return result;
    }
  } // rbprm
} //hpp
