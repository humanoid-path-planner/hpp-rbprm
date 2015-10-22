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
#include <hpp/model/device.hh>
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


    void RemoveNonLimbCollisionRec(const model::JointPtr_t joint, const std::string& limbname,
                                   const model::ObjectVector_t &collisionObjects,
                                   core::CollisionValidationPtr_t& collisionValidation)
    {
        if(joint->name() == limbname) return;
        for(model::ObjectVector_t::const_iterator cit = collisionObjects.begin();
            cit != collisionObjects.end(); ++cit)
        {
            collisionValidation->removeObstacleFromJoint(joint, *cit);
        }
        for(std::size_t i=0; i<joint->numberChildJoints(); ++i)
        {
            RemoveNonLimbCollisionRec(joint->childJoint(i), limbname, collisionObjects, collisionValidation);
        }
    }


    bool RbPrmFullBody::AddHeuristic(const std::string& name, const sampling::heuristic func)
    {
        return factory_.AddHeuristic(name, func);
    }

    void RbPrmFullBody::AddLimb(const std::string& id, const std::string& name, const std::string &effectorName,
                                const fcl::Vec3f &offset,const fcl::Vec3f &normal, const double x,
                                const double y,
                                const model::ObjectVector_t &collisionObjects, const std::size_t nbSamples, const std::string &heuristicName, const double resolution)
    {
        rbprm::T_Limb::const_iterator cit = limbs_.find(id);
        std::map<std::string, const sampling::heuristic>::const_iterator hit = factory_.heuristics_.find(heuristicName);
        if(cit != limbs_.end())
        {
            throw std::runtime_error ("Impossible to add limb for joint "
                                      + id + " to robot; limb already exists");
        }
        else if(hit == factory_.heuristics_.end())
        {
            throw std::runtime_error ("Impossible to add limb for joint "
                                      + id + " to robot; heuristic not found " + heuristicName +".");
        }
        else
        {
            model::JointPtr_t joint = device_->getJointByName(name);
            rbprm::RbPrmLimbPtr_t limb = rbprm::RbPrmLimb::create(joint, effectorName, offset,normal,x,y, nbSamples, hit->second, resolution);
            core::CollisionValidationPtr_t limbcollisionValidation_ = core::CollisionValidation::create(this->device_);
            // adding collision validation
            //core::CollisionValidationPtr_t colVal = core::CollisionValidation::create(device_);
            for(model::ObjectVector_t::const_iterator cit = collisionObjects.begin();
                cit != collisionObjects.end(); ++cit)
            {
                if(limbs_.empty())
                {
                    collisionValidation_->addObstacle(*cit);
                }
                limbcollisionValidation_->addObstacle(*cit);
                //remove effector collision
                model::JointPtr_t collisionFree = limb->effector_;
                while(collisionFree)
                {
                    collisionValidation_->removeObstacleFromJoint(collisionFree, *cit);
                    limbcollisionValidation_->removeObstacleFromJoint(collisionFree,*cit);
                    collisionFree = collisionFree->numberChildJoints()>0 ? collisionFree->childJoint(0) : 0;
                }
            }
            limbs_.insert(std::make_pair(id, limb));
            RemoveNonLimbCollisionRec(device_->rootJoint(),name,collisionObjects,limbcollisionValidation_);
            limbcollisionValidations_.insert(std::make_pair(id, limbcollisionValidation_));
            // insert limb to root group
            T_LimbGroup::iterator cit = limbGroups_.find(name);
            if(cit != limbGroups_.end())
            {
                cit->second.push_back(id);
            }
            else
            {
                std::vector<std::string> group;
                group.push_back(id);
                limbGroups_.insert(std::make_pair(name, group));
            }

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
    std::vector<bool> setMaintainRotationConstraints(const fcl::Vec3f&) // direction)
    {
        std::vector<bool> res;
        for(std::size_t i =0; i <3; ++i)
        {
            //res.push_back(std::abs(direction[i]) < 0.2);
            res.push_back(true);
        }
        return res;
    }

    std::vector<bool> setRotationConstraints(const fcl::Vec3f&)// direction)
    {
        std::vector<bool> res;
        for(std::size_t i =0; i <3; ++i)
        {
            //res.push_back(std::abs(direction[i]) < 2*std::numeric_limits<double>::epsilon());
            res.push_back(true);
        }
        return res;
    }


    std::vector<bool> setTranslationConstraints(const fcl::Vec3f&)// normal)
    {
        std::vector<bool> res;
        for(std::size_t i =0; i <3; ++i)
        {
            //res.push_back(std::abs(normal[i]) > 0.2);
            res.push_back(true);
        }
        return res;
    }

    bool ComputeCollisionFreeConfiguration(const hpp::rbprm::RbPrmFullBodyPtr_t& body,
                              State& current,
                              core::CollisionValidationPtr_t validation,
                              const hpp::rbprm::RbPrmLimbPtr_t& limb, model::ConfigurationOut_t configuration,
                                           bool stability = true)
    {
        for(std::deque<sampling::Sample>::const_iterator cit = limb->sampleContainer_.samples_.begin();
            cit != limb->sampleContainer_.samples_.end(); ++cit)
        {
            sampling::Load(*cit, configuration);
            if(validation->validate(configuration) && (!stability || stability::IsStable(body,current)))
            {
                current.configuration_ = configuration;
                //std::cout << "found collision free non contact for " << limb->limb_->name() << std::endl;
                return true;
            }
        }
        //std::cout << "no collision free configuration to maintain balance" << limb->limb_->name() << std::endl;
        return false;
    }

    // first step
    State MaintainPreviousContacts(const State& previous, const hpp::rbprm::RbPrmFullBodyPtr_t& body,
                                   std::map<std::string,core::CollisionValidationPtr_t>& limbValidations,
                                   model::ConfigurationIn_t configuration, bool& contactMaintained, bool& multipleBreaks)
    {
        contactMaintained = true;
        std::vector<std::string> brokenContacts;
        // iterate over every existing contact and try to maintain them
        State current;
        current.configuration_ = configuration;
        model::Configuration_t config = configuration;
        core::ConfigurationIn_t save = body->device_->currentConfiguration();
        // iterate over contact filo list
        std::queue<std::string> previousStack = previous.contactOrder_;
        while(!previousStack.empty())
        {
            const std::string name = previousStack.front();
            previousStack.pop();
            const RbPrmLimbPtr_t limb = body->GetLimbs().at(name);
            // try to maintain contact
            const fcl::Vec3f& ppos  =previous.contactPositions_.at(name);
            //const fcl::Vec3f& pnorm  =previous.contactNormals_.at(name);
            core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(body->device_,"proj", 1e-2, 30);
            LockJointRec(limb->limb_->name(), body->device_->rootJoint(), proj);
//const fcl::Vec3f z = limb->effector_->currentTransformation().getRotation() * limb->normal_;
            //const fcl::Matrix3f alignRotation = tools::GetRotationMatrix(z,previous.contactNormals_.at(name));
            //const fcl::Matrix3f rotation = alignRotation * limb->effector_->currentTransformation().getRotation();
//const fcl::Matrix3f& rotation = previous.contactRotation_.at(name);
            proj->add(core::NumericalConstraint::create (constraints::Position::create(body->device_, limb->effector_,fcl::Vec3f(0,0,0), ppos /*- rotation * limb->offset_*/)));
            /**/
/*proj->add(core::NumericalConstraint::create (constraints::Orientation::create(body->device_,
                                                                              limb->effector_,
                                                                              rotation,//previous.contactRotation_.at(name),
                                                                              setMaintainRotationConstraints(z))));*/

            if(proj->apply(config))
            {
                 //boost::assign::list_of (true)(true)(true))));
                if(limbValidations.at(name)->validate(config))
                {
                    // stable?
                    current.contacts_[name] = true;
                    current.contactPositions_[name] = previous.contactPositions_.at(name);
                    current.contactNormals_[name] = previous.contactNormals_.at(name);
                    current.contactRotation_[name] = previous.contactRotation_.at(name);
                    current.contactOrder_.push(name);
                    current.configuration_ = config;
                    //std::cout << "\t maintaining contact " << name << "position" << previous.contactPositions_.at(name) << std::endl;
                    //std::cout << "\t real position" << limb->effector_->currentTransformation().getTranslation() << std::endl;
                }
                else
                {
                    contactMaintained = false;
//std::cout << "\t breaking contact (collision) " << name << std::endl;
                    ComputeCollisionFreeConfiguration(body,current,limbValidations.at(name),limb,current.configuration_,false);
                    brokenContacts.push_back(name);
                }
            }
            else
            {
                contactMaintained = false;
//std::cout << "\t breaking contact (invalid configuration) " << name << " posi " << ppos << std::endl;
                ComputeCollisionFreeConfiguration(body,current,limbValidations.at(name),limb,current.configuration_,false);
                brokenContacts.push_back(name);
            }
        }
        // reload previous configuration
        body->device_->currentConfiguration(save);
        if(brokenContacts.size() > 1)
        {
            contactMaintained = false;
            multipleBreaks = true;
            // replanning for last contact created.
// std::cout << "*********** MULTI CONTACT BREK ****** " << brokenContacts.size() << " " << std::endl;
        }
        return current;
    }

    bool ContactExistsWithinGroup(const hpp::rbprm::RbPrmLimbPtr_t& limb,
                                  const hpp::rbprm::RbPrmFullBody::T_LimbGroup& limbGroups,
                                  const State& current)
    {
        const std::vector<std::string>& group = limbGroups.at(limb->limb_->name());
        for(std::vector<std::string>::const_iterator cit = group.begin();
            cit != group.end(); ++cit)
        {
            if(current.contactPositions_.find(*cit) != current.contactPositions_.end())
            {
                return true;
            }
        }
        return false;
    }


    bool ComputeStableContact(const hpp::rbprm::RbPrmFullBodyPtr_t& body,
                              State& current,
                              core::CollisionValidationPtr_t validation,
                              const std::string& limbId,
                              const hpp::rbprm::RbPrmLimbPtr_t& limb,
                              model::ConfigurationIn_t rbconfiguration, model::ConfigurationIn_t nextrbconfiguration,
                              model::ConfigurationOut_t configuration,
                              const model::ObjectVector_t &collisionObjects, const fcl::Vec3f& direction, fcl::Vec3f& position, fcl::Vec3f& normal,
                              bool contactIfFails = true, bool stableForOneContact = true,
                              const sampling::heuristic evaluate = 0)
    {
        bool apply = false;
      // state already stable just find collision free configuration
      if(current.stable)
      {
         //if (ComputeCollisionFreeConfiguration(body, current, validation, limb, configuration)) return true;
      }
      fcl::Matrix3f rotation;
      sampling::T_OctreeReport finalSet;

      limb->limb_->robot()->currentConfiguration(nextrbconfiguration);
      limb->limb_->robot()->computeForwardKinematics ();
      fcl::Transform3f transformNext = limb->limb_->robot()->rootJoint()->childJoint(0)->currentTransformation (); // get root transform from configuration

      limb->limb_->robot()->currentConfiguration(rbconfiguration);
      limb->limb_->robot()->computeForwardKinematics ();
      fcl::Transform3f transform = limb->limb_->robot()->rootJoint()->childJoint(0)->currentTransformation (); // get root transform from configuration

      std::vector<sampling::T_OctreeReport> reports(collisionObjects.size());
      std::size_t i (0);
      //#pragma omp parallel for
      // request samples which collide with each of the collision objects
      sampling::heuristic eval = evaluate; if(!eval) eval =  limb->sampleContainer_.evaluate_;
      for(model::ObjectVector_t::const_iterator oit = collisionObjects.begin();
          oit != collisionObjects.end(); ++oit, ++i)
      {
          sampling::GetCandidates(limb->sampleContainer_, transform,transformNext, *oit, direction, reports[i], eval);
      }
      // order samples according to EFORT
      for(std::vector<sampling::T_OctreeReport>::const_iterator cit = reports.begin();
          cit != reports.end(); ++cit)
      {
          finalSet.insert(cit->begin(), cit->end());
      }
//std::cout << "candidates " << finalSet.size() << std::endl;
      // pick first sample which is collision free
      bool found_sample(false);
      bool unstableContact(false); //set to true in case no stable contact is found
      core::Configuration_t unstableContactConfiguration;
      sampling::T_OctreeReport::const_iterator it = finalSet.begin();
      for(;!found_sample && it!=finalSet.end(); ++it)
      {
          const sampling::OctreeReport& bestReport = *it;
          sampling::Load(*bestReport.sample_, configuration);
          body->device_->currentConfiguration(configuration);
          {

/********************/
// Justin
 /*             normal = bestReport.normal_;
//normal = fcl::Vec3f(0,0,1);
              position = bestReport.contact_.pos;
              // the normal is given by the normal of the contacted object
              //const fcl::Vec3f& z= limb->normal_;
              fcl::Vec3f z = limb->effector_->currentTransformation().getRotation() * limb->normal_;
              fcl::Matrix3f alignRotation = tools::GetRotationMatrix(limb->normal_,normal);
              const fcl::Matrix3f alignRotationPos = tools::GetRotationMatrix(z,normal);
              if(limbId.find("3Rarm")!=std::string::npos)
              {
                  fcl::Matrix3f zr;
                  zr(0,0) = 0; zr(0,1) = -1; zr(0,2) = 0;
                  zr(1,0) = 1;zr(1,1) = 0; zr(1,2) = 0;
                  zr(2,0) = 0; zr(2,1) = 0; zr(2,2) = 1;
                  alignRotation = alignRotation * zr;
              }
              rotation = alignRotationPos * limb->effector_->currentTransformation().getRotation();
              // Add constraints to resolve Ik
              core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(body->device_,"proj", 1e-4, 20);
              //rotation = tools::GetRotationMatrix(z,normal);
              // get current normal orientation
              LockJointRec(limb->limb_->name(), body->device_->rootJoint(), proj);
              proj->add(core::NumericalConstraint::create (constraints::Position::create(body->device_,
                                                                                         limb->effector_,
                                                                                         fcl::Vec3f(0,0,0),
                                                                                         position - rotation * limb->offset_, //)));
                                                                                         model::matrix3_t::getIdentity(),
                                                                                         setTranslationConstraints(normal))));//


              proj->add(core::NumericalConstraint::create (constraints::Orientation::create(body->device_,
                                                                                            limb->effector_,
                                                                                            alignRotation,
                                                                                            setRotationConstraints(z))));*/
/****************************/




              normal = bestReport.normal_;
//normal = fcl::Vec3f(0,0,1);
              position = bestReport.contact_.pos;
              // the normal is given by the normal of the contacted object
              //const fcl::Vec3f& z= limb->normal_;
              const fcl::Vec3f z = limb->effector_->currentTransformation().getRotation() * limb->normal_;
              const fcl::Matrix3f alignRotation = tools::GetRotationMatrix(z,normal);
              rotation = alignRotation * limb->effector_->currentTransformation().getRotation();
              // Add constraints to resolve Ik
              core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(body->device_,"proj", 1e-4, 20);
              //rotation = tools::GetRotationMatrix(z,normal);
              // get current normal orientation
              LockJointRec(limb->limb_->name(), body->device_->rootJoint(), proj);
              proj->add(core::NumericalConstraint::create (constraints::Position::create(body->device_,
                                                                                         limb->effector_,
                                                                                         fcl::Vec3f(0,0,0),
                                                                                         position - rotation * limb->offset_, //)));
                                                                                         model::matrix3_t::getIdentity(),
                                                                                         setTranslationConstraints(normal))));//


              /*proj->add(core::NumericalConstraint::create (constraints::Orientation::create(body->device_,
                                                                                            limb->effector_,
                                                                                            rotation,
                                                                                            setRotationConstraints(z))));*/

              if(proj->apply(configuration))
              {
                apply = true;
                //  std::cout << "rotation " << rotation << " \n current rotation \n" << limb->effector_->currentTransformation().getRotation() << std::endl;
                if(validation->validate(configuration))
                {
                    // stabgle
                    // create new state
                    body->device_->currentConfiguration(configuration);
                    body->device_->computeForwardKinematics();
                    State tmp (current);
                    tmp.contacts_[limbId] = true;
                    tmp.contactPositions_[limbId] = limb->effector_->currentTransformation().getTranslation();
                    tmp.contactRotation_[limbId] = limb->effector_->currentTransformation().getRotation();
                    tmp.contactNormals_[limbId] = normal;
                    tmp.configuration_ = configuration;
                    ++tmp.nbContacts;
                    if((tmp.nbContacts == 1 && !stableForOneContact) || stability::IsStable(body,tmp))
                    //if(stability::IsStable(body,tmp))
                    {
                        position = limb->effector_->currentTransformation().getTranslation();
                        rotation = limb->effector_->currentTransformation().getRotation();
                        found_sample = true;
                        //std::cout << "handled offset " << position - limb->offset_ << std::endl;
                    }
                    else if(!unstableContact && contactIfFails)
                    {
                        position = limb->effector_->currentTransformation().getTranslation();
                        rotation = limb->effector_->currentTransformation().getRotation();
                        unstableContact = true;
                        unstableContactConfiguration = configuration;
                    }
                    // if no stable candidate is found, select best contact
                    // anyway
                }
              }
          }
      }
      if(found_sample)// && (stableForOneContact || current.nbContacts > 0 ))
      {
          //std::cout << "stable contact found for " << limbId << std::endl;
          current.stable = true;
      }
      else if(!found_sample && unstableContact)
      {
//std::cout << "no stable contact found, chose one anyway " << limbId << std::endl;
          found_sample = true;
          configuration = unstableContactConfiguration;
      }
      else
      {
//std::cout << "did not find any contact"  << limbId << std::endl;
          if(!found_sample)
          {
              ComputeCollisionFreeConfiguration(body, current, validation, limb, configuration,false);
          }
          if(apply)
          {
//std::cout << "too many collisions found " << std::endl;
          }
      }
      if(found_sample)
      {
          current.contacts_[limbId] = true;
          current.contactNormals_[limbId] = normal;
          current.contactPositions_[limbId] = position;
          current.contactRotation_[limbId] = rotation;
          current.configuration_ = configuration;
          current.contactOrder_.push(limbId);
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
        State previous = result;
        while(!result.stable &&  !oldOrder.empty())
        {
            std::string previousContactName = oldOrder.front();
            std::string groupName = body->GetLimbs().at(previousContactName)->limb_->name();
            const std::vector<std::string>& group = body->GetGroups().at(groupName);
//std::cout << " testing state " << groupName << std::endl;
            oldOrder.pop();
            fcl::Vec3f normal, position;
            core::ConfigurationIn_t save = body->device_->currentConfiguration();
            bool notFound(true);
            for(std::vector<std::string>::const_iterator cit = group.begin();
                notFound && cit != group.end(); ++cit)
            {
                if(ComputeStableContact(body, result, validation, *cit, body->GetLimbs().at(*cit),save,save, config, collisionObjects, direction, position, normal, false))
                {
                    //std::cout << "state saved with " << *cit << std::endl;
                    //std::cout << "position  " << result.contactPositions_.at(*cit) << std::endl;
                    result.stable = true;
                    nContactName = *cit;
                    notFound = false;
                }
                else
                {
                    result = previous;
                    config = savedConfig;
                }
            }
            if(notFound)
            {
                config = savedConfig;
                result.configuration_ = savedConfig;
                poppedContacts.push(previousContactName);
                body->device_->currentConfiguration(save);
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
        body->device_->computeForwardKinematics();
        for(T_Limb::const_iterator lit = limbs.begin(); lit != limbs.end(); ++lit)
        {
            if(!ContactExistsWithinGroup(lit->second, body->limbGroups_ ,result))
            {
                fcl::Vec3f normal, position;
                ComputeStableContact(body,result, body->limbcollisionValidations_.at(lit->first), lit->first, lit->second,
                                     configuration, configuration,
                                     result.configuration_, collisionObjects, direction, position, normal, true, false);
            }
        }
        // reload previous configuration
        body->device_->currentConfiguration(save);
        return result;
    }

    hpp::rbprm::State ComputeContacts(const hpp::rbprm::State& previous, const hpp::rbprm::RbPrmFullBodyPtr_t& body, model::ConfigurationIn_t configuration,
                                      model::ConfigurationIn_t nextconfiguration,
                                      const model::ObjectVector_t& collisionObjects, const fcl::Vec3f& direction, bool& contactMaintained, bool& multipleBreaks, const bool allowFailure)
    {
    static int id = 0;
    const T_Limb& limbs = body->GetLimbs();
    // save old configuration
    core::ConfigurationIn_t save = body->device_->currentConfiguration();
    model::Device::Computation_t flag = body->device_->computationFlag ();
    model::Device::Computation_t newflag = static_cast <model::Device::Computation_t> (model::Device::JOINT_POSITION);
    body->device_->controlComputation (newflag);
    body->device_->currentConfiguration(configuration);
    body->device_->computeForwardKinematics ();
    State result = MaintainPreviousContacts(previous,body, body->limbcollisionValidations_, configuration, contactMaintained, multipleBreaks);
    if(multipleBreaks && !allowFailure)
    {
        fcl::Vec3f normal, position;
        result = previous;
//result.configuration_.head<3>() = previous.configuration_.head<3>() + (configuration - previous.configuration_).head<3>() * 0.05;
        result.stable = false;
        std::string replaceContact =  result.RemoveFirstContact();
        if(replaceContact.empty()) return result;
        model::Configuration_t config = previous.configuration_;
        body->device_->currentConfiguration(config);
        body->device_->computeForwardKinematics();
        ComputeStableContact(body,result,body->limbcollisionValidations_.at(replaceContact),replaceContact,body->limbs_.at(replaceContact),
                             configuration,nextconfiguration, config,collisionObjects,direction,position, normal, true, false, body->factory_.heuristics_["random"]);
        body->device_->currentConfiguration(save);
        body->device_->controlComputation (flag);
        ++id;
        return result;
    }
    core::Configuration_t config = result.configuration_;
    for(T_Limb::const_iterator lit = limbs.begin(); lit != limbs.end(); ++lit)
    {
        fcl::Vec3f normal, position;
        if(result.contacts_.find(lit->first) == result.contacts_.end())
        {
            if(!ContactExistsWithinGroup(lit->second, body->limbGroups_ ,result))
            {
                contactMaintained = !ComputeStableContact(body, result, body->limbcollisionValidations_.at(lit->first), lit->first, lit->second, configuration, nextconfiguration,
                                                          config, collisionObjects, direction, position, normal) && contactMaintained;
            }
        }
    }
    // reload previous configuration
    // no stable contact was found / limb maintained
    if(!result.stable)
    {
        if(contactMaintained)
        {
            contactMaintained = false;
//std::cout << "replanning unstable state " << id+1 << std::endl;
            RepositionContacts(result, body, body->collisionValidation_, config, collisionObjects, direction);
        }
        else // already broke contact; state is invalid
        {
//std::cout << "contact broken and replaced contact unstable, replanning " << id+1 << std::endl;
            fcl::Vec3f normal, position;
            result = previous;
            result.stable = false;
            std::string replaceContact =  result.RemoveFirstContact();
            if(!replaceContact.empty())
            {
                model::Configuration_t config = previous.configuration_;
                body->device_->currentConfiguration(config);
                body->device_->computeForwardKinematics();
                if(!ComputeStableContact(body,result,body->limbcollisionValidations_.at(replaceContact),replaceContact,
                                     body->limbs_.at(replaceContact),
                                     configuration, nextconfiguration, config,
                                     collisionObjects,direction,position, normal))
                {
//std::cout << "could not replace contact " << id+1 << std::endl;
                }
            }
            body->device_->currentConfiguration(save);
            body->device_->controlComputation (flag);
            ++id;
            return result;
        }
    }
    bool stab =  stability::IsStable(body,result);
    std::cout << "state " << ++id << " stable ? " << stab << " " << result.stable << std::endl;
    if(!stab && result.stable)
    {
        std::cout << "error " << std::endl;
        result.stable = false;
    }
    body->device_->currentConfiguration(save);    
    body->device_->controlComputation (flag);
    result.nbContacts = result.contactNormals_.size();
    return result;
    }
  } // rbprm
} //hpp
