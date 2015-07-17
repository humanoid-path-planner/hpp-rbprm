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

#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/locked-joint.hh>
#include <hpp/constraints/position.hh>
#include <hpp/constraints/orientation.hh>

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


    void RbPrmFullBody::AddLimb(const std::string& name,
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
            limbs_.insert(std::make_pair(name, rbprm::RbPrmLimb::create(joint,nbSamples,resolution)));
        }
    }

    void RbPrmFullBody::init(const RbPrmFullBodyWkPtr_t& weakPtr)
    {
        weakPtr_ = weakPtr;
    }

    RbPrmFullBody::RbPrmFullBody (const model::DevicePtr_t& device)
        : device_(device)
        , weakPtr_()
    {
        // NOTHING
    }

    void LockJointRec(const std::string& limb, const model::JointPtr_t joint, core::ConfigProjectorPtr_t& projector )
    {
        if(joint->name() == limb) return;
        const core::Configuration_t& c = joint->robot()->currentConfiguration();
        core::size_type rankInConfiguration (joint->rankInConfiguration ());
        {
            projector->add(core::LockedJoint::create(joint,c.segment(rankInConfiguration, joint->configSize())));
        }
        for(std::size_t i=0; i< joint->numberChildJoints(); ++i)
        {
            LockJointRec(limb,joint->childJoint(i), projector);
        }
    }

    bool ComputeContact(const hpp::rbprm::RbPrmFullBodyPtr_t& body,
                      const hpp::rbprm::RbPrmLimbPtr_t& limb, model::Configuration_t& configuration,
                      const model::ObjectVector_t &collisionObjects, const Eigen::Vector3d& direction, fcl::Vec3f& position, fcl::Vec3f& normal)
    {
      sampling::T_OctreeReport finalSet;
      fcl::Transform3f transform = limb->limb_->robot()->rootJoint()->currentTransformation (); // get root transform from configuration
      std::vector<sampling::T_OctreeReport> reports(collisionObjects.size());
      std::size_t i (0);
      //#pragma omp parallel for
      for(model::ObjectVector_t::const_iterator oit = collisionObjects.begin();
          oit != collisionObjects.end(); ++oit, ++i)
      {
          sampling::GetCandidates(limb->sampleContainer_, transform, *oit, direction, reports[i]);
      }
      for(std::vector<sampling::T_OctreeReport>::const_iterator cit = reports.begin();
          cit != reports.end(); ++cit)
      {
          finalSet.insert(cit->begin(), cit->end());
      }
      if(finalSet.empty()) return false;
      // pick first sample
      const sampling::OctreeReport& bestReport = *finalSet.begin();
      sampling::Load(*bestReport.sample_, configuration);
      position = bestReport.contact_.pos;
      // Add constraints to resolve Ik
      core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(body->device_,"proj", 1e-4, 20);
      LockJointRec(limb->limb_->name(), body->device_->rootJoint(), proj);
      proj->add(core::NumericalConstraint::create (constraints::Position::create(body->device_, limb->effector_,fcl::Vec3f(0,0,0), position)));
      //lock all joints
      proj->apply(configuration);
std::cout << "efector " << limb->effector_->name() << std::endl;
std::cout << "sample pos " << bestReport.sample_->effectorPosition_ << std::endl;
std::cout << "contact pos " << bestReport.contact_.pos << std::endl;
std::cout << "best found " << bestReport.sample_->id_ << std::endl;
std::cout << "robot pos " << bestReport.sample_->id_ << std::endl;
      //core::Configuration_t projectedConf = body->device_->currentConfiguration();
      //body->device_->currentConfiguration(projectedConf);
      normal = -bestReport.contact_.normal;
      return !finalSet.empty();
    }

    hpp::rbprm::State ComputeContacts(const hpp::rbprm::RbPrmFullBodyPtr_t& body, const model::Configuration_t& configuration,
                                    const model::ObjectVector_t& collisionObjects, const Eigen::Vector3d& direction)
    {
std::cout << "config " << configuration <<  std::endl;
    const T_Limb& limbs = body->GetLimbs();
    State result;
    result.configuration_ = configuration;
    for(T_Limb::const_iterator lit = limbs.begin(); lit != limbs.end(); ++lit)
    {
        fcl::Vec3f normal, position;
        if(ComputeContact(body, lit->second, result.configuration_, collisionObjects, direction, normal, position))
        {
            result.contacts_[lit->first] = true;
            result.contactNormals_[lit->first] = normal;
            result.contactPositions_[lit->first] = position;
        }
        else
        {
            result.contacts_[lit->first] = false;
        }
    }
std::cout << "config " << configuration <<  std::endl;
    return result;
    }
  } // rbprm
} //hpp
