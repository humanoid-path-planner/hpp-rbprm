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
#include <hpp/rbprm/ik-solver.hh>
#include <hpp/rbprm/projection/projection.hh>
#include <hpp/rbprm/contact_generation/contact_generation.hh>
#include <hpp/rbprm/contact_generation/algorithm.hh>

#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/locked-joint.hh>
#include <hpp/model/device.hh>
#include <hpp/constraints/generic-transformation.hh>
#include <hpp/model/configuration.hh>
#include <hpp/fcl/BVH/BVH_model.h>

#include <stack>

#ifdef PROFILE
    #include "hpp/rbprm/rbprm-profiler.hh"
#endif

using namespace hpp::rbprm::projection;

namespace hpp {
  namespace rbprm {

    const double epsilon = 10e-3;

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

    bool RbPrmFullBody::AddHeuristic(const std::string& name, const sampling::heuristic func)
    {
        return factory_.AddHeuristic(name, func);
    }


    void RbPrmFullBody::AddLimbPrivate(rbprm::RbPrmLimbPtr_t limb, const std::string& id, const std::string& name,
                        const model::ObjectVector_t &collisionObjects, const bool disableEffectorCollision)
    {
        core::CollisionValidationPtr_t limbcollisionValidation_ = core::CollisionValidation::create(this->device_);
        // adding collision validation
        for(model::ObjectVector_t::const_iterator cit = collisionObjects.begin();
            cit != collisionObjects.end(); ++cit)
        {
            if(limbs_.empty())
            {
                collisionValidation_->addObstacle(*cit);
            }
            limbcollisionValidation_->addObstacle(*cit);
            //remove effector collision
            if(disableEffectorCollision)
            {
                hpp::tools::RemoveEffectorCollision<core::CollisionValidation>((*collisionValidation_.get()), limb->effector_, *cit);
                hpp::tools::RemoveEffectorCollision<core::CollisionValidation>((*limbcollisionValidation_.get()), limb->effector_, *cit);
            }
        }
        limbs_.insert(std::make_pair(id, limb));
        tools::RemoveNonLimbCollisionRec<core::CollisionValidation>(device_->rootJoint(),name,collisionObjects,*limbcollisionValidation_.get());
        hpp::core::RelativeMotion::matrix_type m = hpp::core::RelativeMotion::matrix(device_);
        limbcollisionValidation_->filterCollisionPairs(m);
        collisionValidation_->filterCollisionPairs(m);
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

    std::map<std::string, const sampling::heuristic>::const_iterator checkLimbData(const std::string& id, const rbprm::T_Limb& limbs, const rbprm::sampling::HeuristicFactory& factory, const std::string& heuristicName)
    {
        rbprm::T_Limb::const_iterator cit = limbs.find(id);
        std::map<std::string, const sampling::heuristic>::const_iterator hit = factory.heuristics_.find(heuristicName);
        if(cit != limbs.end())
            throw std::runtime_error ("Impossible to add limb for joint "
                                      + id + " to robot; limb already exists");
        else if(hit == factory.heuristics_.end())
            throw std::runtime_error ("Impossible to add limb for joint "
                                      + id + " to robot; heuristic not found " + heuristicName +".");
        return hit;
    }

    void RbPrmFullBody::AddLimb(const std::string& id, const std::string& name, const std::string &effectorName,
                                const fcl::Vec3f &offset,const fcl::Vec3f &normal, const double x,
                                const double y,
                                const model::ObjectVector_t &collisionObjects, const std::size_t nbSamples, const std::string &heuristicName, const double resolution,
                                ContactType contactType, const bool disableEffectorCollision,  const bool grasp)
    {
        std::map<std::string, const sampling::heuristic>::const_iterator hit = checkLimbData(id, limbs_,factory_,heuristicName);
        model::JointPtr_t joint = device_->getJointByName(name);
        rbprm::RbPrmLimbPtr_t limb = rbprm::RbPrmLimb::create(joint, effectorName, offset,normal,x,y, nbSamples, hit->second, resolution,contactType, disableEffectorCollision, grasp);
        AddLimbPrivate(limb, id, name,collisionObjects, disableEffectorCollision);
    }

    void RbPrmFullBody::AddLimb(const std::string& database, const std::string& id,
                                const model::ObjectVector_t &collisionObjects,
                                const std::string& heuristicName,
                                const bool loadValues, const bool disableEffectorCollision,
                                const bool grasp)
    {
        std::map<std::string, const sampling::heuristic>::const_iterator hit = checkLimbData(id, limbs_,factory_,heuristicName);;
        std::ifstream myfile (database.c_str());
        if (!myfile.good())
            throw std::runtime_error ("Impossible to open database");
        rbprm::RbPrmLimbPtr_t limb = rbprm::RbPrmLimb::create(device_, myfile, loadValues, hit->second, disableEffectorCollision, grasp);
        myfile.close();
        AddLimbPrivate(limb, id, limb->limb_->name(),collisionObjects, disableEffectorCollision);
    }

    void RbPrmFullBody::init(const RbPrmFullBodyWkPtr_t& weakPtr)
    {
        weakPtr_ = weakPtr;
    }

    RbPrmFullBody::RbPrmFullBody (const model::DevicePtr_t& device)
        : device_(device)
        , collisionValidation_(core::CollisionValidation::create(device))
        , staticStability_(true)
        , mu_(0.5)
        , weakPtr_()
    {
        // NOTHING
    }
  } // rbprm
} //hpp
