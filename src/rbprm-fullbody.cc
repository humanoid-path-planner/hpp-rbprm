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
        , weakPtr_()
    {
        // NOTHING
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

    ContactComputationStatus ComputeStableContact(const hpp::rbprm::RbPrmFullBodyPtr_t& body,
                              State& current,
                              core::CollisionValidationPtr_t validation,
                              const std::string& limbId,
                              const hpp::rbprm::RbPrmLimbPtr_t& limb,
                              model::ConfigurationIn_t rbconfiguration,
                              model::ConfigurationOut_t configuration,
                              const affMap_t& affordances,
                              const std::map<std::string, std::vector<std::string> >& affFilters,
                              const fcl::Vec3f& direction,
                              fcl::Vec3f& position, fcl::Vec3f& normal, const double robustnessTreshold,
                              bool contactIfFails = true, bool stableForOneContact = true,
                              const sampling::heuristic evaluate = 0)
    {
        contact::ContactGenHelper contactGenHelper(body,current,current.configuration_,affordances,affFilters,robustnessTreshold,1,1,false,false,
                                          direction,fcl::Vec3f(0,0,0),contactIfFails,stableForOneContact);

        ProjectionReport rep = contact::generate_contact(contactGenHelper,limbId,evaluate);
        current = rep.result_;
        configuration = rep.result_.configuration_;
        if(rep.status_ != NO_CONTACT)
        {
            position = rep.result_.contactPositions_[limbId];
            normal = rep.result_.contactNormals_[limbId];
        }
        if(rep.status_ == STABLE_CONTACT)
            current.stable = true;
        return rep.status_;
    }

    static int nbFAILSNEW = 0;
    static int nbSUCCNEW = 0;

    hpp::rbprm::State ComputeContacts(const hpp::rbprm::RbPrmFullBodyPtr_t& body,
			model::ConfigurationIn_t configuration, const affMap_t& affordances,
      const std::map<std::string, std::vector<std::string> >& affFilters, const fcl::Vec3f& direction,
			const double robustnessTreshold)
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
                                hpp::model::ObjectVector_t affs = contact::getAffObjectsForLimb (lit->first,
                                    affordances, affFilters);

                fcl::Vec3f normal, position;
                ComputeStableContact(body,result,
                                    body->limbcollisionValidations_.at(lit->first), lit->first,
                                    lit->second, configuration, result.configuration_, affordances,affFilters,
                                    direction, position, normal, robustnessTreshold, false, false);
            }
            result.nbContacts = result.contactNormals_.size();
        }
        // reload previous configuration
        body->device_->currentConfiguration(save);
        return result;
    }

    hpp::rbprm::State ComputeContacts(const hpp::rbprm::State& previous,
			const hpp::rbprm::RbPrmFullBodyPtr_t& body,
			model::ConfigurationIn_t configuration,
			const affMap_t& affordances,
			const std::map<std::string, std::vector<std::string> >& affFilters,
            const fcl::Vec3f& direction, bool& contactMaintained, bool& multipleBreaks, bool& repositioned,
      const bool allowFailure, const double robustnessTreshold)
    {
        // save old configuration
        core::ConfigurationIn_t save = body->device_->currentConfiguration();
        model::Device::Computation_t flag = body->device_->computationFlag ();
        model::Device::Computation_t newflag = static_cast <model::Device::Computation_t> (model::Device::JOINT_POSITION);
        // load new root position
        body->device_->controlComputation (newflag);
        body->device_->currentConfiguration(configuration);
        body->device_->computeForwardKinematics ();
        // try to maintain previous contacts
        contact::ContactGenHelper cHelper(body,previous,configuration,affordances,affFilters,robustnessTreshold,1,1,false,
                                          true,direction,fcl::Vec3f(0,0,0),false,false);
        contact::ContactReport rep = contact::oneStep(cHelper);
        contactMaintained = rep.success_ && rep.contactMaintained_;
        multipleBreaks = rep.multipleBreaks_;

        // copy extra dofs
        if(rep.success_)
        {
            const model::size_type& extraDim = body->device_->extraConfigSpace().dimension();
            rep.result_.configuration_.tail(extraDim) = configuration.tail(extraDim);
        }
        else
        {
            rep.multipleBreaks_ = true;
            std::cout << "Contact gen failure" << ++nbFAILSNEW << std::endl;
        }
        if(rep.repositionedInPlace_)
        {
            repositioned = true;
        }
        else
        {
            ++nbSUCCNEW;
        }
        body->device_->currentConfiguration(save);
        body->device_->controlComputation (flag);
        return rep.result_;
    }
  } // rbprm
} //hpp
