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
#include <hpp/pinocchio/joint.hh>
#include <hpp/rbprm/tools.hh>
#include <hpp/rbprm/stability/stability.hh>
#include <hpp/rbprm/projection/projection.hh>
#include <hpp/rbprm/contact_generation/contact_generation.hh>
#include <hpp/rbprm/contact_generation/algorithm.hh>

#include <hpp/core/constraint-set.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/constraints/locked-joint.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/constraints/generic-transformation.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/fcl/BVH/BVH_model.h>

#include <stack>

#ifdef PROFILE
    #include "hpp/rbprm/rbprm-profiler.hh"
#endif

using namespace hpp::rbprm::projection;

namespace hpp {
  namespace rbprm {

    const double epsilon = 10e-3;

    RbPrmFullBodyPtr_t RbPrmFullBody::create (const pinocchio::DevicePtr_t &device)
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
                        const hpp::core::ObjectStdVector_t &collisionObjects, const bool disableEffectorCollision,const bool nonContactingLimb)
    {
        core::CollisionValidationPtr_t limbcollisionValidation_ = core::CollisionValidation::create(this->device_);
        rbprm::T_Limb limbs;
        if(nonContactingLimb)
          limbs=nonContactingLimbs_;
        else
          limbs=limbs_;        
        //pinocchio::JointPtr_t effectorJoint (new pinocchio::Joint(limb->effector_.joint()));
        hpp::tools::addLimbCollisionRec<hpp::core::CollisionValidation>
                (limb->limb_, limb->effector_, collisionObjects,(*limbcollisionValidation_.get()), disableEffectorCollision);
        if(limbs.empty())
        {
            hpp::tools::addLimbCollisionRec<hpp::core::CollisionValidation>
                (device_->rootJoint(), limb->effector_, collisionObjects,(*collisionValidation_.get()), disableEffectorCollision);
        }
        // adding collision validation
        /*for(hpp::core::ObjectStdVector_t::const_iterator cit = collisionObjects.begin();
            cit != collisionObjects.end(); ++cit)
        {
            if(limbs.empty())
            {
                collisionValidation_->addObstacle(*cit);
            }
            std::cout << "adding obstacle to limb validation " <<(*cit)->name() << std::endl;
            limbcollisionValidation_->addObstacle(*cit);
            //remove effector collision
            if(disableEffectorCollision)
            {
                hpp::tools::RemoveEffectorCollision<hpp::core::CollisionValidation>((*collisionValidation_.get()), limb->effector_, *cit);
                hpp::tools::RemoveEffectorCollision<hpp::core::CollisionValidation>((*limbcollisionValidation_.get()), limb->effector_, *cit);
            }
        }*/
        if(nonContactingLimb)
          nonContactingLimbs_.insert(std::make_pair(id, limb));
        else
          limbs_.insert(std::make_pair(id, limb));
        //tools::RemoveNonLimbCollisionRec<core::CollisionValidation>(device_->rootJoint(),name,collisionObjects,*limbcollisionValidation_.get());
        hpp::core::RelativeMotion::matrix_type m = hpp::core::RelativeMotion::matrix(device_);
        limbcollisionValidation_->filterCollisionPairs(m);
        collisionValidation_->filterCollisionPairs(m);
        hppDout(notice,"insert limb validation with id = "<<id);
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
                                const fcl::Vec3f &offset, const fcl::Vec3f &limbOffset, const fcl::Vec3f &normal, const double x,
                                const double y,
                                const hpp::core::ObjectStdVector_t &collisionObjects, const std::size_t nbSamples, const std::string &heuristicName, const double resolution,
                                ContactType contactType, const bool disableEffectorCollision,  const bool grasp,
                                const std::string& kinematicConstraintsPath, const double kinematicConstraintsMin)
    {
        std::map<std::string, const sampling::heuristic>::const_iterator hit = checkLimbData(id, limbs_,factory_,heuristicName);
        pinocchio::JointPtr_t joint = device_->getJointByName(name);
        rbprm::RbPrmLimbPtr_t limb = rbprm::RbPrmLimb::create(joint, effectorName, offset,limbOffset,normal,x,y, nbSamples, hit->second, resolution,contactType, disableEffectorCollision, grasp,kinematicConstraintsPath,kinematicConstraintsMin);
        AddLimbPrivate(limb, id, name,collisionObjects, disableEffectorCollision);
    }

    void RbPrmFullBody::AddNonContactingLimb(const std::string& id, const std::string& name, const std::string &effectorName,
                                const hpp::core::ObjectStdVector_t &collisionObjects, const std::size_t nbSamples)
    {
        std::map<std::string, const sampling::heuristic>::const_iterator hit = checkLimbData(id, nonContactingLimbs_,factory_,"static");
        pinocchio::JointPtr_t joint = device_->getJointByName(name);
        rbprm::RbPrmLimbPtr_t limb = rbprm::RbPrmLimb::create(joint, effectorName, fcl::Vec3f(0,0,0),fcl::Vec3f(0,0,0),fcl::Vec3f(0,0,1),0,0  , nbSamples, hit->second,0.03);
        AddLimbPrivate(limb, id, name,collisionObjects, false,true);
    }

    void RbPrmFullBody::AddLimb(const std::string& database, const std::string& id,
                                const hpp::core::ObjectStdVector_t &collisionObjects,
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

    const rbprm::RbPrmLimbPtr_t RbPrmFullBody::GetLimb(std::string name,bool onlyWithContact){
      T_Limb::const_iterator lit = GetLimbs().find(std::string(name));
      if(lit == GetLimbs().end())
      {
          if(onlyWithContact){
            std::string err("No limb " + std::string(name) + " was defined for robot" + device_->name());
            throw std::runtime_error (err.c_str());
          }
          lit = GetNonContactingLimbs().find(std::string(name));
          if(lit == GetNonContactingLimbs().end())
          {
            std::string err("No limb " + std::string(name) + " was defined for robot" + device_->name());
            throw std::runtime_error (err.c_str());
          }
      }
      return lit->second;
    }

    bool RbPrmFullBody::addEffectorTrajectory(const size_t pathId,const std::string& effectorName,const std::vector<bezier_Ptr>& trajectories){
        bool success;
        if (effectorsTrajectoriesMaps_.find(pathId) == effectorsTrajectoriesMaps_.end()){
            // no map for this index, create a new one with the pair (name,path)
            EffectorTrajectoriesMap_t map;
            map.insert(std::make_pair(effectorName,trajectories));
            success = effectorsTrajectoriesMaps_.insert(std::make_pair(pathId,map)).second;
        }else{
            // there is already a trajectory at this index, we add the trajectory for this effector to the map
            success = effectorsTrajectoriesMaps_.at(pathId).insert(std::make_pair(effectorName,trajectories)).second;
        }
        return success;
    }


    bool RbPrmFullBody::addEffectorTrajectory(const size_t pathId, const std::string& effectorName, const bezier_Ptr &trajectory){
        bool success;
        if (effectorsTrajectoriesMaps_.find(pathId) == effectorsTrajectoriesMaps_.end()){
            // no map for this index, create a new one with the pair (name,path)
            EffectorTrajectoriesMap_t map;
            std::vector<bezier_Ptr> vec; vec.push_back(trajectory);
            map.insert(std::make_pair(effectorName,vec));
            success = effectorsTrajectoriesMaps_.insert(std::make_pair(pathId,map)).second;
        }else{
            // there is already a trajectory at this index, we check if there is already one for the given effector name :
            if(effectorsTrajectoriesMaps_.at(pathId).find(effectorName) == effectorsTrajectoriesMaps_.at(pathId).end()){
                // there is no trajectory for this effector name, we create a vector with the trajectory and add it
                std::vector<bezier_Ptr> vector;
                vector.push_back(trajectory);
                success = effectorsTrajectoriesMaps_.at(pathId).insert(std::make_pair(effectorName,vector)).second;
            }else{
                // there is already a trajectory with this effector, we add the new one to the vector
                effectorsTrajectoriesMaps_.at(pathId).at(effectorName).push_back(trajectory);
                success = true ; // ??
            }

        }
        return success;
    }

    bool RbPrmFullBody::getEffectorsTrajectories(const size_t pathId,EffectorTrajectoriesMap_t& result){
        if (effectorsTrajectoriesMaps_.find(pathId) == effectorsTrajectoriesMaps_.end())
            return false;
        result = effectorsTrajectoriesMaps_.at(pathId);
        return true;
    }

    bool RbPrmFullBody::getEffectorTrajectory(const size_t pathId, const std::string& effectorName, std::vector<bezier_Ptr> &result){
        if (effectorsTrajectoriesMaps_.find(pathId) == effectorsTrajectoriesMaps_.end())
            return false;
        EffectorTrajectoriesMap_t map = effectorsTrajectoriesMaps_.at(pathId);
        if (map.find(effectorName) == map.end())
            return false;
        result = map.at(effectorName);
        return true;
    }

    void RbPrmFullBody::referenceConfig(pinocchio::Configuration_t referenceConfig)
    {
        reference_ = referenceConfig;
        //create transform of the freeflyer in the world frame :
        fcl::Transform3f tRoot;
        fcl::Transform3f tJoint_world,tJoint_robot;
        tRoot.setTranslation(fcl::Vec3f(referenceConfig.head<3>()));
        fcl::Quaternion3f quatRoot(referenceConfig[3],referenceConfig[4],referenceConfig[5],referenceConfig[6]);
        tRoot.setQuatRotation(quatRoot);
        hppDout(notice,"reference root transform : "<<tRoot.getTranslation() <<" ; " <<tRoot.getRotation() );
        // retrieve transform of each effector joint
        device_->currentConfiguration(referenceConfig);
        device_->computeForwardKinematics();
        if (limbs_.empty())
            hppDout(warning,"No limbs found when setting reference configuration.");
        for(CIT_Limb lit = limbs_.begin() ; lit != limbs_.end() ; ++lit){
            hpp::pinocchio::Transform3f tf = lit->second->effector_.currentTransformation();
            tJoint_world = fcl::Transform3f(tf.rotation(),tf.translation());
            hppDout(notice,"tJoint of "<<lit->first<<" : "<<tJoint_world.getTranslation() <<" ; " <<tJoint_world.getRotation() );
            tJoint_robot = tRoot.inverseTimes(tJoint_world);
            hppDout(notice,"tJoint relative : "<<tJoint_robot.getTranslation() <<" ; " <<tJoint_robot.getRotation() );
            lit->second->effectorReferencePosition_ = tJoint_robot.getTranslation();
        }
    }




    void RbPrmFullBody::init(const RbPrmFullBodyWkPtr_t& weakPtr)
    {
        weakPtr_ = weakPtr;
    }

    RbPrmFullBody::RbPrmFullBody (const pinocchio::DevicePtr_t& device)
        : device_(device)
        , collisionValidation_(core::CollisionValidation::create(device))
        , staticStability_(true)
        , mu_(0.5)
        , reference_(device_->neutralConfiguration())
        , effectorsTrajectoriesMaps_()
        , weakPtr_()
    {
        // NOTHING
    }
  } // rbprm
} //hpp
