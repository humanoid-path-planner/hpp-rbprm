//
// Copyright (c) 2014 CNRS
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
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_RBPRM_TOOLS_HH
# define HPP_RBPRM_TOOLS_HH

# include <hpp/core/config-validation.hh>
# include <hpp/model/joint.hh>
# include <hpp/rbprm/config.hh>
# include <hpp/model/collision-object.hh>
# include <Eigen/Core>

namespace hpp {
  namespace tools {
  /// Uses Rodriguez formula to find transformation between two vectors.
  Eigen::Matrix3d GetRotationMatrix(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
  fcl::Matrix3f GetRotationMatrix(const fcl::Vec3f& from, const fcl::Vec3f& to);
  fcl::Matrix3f GetZRotMatrix(const core::value_type theta);
  fcl::Matrix3f GetYRotMatrix(const core::value_type theta);
  fcl::Matrix3f GetXRotMatrix(const core::value_type theta);
  model::Configuration_t interpolate(model::ConfigurationIn_t q1, model::ConfigurationIn_t q2, const model::value_type& u);
  model::value_type distance (model::ConfigurationIn_t q1, model::ConfigurationIn_t q2);

  template<typename K, typename V>
  void addToMap(const K& key, const V& value);

  template<typename T>
  bool insertIfNew(std::vector<T>& data, const T& value);

  template<typename T>
  void RemoveEffectorCollision(T& validation, model::JointPtr_t effectorJoint, const model::ObjectVector_t& obstacles);
  template<typename T>
  void RemoveEffectorCollision(T& validation, model::JointPtr_t effectorJoint, const model::CollisionObjectPtr_t obstacle);
  template<typename T>
  void RemoveEffectorCollisionRec(T& validation, model::JointPtr_t joint, const model::CollisionObjectPtr_t obstacle);

  ///Lock all joints in a kinematic chain, except for one joint and its subchain
  /// \param spared Name of the root of the unlocked kinematic chain
  /// \param joint Root of the considered kinematic chain to block
  /// \param projector Projector on which to block the joints
  void LockJointRec(const std::string& spared, const model::JointPtr_t joint, core::ConfigProjectorPtr_t& projector);

  ///Lock all joints in a kinematic chain, except for a list of subchains
  /// \param spared names of the root of the unlocked kinematic chains
  /// \param joint Root of the considered kinematic chain to block
  /// \param projector Projector on which to block the joints
  void LockJointRec(const std::vector<std::string>& spared, const model::JointPtr_t joint, core::ConfigProjectorPtr_t& projector);

  ///Lock a single joint
  /// \param joint of the considered kinematic chain to block
  /// \param projector Projector on which to block the joints
  /// \param constant if false, joint lock constraint can be updated with rightHandSide method
  void LockJoint(const model::JointPtr_t joint, core::ConfigProjectorPtr_t& projector, const bool constant=true);

  ///Some io tools for serialization
  namespace io
  {
      double StrToD (const std::string &str);
      int StrToI (const std::string &str);
      double StrToD (std::ifstream& input);
      int    StrToI (std::ifstream& input);
      std::vector<std::string> splitString(const std::string& s, const char sep);
      void writeMatrix      (const Eigen::MatrixXd& mat, std::ostream& output);
      void writeVecFCL      (const fcl::Vec3f& vec     , std::ostream& output);
      void writeRotMatrixFCL(const fcl::Matrix3f& mat  , std::ostream& output);
      Eigen::MatrixXd readMatrix      (std::ifstream& myfile);
      fcl::Matrix3f   readRotMatrixFCL(std::ifstream& myfile);
      fcl::Vec3f      readVecFCL      (std::ifstream& myfile);
      Eigen::MatrixXd readMatrix      (std::ifstream& myfile, std::string& line);
      fcl::Matrix3f   readRotMatrixFCL(std::ifstream& myfile, std::string& line);
      fcl::Vec3f      readVecFCL      (std::ifstream& myfile, std::string& line);
  } // namespace io

  template<typename T>
  void RemoveEffectorCollisionRec(T& validation, model::JointPtr_t joint, const model::CollisionObjectPtr_t obstacle)
  {
      try
      {
        validation.removeObstacleFromJoint(joint,obstacle);
      }
      catch(const std::runtime_error& e)
      {
          std::cout << "WARNING: " << e.what() << std::endl;
          return;
      }
      //then sons
      for(std::size_t i =0; i < joint->numberChildJoints(); ++i)
      {
          RemoveEffectorCollisionRec<T>(validation, joint->childJoint(i), obstacle);
      }
  }

  template<typename T>
  void RemoveEffectorCollision(T& validation, model::JointPtr_t effectorJoint, const hpp::model::ObjectVector_t &obstacles)
  {
      for(hpp::model::ObjectVector_t::const_iterator cit = obstacles.begin();
          cit != obstacles.end(); ++cit)
      {
          RemoveEffectorCollision<T>(validation,effectorJoint,*cit);
      }
  }

  template<typename T>
  void RemoveEffectorCollision(T& validation, model::JointPtr_t effectorJoint, const model::CollisionObjectPtr_t obstacle)
  {
      try
      {
          //remove actual effector or not ?
          validation.removeObstacleFromJoint(effectorJoint,obstacle);
      }
      catch(const std::runtime_error& e)
      {
          std::cout << "WARNING: " << e.what() << std::endl;
          return;
      }
      //then sons
      for(std::size_t i =0; i < effectorJoint->numberChildJoints(); ++i)
      {
          RemoveEffectorCollisionRec<T>(validation, effectorJoint->childJoint(i), obstacle);
      }
  }


  template<typename T>
  void RemoveNonLimbCollisionRec(const model::JointPtr_t joint, const std::string& limbname,
                                 const model::ObjectVector_t &collisionObjects,
                                 T& collisionValidation)
  {
      if(joint->name() == limbname) return;
      for(model::ObjectVector_t::const_iterator cit = collisionObjects.begin();
          cit != collisionObjects.end(); ++cit)
      {
          try
          {
              collisionValidation.removeObstacleFromJoint(joint, *cit);
          }
          catch(const std::runtime_error& e)
          {
              std::cout << "WARNING: "<< e.what() << std::endl;
              return;
          }
      }
      for(std::size_t i=0; i<joint->numberChildJoints(); ++i)
      {
          RemoveNonLimbCollisionRec(joint->childJoint(i), limbname, collisionObjects, collisionValidation);
      }
  }


  template<typename T>
  void RemoveNonLimbCollisionRec(const model::JointPtr_t joint, const std::vector<std::string>& keepers,
                                 const model::ObjectVector_t &collisionObjects,
                                 T& collisionValidation)
  {
      if(std::find(keepers.begin(), keepers.end(), joint->name()) != keepers.end()) return;
      for(model::ObjectVector_t::const_iterator cit = collisionObjects.begin();
          cit != collisionObjects.end(); ++cit)
      {
          try
          {
              collisionValidation.removeObstacleFromJoint(joint, *cit);
          }
          catch(const std::runtime_error& e)
          {
              std::cout << "WARNING: "<< e.what() << std::endl;
              return;
          }
      }
      for(std::size_t i=0; i<joint->numberChildJoints(); ++i)
      {
          RemoveNonLimbCollisionRec(joint->childJoint(i), keepers, collisionObjects, collisionValidation);
      }
  }


  template<typename K, typename V>
  void addToMap(std::map<K,V>& map, const K& key, const V& value)
  {
        if(map.find(key) != map.end())
            map[key] = value;
        else
            map.insert(std::make_pair(key,value));
  }

  template<typename T>
  bool insertIfNew(std::vector<T>& data, const T& value)
  {
      if(std::find(data.begin(), data.end(), value) == data.end())
      {
          data.push_back(value);
          return true;
      }
      return false;
  }


  } // namespace tools
} // namespace hpp

#endif // HPP_RBPRM_TOOLS_HH
