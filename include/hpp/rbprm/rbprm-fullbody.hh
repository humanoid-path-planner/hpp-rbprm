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

#ifndef HPP_RBPRM_FULLBODY_HH
#define HPP_RBPRM_FULLBODY_HH

#include <hpp/rbprm/config.hh>
#include <hpp/rbprm/rbprm-state.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/rbprm/rbprm-limb.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/rbprm/sampling/heuristic.hh>
#include <hpp/rbprm/reports.hh>
#include <hpp/rbprm/interpolation/spline/bezier-path.hh>
#include <vector>

namespace hpp {
namespace rbprm {

using core::size_type;

HPP_PREDEF_CLASS(RbPrmFullBody);

/// Encapsulation of a Device class to handle the generation of contacts
/// configurations for the user defined limbs of the Device.
/// Uses an internal representation for the limbs, and handles
/// collisions and balance criteria for generating their subconfigurations.
///
class RbPrmFullBody;
typedef std::shared_ptr<RbPrmFullBody> RbPrmFullBodyPtr_t;
typedef hpp::core::Container<hpp::core::AffordanceObjects_t> affMap_t;
typedef std::map<std::string, std::vector<bezier_Ptr> > EffectorTrajectoriesMap_t;

class HPP_RBPRM_DLLAPI RbPrmFullBody {
 public:
  static RbPrmFullBodyPtr_t create(const pinocchio::DevicePtr_t& device);

 public:
  virtual ~RbPrmFullBody();

 public:
  /// Creates a Limb for the robot,
  /// identified by its name. Stores a sample
  /// container, used for requests
  ///
  /// \param id: user defined id for the limb. Must be unique.
  /// The id is used if several contact points are defined for the same limb (ex: the knee and the foot)
  /// \param name: name of the joint corresponding to the root of the limb.
  /// \param effectorName name of the joint to be considered as the effector of the limb
  /// \param offset position of the effector in joint coordinates relatively to the effector joint
  /// \param unit normal vector of the contact point, expressed in the effector joint coordinates
  /// \param x width of the default support polygon of the effector
  /// \param y height of the default support polygon of the effector
  /// \param collisionObjects objects to be considered for collisions with the limb. TODO remove
  /// \param nbSamples number of samples to generate for the limb
  /// \param resolution, resolution of the octree voxels. The samples generated are stored in an octree data
  /// structure to perform efficient proximity requests. The resulution of the octree, in meters, specifies the size
  /// of the unit voxel of the octree. The larger they are, the more samples will be considered as candidates for
  /// contact. This can be problematic in terms of performance. The default value is 3 cm. \param resolution,
  /// resolution of the octree voxels. The samples generated are stored in an octree data \param
  /// disableEffectorCollision, whether collision detection should be disabled for end effector bones
  void AddLimb(const std::string& id, const std::string& name, const std::string& effectorName,
               const fcl::Vec3f& offset, const fcl::Vec3f& limbOffset, const fcl::Vec3f& normal, const double x,
               const double y, const core::ObjectStdVector_t& collisionObjects, const std::size_t nbSamples,
               const std::string& heuristic = "static", const double resolution = 0.03,
               ContactType contactType = _6_DOF, const bool disableEffectorCollision = false, const bool grasp = false,
               const std::string& kinematicConstraintsPath = std::string(), const double kinematicConstraintsMin = 0.);

  /// Creates a Limb for the robot,
  /// identified by its name. Stores a sample
  /// container, used for requests
  ///
  /// \param database: path to the sample database used for the limbs
  /// \param id: user defined id for the limb. Must be unique.
  /// The id is used if several contact points are defined for the same limb (ex: the knee and the foot)
  /// \param collisionObjects objects to be considered for collisions with the limb. TODO remove
  /// structure to perform efficient proximity requests. The resulution of the octree, in meters, specifies the size
  /// of the unit voxel of the octree. The larger they are, the more samples will be considered as candidates for
  /// contact. This can be problematic in terms of performance. The default value is 3 cm. \param
  /// disableEffectorCollision, whether collision detection should be disabled for end effector bones
  void AddLimb(const std::string& database, const std::string& id, const core::ObjectStdVector_t& collisionObjects,
               const std::string& heuristicName, const bool loadValues, const bool disableEffectorCollision = false,
               const bool grasp = false);

  ///
  /// \brief AddNonContactingLimb add a limb not used for contact generation
  /// \param id: user defined id for the limb. Must be unique.
  /// The id is used if several contact points are defined for the same limb (ex: the knee and the foot)
  /// \param name: name of the joint corresponding to the root of the limb.
  /// \param effectorName name of the joint to be considered as the effector of the limb
  /// \param collisionObjects objects to be considered for collisions with the limb. TODO remove
  /// \param nbSamples number of samples to generate for the limb
  void AddNonContactingLimb(const std::string& id, const std::string& name, const std::string& effectorName,
                            const hpp::core::ObjectStdVector_t& collisionObjects, const std::size_t nbSamples);

  /// Add a new heuristic for biasing sample candidate selection
  ///
  /// \param name: name used to identify the heuristic
  /// \param func the actual heuristic method
  /// \return whether the heuristic has been added. False is returned if a heuristic with that name already exists.
  bool AddHeuristic(const std::string& name, const sampling::heuristic func);

 public:
  typedef std::map<std::string, std::vector<std::string> > T_LimbGroup;

 public:
  const rbprm::T_Limb& GetLimbs() { return limbs_; }
  const rbprm::RbPrmLimbPtr_t GetLimb(std::string name, bool onlyWithContact = false);
  const rbprm::T_Limb& GetNonContactingLimbs() { return nonContactingLimbs_; }
  const T_LimbGroup& GetGroups() { return limbGroups_; }
  const core::CollisionValidationPtr_t& GetCollisionValidation() { return collisionValidation_; }
  const std::map<std::string, core::CollisionValidationPtr_t>& GetLimbCollisionValidation() {
    return limbcollisionValidations_;
  }
  const pinocchio::DevicePtr_t device_;
  void staticStability(bool staticStability) { staticStability_ = staticStability; }
  bool staticStability() const { return staticStability_; }
  double getFriction() const { return mu_; }
  void setFriction(double mu) { mu_ = mu; }
  pinocchio::Configuration_t referenceConfig() { return reference_; }
  void referenceConfig(pinocchio::Configuration_t referenceConfig);
  pinocchio::Configuration_t postureWeights() { return postureWeights_; }
  void postureWeights(pinocchio::Configuration_t postureWeights);
  bool usePosturalTaskContactCreation() { return usePosturalTaskContactCreation_; }
  void usePosturalTaskContactCreation(bool usePosturalTaskContactCreation) {
    usePosturalTaskContactCreation_ = usePosturalTaskContactCreation;
  }
  bool addEffectorTrajectory(const size_t pathId, const std::string& effectorName, const bezier_Ptr& trajectory);
  bool addEffectorTrajectory(const size_t pathId, const std::string& effectorName,
                             const std::vector<bezier_Ptr>& trajectories);
  bool getEffectorsTrajectories(const size_t pathId, EffectorTrajectoriesMap_t& result);
  bool getEffectorTrajectory(const size_t pathId, const std::string& effectorName, std::vector<bezier_Ptr>& result);
  bool toggleNonContactingLimb(std::string name);

 private:
  core::CollisionValidationPtr_t collisionValidation_;
  std::map<std::string, core::CollisionValidationPtr_t> limbcollisionValidations_;
  rbprm::T_Limb limbs_;
  rbprm::T_Limb nonContactingLimbs_;  // this is the list of limbs that are not used during contact generation.
  T_LimbGroup limbGroups_;
  sampling::HeuristicFactory factory_;
  bool staticStability_;
  double mu_;
  pinocchio::Configuration_t reference_;
  pinocchio::Configuration_t postureWeights_;  // weight used to compute the distance in the postural tasks
  bool usePosturalTaskContactCreation_;  // if true, during the contact creation the orientation of the feet along the
                                         // contact normal is optimized for a postural task
  std::map<size_t, EffectorTrajectoriesMap_t>
      effectorsTrajectoriesMaps_;  // the map link the pathIndex (the same as in the wholeBody paths in problem solver)
                                   // to a map of trajectories for each effectors.
 private:
  void AddLimbPrivate(rbprm::RbPrmLimbPtr_t limb, const std::string& id, const std::string& name,
                      const hpp::core::ObjectStdVector_t& collisionObjects, const bool disableEffectorCollision,
                      const bool nonContactingLimb = false);

 protected:
  RbPrmFullBody(const pinocchio::DevicePtr_t& device);

  ///
  /// \brief Initialization.
  ///
  void init(const RbPrmFullBodyWkPtr_t& weakPtr);

 private:
  RbPrmFullBodyWkPtr_t weakPtr_;
};  // class RbPrmDevice
}  // namespace rbprm

}  // namespace hpp

#endif  // HPP_RBPRM_DEVICE_HH
