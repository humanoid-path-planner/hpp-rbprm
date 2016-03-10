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
# define HPP_RBPRM_FULLBODY_HH

#include <hpp/rbprm/config.hh>
#include <hpp/rbprm/rbprm-state.hh>
#include <hpp/model/device.hh>
#include <hpp/rbprm/rbprm-limb.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/rbprm/sampling/heuristic.hh>

#include  <vector>

namespace hpp {
  namespace rbprm {

    HPP_PREDEF_CLASS(RbPrmFullBody);

    /// Encapsulation of a Device class to handle the generation of contacts
    /// configurations for the user defined limbs of the Device.
    /// Uses an internal representation for the limbs, and handles
    /// collisions and balance criteria for generating their subconfigurations.
    ///
    class RbPrmFullBody;
    typedef boost::shared_ptr <RbPrmFullBody> RbPrmFullBodyPtr_t;

    class HPP_RBPRM_DLLAPI RbPrmFullBody
    {
    public:
        static RbPrmFullBodyPtr_t create (const model::DevicePtr_t& device);

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
        /// of the unit voxel of the octree. The larger they are, the more samples will be considered as candidates for contact.
        /// This can be problematic in terms of performance. The default value is 3 cm.
        void AddLimb(const std::string& id, const std::string& name, const std::string& effectorName, const fcl::Vec3f &offset,
                     const fcl::Vec3f &normal,const double x, const double y,
                     const model::ObjectVector_t &collisionObjects,
                     const std::size_t nbSamples, const std::string& heuristic = "static", const double resolution = 0.03,
                     ContactType contactType = _6_DOF);

        /// Add a new heuristic for biasing sample candidate selection
        ///
        /// \param name: name used to identify the heuristic
        /// \param func the actual heuristic method
        /// \return whether the heuristic has been added. False is returned if a heuristic with that name already exists.
        bool AddHeuristic(const std::string& name, const sampling::heuristic func);

    public:
        typedef std::map<std::string, std::vector<std::string> > T_LimbGroup;

    public:
        const rbprm::T_Limb& GetLimbs() {return limbs_;}
        const T_LimbGroup& GetGroups() {return limbGroups_;}
        const model::DevicePtr_t device_;

    private:
        core::CollisionValidationPtr_t collisionValidation_;
        std::map<std::string, core::CollisionValidationPtr_t> limbcollisionValidations_;
        rbprm::T_Limb limbs_;
        T_LimbGroup limbGroups_;
        sampling::HeuristicFactory factory_;


    protected:
      RbPrmFullBody (const model::DevicePtr_t &device);

      ///
      /// \brief Initialization.
      ///
      void init (const RbPrmFullBodyWkPtr_t& weakPtr);

    private:
      RbPrmFullBodyWkPtr_t weakPtr_;
      friend hpp::rbprm::State HPP_RBPRM_DLLAPI ComputeContacts(const hpp::rbprm::RbPrmFullBodyPtr_t& body, model::ConfigurationIn_t configuration,
                                        const model::ObjectVector_t& collisionObjects, const fcl::Vec3f& direction, const double robustnessTreshold);

      friend hpp::rbprm::State HPP_RBPRM_DLLAPI ComputeContacts(const hpp::rbprm::State& previous, const hpp::rbprm::RbPrmFullBodyPtr_t& body, model::ConfigurationIn_t configuration,
                                        const model::ObjectVector_t& collisionObjects, const fcl::Vec3f& direction, bool& contactMaintained, bool& multipleBreaks, const bool allowFailure, const double robustnessTreshold);
    }; // class RbPrmDevice

    /// Generates a balanced contact configuration, considering the
    /// given current configuration of the robot, and a direction of motion.
    /// Typically used to generate a start and / or goal configuration automatically for a planning problem.
    ///
    /// \param body The considered FullBody robot for which to generate contacts
    /// \param configuration Current full body configuration.
    /// \param collisionObjects the set of 3D objects to consider for collision and contact creation.
    /// \param direction An estimation of the direction of motion of the character.
    /// \param robustnessTreshold minimum value of the static equilibrium robustness criterion required to accept the configuration (0 by default).
    /// \return a State describing the computed contact configuration, with relevant contact information and balance information.
    hpp::rbprm::State HPP_RBPRM_DLLAPI ComputeContacts(const hpp::rbprm::RbPrmFullBodyPtr_t& body, model::ConfigurationIn_t configuration,
                                      const model::ObjectVector_t& collisionObjects, const fcl::Vec3f& direction,
                                                       const double robustnessTreshold = 0);

    /// Generates a balanced contact configuration, considering the
    /// given current configuration of the robot, and a previous, balanced configuration.
    /// Existing contacts are maintained provided joint limits and balance remains respected.
    /// Otherwise a contact generation strategy is employed.
    ///
    /// \param previous The previously computed State of the robot
    /// \param body The considered FullBody robot for which to generate contacts
    /// \param configuration Current full body configuration.
    /// \param collisionObjects the set of 3D objects to consider for collision and contact creation.
    /// \param direction An estimation of the direction of motion of the character.
    /// \param contactMaintained parameter set to true if all the contacts were maintained, regarding the previous state
    /// \param multipleBreaks If the contact generation failed at this stage because multiple contacts were broken, is set to true.
    /// \param allowFailure allow multiple breaks in the contact computation.
    /// \param robustnessTreshold minimum value of the static equilibrium robustness criterion required to accept the configuration (0 by default).
    /// \return a State describing the computed contact configuration, with relevant contact information and balance information.
    hpp::rbprm::State HPP_RBPRM_DLLAPI ComputeContacts(const hpp::rbprm::State& previous, const hpp::rbprm::RbPrmFullBodyPtr_t& body, model::ConfigurationIn_t configuration,
                                            const model::ObjectVector_t& collisionObjects, const fcl::Vec3f& direction, bool& contactMaintained, bool& multipleBreaks, const bool allowFailure,
                                            const double robustnessTreshold = 0);
  } // namespace rbprm

} // namespace hpp

#endif // HPP_RBPRM_DEVICE_HH
