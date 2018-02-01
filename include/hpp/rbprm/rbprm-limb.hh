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

#ifndef HPP_RBPRM_LIMB_HH
# define HPP_RBPRM_LIMB_HH

# include <hpp/rbprm/config.hh>
# include <hpp/rbprm/sampling/sample-db.hh>
# include <hpp/rbprm/sampling/heuristic.hh>
# include <hpp/model/device.hh>

namespace hpp {
  namespace rbprm {

  enum ContactType
    {
      _6_DOF  = 0,  // Position and orientation are considered (surface contact)
      _3_DOF  = 1,  // Only 3D position is considered (punctual contact)
      _UNDEFINED
    };


    HPP_PREDEF_CLASS(RbPrmLimb);

    /// Representation of a robot limb.
    /// Contains a SampleDB used for computing contact candidates
    ///
    class RbPrmLimb;
    typedef boost::shared_ptr <RbPrmLimb> RbPrmLimbPtr_t;
    typedef std::map<std::string, const rbprm::RbPrmLimbPtr_t > T_Limb;
    typedef T_Limb::const_iterator CIT_Limb;
    typedef Eigen::Matrix <model::value_type, Eigen::Dynamic, Eigen::Dynamic> MatrixXX;
    typedef Eigen::Matrix <model::value_type, Eigen::Dynamic, 3>              MatrixX3;
    typedef Eigen::Matrix <model::value_type, Eigen::Dynamic, 1>              VectorX;


    class HPP_RBPRM_DLLAPI RbPrmLimb
    {
    public:
        /// Creates a Limb a Fullbody instance  Stores a sample
        /// container, used for proximity requests. The Effector is considered as the last joint
        /// of the depth first search on the kinematic subchain.
        ///
        /// \param limb Joint instance that serves as a root for the limb
        /// \param effectorName name of the joint to be considered as the effector of the limb
        /// \param offset position of the effector in joint coordinates relatively to the effector joint
        /// \param unit normal vector of the contact point, expressed in the effector joint coordinates
        /// \param x width of the default support polygon of the effector
        /// \param y height of the default support polygon of the effector
        /// \param nbSamples number of samples to generate for the limb
        /// \param evaluate heuristic method used to bias sample generation
        /// \param resolution, resolution of the octree voxels. The samples generated are stored in an octree data
        /// structure to perform efficient proximity requests. The resulution of the octree, in meters, specifies the size
        /// of the unit voxel of the octree. The larger they are, the more samples will be considered as candidates for contact.
        /// This can be problematic in terms of performance. The default value is 3 cm.
        /// \param contactType Whether the contact is a surface contact (orientation matters) or a punctual contact
        /*static RbPrmLimbPtr_t create (const model::JointPtr_t limb, const fcl::Vec3f &offset,
                                      const fcl::Vec3f &normal,const double x, const double y,
                                      const std::size_t nbSamples, const sampling::heuristic evaluate = 0,
                                      const double resolution = 0.1, ContactType contactType = _6_DOF);*/


        /// Creates a Limb a Fullbody instance  Stores a sample
        /// container, used for proximity requests.
        ///
        /// \param limb Joint instance that serves as a root for the limb
        /// \param offset position of the effector in joint coordinates relatively to the effector joint
        /// \param limbOffset is the offset between the limb_ joint position and it's link
        /// \param unit normal vector of the contact point, expressed in the effector joint coordinates
        /// \param x width of the default support polygon of the effector
        /// \param y height of the default support polygon of the effector
        /// \param nbSamples number of samples to generate for the limb
        /// \param evaluate heuristic method used to bias sample generation
        /// \param resolution, resolution of the octree voxels. The samples generated are stored in an octree data
        /// structure to perform efficient proximity requests. The resulution of the octree, in meters, specifies the size
        /// of the unit voxel of the octree. The larger they are, the more samples will be considered as candidates for contact.
        /// This can be problematic in terms of performance. The default value is 3 cm.
        /// \param contactType Whether the contact is a surface contact (orientation matters) or a punctual contact
        /// \param disableEndEffectorCollision Whether the end effector bodies should be counted for collision detection
        static RbPrmLimbPtr_t create (const model::JointPtr_t limb, const std::string& effectorName, const fcl::Vec3f &offset,
                                      const fcl::Vec3f &limbOffset, const fcl::Vec3f &normal,const double x, const double y,
                                      const std::size_t nbSamples, const sampling::heuristic evaluate = 0,
                                      const double resolution = 0.1, ContactType contactType = _6_DOF,
                                      bool disableEndEffectorCollision = false,
                                      bool grasps = false);

        static RbPrmLimbPtr_t create (const model::DevicePtr_t device, std::ifstream& fileStream, const bool loadValues = true,
                                      const hpp::rbprm::sampling::heuristic evaluate = 0,
                                      bool disableEndEffectorCollision = false,
                                      bool grasps = false);

    public:
        ~RbPrmLimb();

    public:
        fcl::Transform3f octreeRoot() const;

    public:
        const model::JointPtr_t limb_;
        const model::JointPtr_t effector_;
        const fcl::Matrix3f effectorDefaultRotation_; // effector transform in rest pose
        const fcl::Vec3f offset_; // effector location
        const fcl::Vec3f limbOffset_; // offset of between the limb_ joint position and it's link
        const fcl::Vec3f normal_; // effector normal for surface
        const double x_; // half width
        const double y_; // half length of contact surface
        const ContactType contactType_;
        sampling::heuristic evaluate_;
        sampling::SampleDB sampleContainer_;
        const bool disableEndEffectorCollision_;
        const bool grasps_;
        fcl::Vec3f effectorReferencePosition_;
        std::pair<MatrixXX, MatrixXX> kinematicConstraints_;

    protected:

      RbPrmLimb (const model::JointPtr_t& limb, const std::string& effectorName,  const fcl::Vec3f &offset,
                 const fcl::Vec3f &limbOffset, const fcl::Vec3f &normal,const double x, const double y,
                 const std::size_t nbSamples, const sampling::heuristic evaluate,
                 const double resolution, ContactType contactType,
                 bool disableEndEffectorCollision = false,
                 bool grasps = false);

      RbPrmLimb (const model::DevicePtr_t device, std::ifstream& fileStream, const bool loadValues,
                 const hpp::rbprm::sampling::heuristic evaluate,
                 bool disableEndEffectorCollision = false,
                 bool grasps = false);
      ///
      /// \brief Initialization.
      ///
      void init (const RbPrmLimbWkPtr_t& weakPtr);

    private:
      RbPrmLimbWkPtr_t weakPtr_;
    }; // class RbPrmLimb


    HPP_RBPRM_DLLAPI bool saveLimbInfoAndDatabase(const RbPrmLimbPtr_t limb, std::ofstream& dbFile);

  } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_LIMB_HH
