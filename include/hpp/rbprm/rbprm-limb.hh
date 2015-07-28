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
# include <hpp/rbprm/sampling/sample-container.hh>
# include <hpp/model/device.hh>

namespace hpp {
  namespace rbprm {
    HPP_PREDEF_CLASS(RbPrmLimb);

    /// Representation of a robot limb.
    /// Contains a SampleContainer used for computing contact candidates
    ///
    class RbPrmLimb;
    typedef boost::shared_ptr <RbPrmLimb> RbPrmLimbPtr_t;
    typedef std::map<std::string, const rbprm::RbPrmLimbPtr_t > T_Limb;

    class HPP_RBPRM_DLLAPI RbPrmLimb
    {
    public:
        static RbPrmLimbPtr_t create (const model::JointPtr_t limb, const fcl::Vec3f &offset,
                                      const fcl::Vec3f &normal,const double x, const double y,
                                      const std::size_t nbSamples, const double resolution);

        static RbPrmLimbPtr_t create (const model::JointPtr_t limb, const std::string& effectorName, const fcl::Vec3f &offset,
                                      const fcl::Vec3f &normal,const double x, const double y,
                                      const std::size_t nbSamples, const double resolution);

    public:
        ~RbPrmLimb();

    public:
        const model::JointPtr_t limb_;
        const model::JointPtr_t effector_;
        const fcl::Matrix3f effectorDefaultRotation_; // effector transform in rest pose
        const sampling::SampleContainer sampleContainer_;
        const fcl::Vec3f offset_; // effector location
        const fcl::Vec3f normal_; // effector normal for surface
        const double x_; // half width
        const double y_; // half length of contact surface

    protected:
      RbPrmLimb (const model::JointPtr_t& limb,  const fcl::Vec3f &offset,
                 const fcl::Vec3f &normal,const double x, const double y,
                 const std::size_t nbSamples, const double resolution);

      RbPrmLimb (const model::JointPtr_t& limb, const std::string& effectorName,  const fcl::Vec3f &offset,
                 const fcl::Vec3f &normal,const double x, const double y,
                 const std::size_t nbSamples, const double resolution);

      ///
      /// \brief Initialization.
      ///
      void init (const RbPrmLimbWkPtr_t& weakPtr);

    private:
      RbPrmLimbWkPtr_t weakPtr_;
    }; // class RbPrmLimb
  } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_LIMB_HH
