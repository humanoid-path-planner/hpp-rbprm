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

#ifndef HPP_RBPRM_SAMPLE_HH
# define HPP_RBPRM_SAMPLE_HH

#include <hpp/rbprm/config.hh>
#include <hpp/model/device.hh>

#include <deque>
namespace hpp {

  namespace rbprm {
  namespace sampling{
    HPP_PREDEF_CLASS(Sample);

    /// Sample configuration for a robot limb, stored
    /// in an octree and used for proximity requests for contact creation.
    /// assumes that joints are compact, ie they all are consecutive in configuration.
    class Sample;
    typedef boost::shared_ptr <Sample> SamplePtr_t;

    class HPP_RBPRM_DLLAPI Sample
    {
    public:
        /// Creates a sample configuration, given the current configuration of a limb.
        /// the current Configuration_t of the limb will be used to compute the sample.
        /// \param limb root of the considered limb
        /// \param effector joint to be considered as the effector of the limb
        /// \param offset location of the contact point of the effector, relatively to the effector joint
        /// \param id optional identifier for the sample
        Sample(const model::JointPtr_t limb, const model::JointPtr_t effector, const fcl::Vec3f& offset = fcl::Vec3f(0,0,0),  const std::size_t id =0);

        /// Creates sample configuration for a limb, extracted from a complete robot configuration, passed as a parameter
        /// \param limb root of the considered limb
        /// \param the configuration from which the limb sample will be extracted
        /// \param effector joint to be considered as the effector of the limb
        /// \param offset location of the contact point of the effector, relatively to the effector joint
        /// \param id optional identifier for the sample
        Sample(const model::JointPtr_t limb, const model::JointPtr_t effector, model::ConfigurationIn_t configuration, const fcl::Vec3f& offset = fcl::Vec3f(0,0,0), const std::size_t id =0);
        Sample(const Sample &clone);
       ~Sample(){}

    public:
      const std::size_t startRank_;
      const std::size_t length_;
      const model::Configuration_t configuration_;
      /// Position relative to robot root (ie, robot base at 0 everywhere)
      const fcl::Vec3f effectorPosition_;
      const Eigen::MatrixXd jacobian_;
      /// Product of the jacobian by its transpose
      const Eigen::Matrix <model::value_type, 6, 6> jacobianProduct_;
      /// id in sample container
      const std::size_t id_;
      const double manipulability_;
      //const fcl::Transform3f rotation_; TODO
    }; // class Sample

/// Automatically generates a deque of sample configuration for a given limb of a robot
    /// \param limb root of the considered limb
    /// \param effector tag identifying the end effector of the limb
    /// \param nbSamples number of samples to be generated
    /// \param offset location of the contact point of the effector relatively to the effector joint origin
    /// \return a deque of sample configurations respecting joint limits.
std::deque<Sample> GenerateSamples(const model::JointPtr_t limb,  const std::string& effector,  const std::size_t nbSamples,const fcl::Vec3f& offset = fcl::Vec3f(0,0,0));

/// Assigns the limb configuration associated with a sample to a robot configuration
/// \param sample The limb configuration to load
/// \param robot the configuration to be modified
void Load(const Sample& sample, model::ConfigurationOut_t robot);

  } // namespace sampling
} // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_SAMPLE_HH
