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
    /// assumes that joints are compact, ie they all are consecutive in configuration
    class Sample;
    typedef boost::shared_ptr <Sample> SamplePtr_t;

    class HPP_RBPRM_DLLAPI Sample
    {
    public:
        /// Creates sample from Configuration
        /// in presented joint
        /// \param limb root of the considered limb
        /// the Configuration_t of this limb will be used to compute the sample
        Sample(const model::JointPtr_t limb);
        /// Creates sample from Configuration
        /// in presented joint
        /// \param limb root of the considered limb
        /// \param configuration used to compute the sample
        Sample(const model::JointPtr_t limb, const model::Configuration_t& configuration);
        Sample(const Sample &clone);
       ~Sample(){}

    public:
      const std::size_t startRank_;
      const std::size_t length_;
      const model::Configuration_t configuration_;
      /// Position relative to robot root (ie, robot base at 0 everywhere)
      const fcl::Vec3f effectorPosition_;
      const Eigen::MatrixXd jacobian_;
      const Eigen::Matrix <model::value_type, 6, 6> jacobianProduct_;
      //const fcl::Transform3f rotation_; TODO
    }; // class Sample

std::deque<Sample> GenerateSamples(const model::JointPtr_t model,  const std::size_t nbSamples);
/// LoadSample into robot
void Load(const Sample& sample, model::Configuration_t& robot);

  } // namespace sampling
} // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_SAMPLE_HH
