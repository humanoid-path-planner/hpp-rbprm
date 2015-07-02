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

#ifndef HPP_RBPRM_SAMPLE_CONTAINER_HH
# define HPP_RBPRM_SAMPLE_CONTAINER_HH

#include <hpp/rbprm/sampling/sample.hh>
#include <hpp/fcl/octree.h>

namespace hpp {

  namespace rbprm {
  namespace sampling{
    HPP_PREDEF_CLASS(SampleContainer);

    /// Sample container for a given limb of a robot.
    /// Stores a list of Sample in two ways: a deque,
    /// and an octree for spatial requests.
    /// indexes of the octree corresponds to end effector positions.
    class SampleContainer;
    typedef boost::shared_ptr <SampleContainer> SampleContainerPtr_t;

    class HPP_RBPRM_DLLAPI SampleContainer
    {
    public:
        /// Creates sample from Configuration
        /// in presented joint
        /// \param limb root joint for the considered limb
        /// \param nbSamples number of samples to generate
        /// \param resolution, resolution of the octree voxels
        SampleContainer(const model::JointPtr_t limb, const std::size_t nbSamples, const double resolution = 0.1);
       ~SampleContainer();

    private:
        SampleContainer(const SampleContainer& sContainer);

    public:
        /// samples generated
        const std::deque<Sample> samples_;
        const fcl::OcTree octree_;
        /// Bounding boxes of areas of interest of the octree
        const std::vector<fcl::CollisionObject*> boxes_;
    }; // class SampleContainer
  } // namespace sampling
} // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_SAMPLE_CONTAINER_HH
