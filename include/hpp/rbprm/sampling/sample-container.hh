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

#include <map>
#include <memory>

namespace hpp {

  namespace rbprm {
  namespace sampling{
    HPP_PREDEF_CLASS(SampleContainer);

    /// Collision report for a Sample for which the octree node is
    /// colliding with the environment.
    struct OctreeReport
    {
        OctreeReport(const Sample*, const fcl::Contact, const double, const fcl::Vec3f& normal);
        /// Sample considered for contact generation
        const Sample* sample_;
        /// Contact information returned from fcl
        fcl::Contact contact_;
        /// heuristic evaluation of the sample
        double value_;
        /// normal vector of the surface in contact
        fcl::Vec3f normal_;
    };


    /// Comparaison operator between Samples
    /// Used to sort the contact candidates depending
    /// on their heuristic value
    struct sample_compare {
        bool operator() (const OctreeReport& lhs, const OctreeReport& rhs) const{
            return lhs.value_ > rhs.value_;
        }
    };

    typedef std::set<OctreeReport, sample_compare> T_OctreeReport;

    /// Sample container for a given limb of a robot.
    /// Stores a list of Sample in two ways: a deque,
    /// and an octree for spatial requests.
    /// indexes of the octree corresponds to end effector positions.
    class SampleContainer;
    typedef boost::shared_ptr <SampleContainer> SampleContainerPtr_t;

    struct SamplePImpl;
    class HPP_RBPRM_DLLAPI SampleContainer
    {
    public:
        typedef std::map<std::size_t, std::vector<const Sample*> > T_VoxelSample;

    public:
        /// Creates Sample from Configuration
        /// in presented joint
        ///
        /// \param limb root joint for the considered limb
        /// \param effector joint to be considered as the effector of the limb
        /// \param nbSamples number of samples to generate
        /// \param offset position of the effector in joint coordinates
        /// \param resolution, resolution of the octree voxels
        SampleContainer(const model::JointPtr_t limb, const std::string& effector, const std::size_t nbSamples, const fcl::Vec3f& offset = fcl::Vec3f(0,0,0), const double resolution = 0.1);
       ~SampleContainer();

    private:
        SampleContainer(const SampleContainer& sContainer);

    public:
        /// samples generated
        const std::deque<Sample> samples_;

    private:
        /// private implementation
        std::auto_ptr<SamplePImpl> pImpl_;

    public:
        /// fcl collision object used for collisions with environment
        const fcl::CollisionObject treeObject_;
        /// Samples sorted by voxel id in the octree
        const T_VoxelSample voxelSamples_;
        /// Bounding boxes of areas of interest of the octree
        const std::vector<fcl::CollisionObject*> boxes_;

        friend bool GetCandidates(const SampleContainer& sc, const fcl::Transform3f& treeTrf,
                                                const hpp::model::CollisionObjectPtr_t& o2,
                                                const fcl::Vec3f& direction, T_OctreeReport& report);

    }; // class SampleContainer

    /// Given the current position of a robot, returns a set
    /// of candidate sample configurations for contact generation.
    /// The set is strictly ordered using a heuristic to determine
    /// the most relevant contacts.
    ///
    /// \param sc the SampleContainer containing all the samples for a given limb
    /// \param treeTrf the current transformation of the root of the robot
    /// \param treeTrf the current transformation of the root of the robot
    /// \param direction the current direction of motion, used to evaluate the sample
    /// heuristically
    /// \return a set of OctreeReport with all the possible candidates for contact
    T_OctreeReport GetCandidates(const SampleContainer& sc, const fcl::Transform3f& treeTrf,
                                            const hpp::model::CollisionObjectPtr_t& o2,
                                            const fcl::Vec3f& direction);


    /// Given the current position of a robot, returns a set
    /// of candidate sample configurations for contact generation.
    /// The set is strictly ordered using a heuristic to determine
    /// the most relevant contacts.
    ///
    /// \param sc the SampleContainer containing all the samples for a given limb
    /// \param treeTrf the current transformation of the root of the robot
    /// \param treeTrf the current transformation of the root of the robot
    /// \param direction the current direction of motion, used to evaluate the sample
    /// heuristically
    /// \param a set of OctreeReport updated as the samples are explored
    /// \return true if at least one candidate was found
    bool GetCandidates(const SampleContainer& sc, const fcl::Transform3f& treeTrf,
                                            const hpp::model::CollisionObjectPtr_t& o2,
                                            const fcl::Vec3f& direction, T_OctreeReport& report);
  } // namespace sampling
} // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_SAMPLE_CONTAINER_HH
