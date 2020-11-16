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

#ifndef HPP_OCTREE_NODE_HH
#define HPP_OCTREE_NODE_HH

#include <hpp/rbprm/sampling/sample.hh>
#include <hpp/fcl/octree.h>

namespace hpp {

namespace rbprm {
namespace sampling {
HPP_PREDEF_CLASS(OctreeNode);

/// Sample container for a given limb of a robot.
/// Stores a list of Sample in two ways: a deque,
/// and an octree for spatial requests.
/// indexes of the octree corresponds to end effector positions.
class OctreeNode;
typedef boost::shared_ptr<OctreeNode> OctreeNodePtr_t;

class HPP_RBPRM_DLLAPI OctreeNode {
 public:
  /// Creates sample from Configuration
  /// in presented joint
  /// \param limb root joint for the considered limb
  /// \param nbSamples number of samples to generate
  /// \param resolution, resolution of the octree voxels
  OctreeNode(const pinocchio::JointPtr_t limb, const std::size_t nbSamples, const double resolution = 0.1);
  ~OctreeNode();

 private:
 public:
  /// samples generated
  const std::deque<Sample> samples_;

 private:
  fcl::OcTree* octree_;  // deleted with geometry_
  const boost::shared_ptr<fcl::CollisionGeometry> geometry_;

 public:
  const fcl::CollisionObject treeObject_;
  /// Bounding boxes of areas of interest of the octree
  const std::vector<fcl::CollisionObject*> boxes_;

};  // class OctreeNode
}  // namespace sampling
}  // namespace rbprm
}  // namespace hpp
#endif  // HPP_OCTREE_NODE_HH
