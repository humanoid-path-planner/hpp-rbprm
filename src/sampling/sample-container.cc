// Copyright (c) 2014, LAAS-CNRS
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
// hpp-rbprm. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/rbprm/sampling/sample-container.hh>
#include <hpp/rbprm/sampling/sample-db.hh>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/model/collision-object.hh>

#include <vector>
#include <algorithm>

using namespace hpp;
using namespace hpp::model;
using namespace hpp::rbprm;
using namespace hpp::rbprm::sampling;
using namespace fcl;

 namespace
 {
    std::map<std::size_t, fcl::CollisionObject*> generateBoxesFromOctomap(const boost::shared_ptr<const octomap::OcTree>& octTree,
                                                                const fcl::OcTree* tree)
    {
        std::map<std::size_t, fcl::CollisionObject*> boxes;
        std::vector<boost::array<FCL_REAL, 6> > boxes_ = tree->toBoxes();
        //boxes.reserve(boxes_.size());
        for(std::size_t i = 0; i < boxes_.size(); ++i)
        {
            FCL_REAL x = boxes_[i][0];
            FCL_REAL y = boxes_[i][1];
            FCL_REAL z = boxes_[i][2];
            FCL_REAL size = boxes_[i][3];
            FCL_REAL cost = boxes_[i][4];
            FCL_REAL threshold = boxes_[i][5];
            std::size_t id = octTree->search(x,y,z) - octTree->getRoot();

            Box* box = new Box(size, size, size);
            box->cost_density = cost;
            box->threshold_occupied = threshold;
            fcl::CollisionObject* obj = new fcl::CollisionObject(boost::shared_ptr<fcl::CollisionGeometry>(box), Transform3f(Vec3f(x, y, z)));
            boxes.insert(std::make_pair(id,obj));
        }
        return boxes;
    }
 }

 SampleContainer::SampleContainer(const model::JointPtr_t limb, const std::string& effector, const std::size_t nbSamples, const Vec3f &offset, const double resolution)
     : sampleDB_(limb,effector,nbSamples,offset,resolution)
     , samples_(sampleDB_.samples_)
     , samplesInVoxels_(sampleDB_.samplesInVoxels_)
     , resolution_(sampleDB_.resolution_)
     , treeObject_(sampleDB_.geometry_)
     , boxes_(generateBoxesFromOctomap(sampleDB_.octomapTree_, sampleDB_.octree_))
 {
     // NOTHING
 }


SampleContainer::~SampleContainer()
{
    for (std::map<std::size_t, fcl::CollisionObject*>::const_iterator it = boxes_.begin();
         it != boxes_.end(); ++it)
    {
        delete it->second;
    }
}


OctreeReport::OctreeReport(const Sample& s, const fcl::Contact c, const double v, const fcl::Vec3f& normal)
    : sample_(s)
    , contact_(c)
    , value_(v)
    , normal_(normal)
{
    // NOTHING
}

// TODO Samples should be Vec3f
bool rbprm::sampling::GetCandidates(const SampleContainer& sc, const fcl::Transform3f& treeTrf,
                                    const fcl::Transform3f& treeTrf2,
                                    const hpp::model::CollisionObjectPtr_t& o2,
                                    const fcl::Vec3f& direction, hpp::rbprm::sampling::T_OctreeReport &reports,
                                    const heuristic evaluate)
{
    bool nextDiffers = treeTrf.getQuatRotation() != treeTrf2.getQuatRotation() ||
            (treeTrf.getTranslation() - treeTrf2.getTranslation()).norm() > 10e-3;
    fcl::CollisionRequest req(1000, true);
    fcl::CollisionResult cResult;
    fcl::CollisionObjectPtr_t obj = o2->fcl();
    fcl::collide(sc.sampleDB_.geometry_.get(), treeTrf, obj->collisionGeometry().get(), obj->getTransform(), req, cResult);
    sampling::T_VoxelSampleId::const_iterator voxelIt;
    Eigen::Vector3d eDir(direction[0], direction[1], direction[2]);
    std::vector<long int> visited;
    std::vector<long int> intersecting;
    int totalSamples = 0;
    int total_rejected = 0;
    int okay = 0;
    for(std::size_t index=0; index<cResult.numContacts(); ++index)
    {
        const Contact& contact = cResult.getContact(index);
        if(true)
        {
            if(std::find(visited.begin(), visited.end(), contact.b1) == visited.end())
                visited.push_back(contact.b1);
            //verifying that position is theoritically reachable from next position
            bool intersectNext = !nextDiffers ||
                    std::find(intersecting.begin(), intersecting.end(), contact.b1) != intersecting.end();
            if(nextDiffers && !intersectNext)
            {
                fcl::CollisionRequest reqTrees;
                fcl::CollisionResult cResultTrees;
                const fcl::CollisionObject* box = sc.boxes_.at(contact.b1);
                intersectNext = fcl::collide(box->collisionGeometry().get(), treeTrf, sc.sampleDB_.geometry_.get(),treeTrf2, reqTrees, cResultTrees);
                if(intersectNext)
                    intersecting.push_back(contact.b1);
            }
            if(true || intersectNext)
            {
                voxelIt = sc.samplesInVoxels_.find(contact.b1);
                const VoxelSampleId& voxelSampleIds = voxelIt->second;
                totalSamples += (int)voxelSampleIds.second;
                for(T_Sample::const_iterator sit = sc.samples_.begin()+ voxelSampleIds.first;
                    sit != sc.samples_.begin()+ voxelSampleIds.first + voxelSampleIds.second; ++sit)
                {
                    //find normal id
                    assert(contact.o2->getObjectType() == fcl::OT_BVH); // only works with meshes
                    const fcl::BVHModel<fcl::OBBRSS>* surface = static_cast<const fcl::BVHModel<fcl::OBBRSS>*> (contact.o2);
                    fcl::Vec3f normal;
                    const fcl::Triangle& tr = surface->tri_indices[contact.b2];
                    const fcl::Vec3f& v1 = surface->vertices[tr[0]];
                    const fcl::Vec3f& v2 = surface->vertices[tr[1]];
                    const fcl::Vec3f& v3 = surface->vertices[tr[2]];
                    normal = (v2 - v1).cross(v3 - v1);
                    normal.normalize();
                    Eigen::Vector3d eNormal(normal[0], normal[1], normal[2]);
                    OctreeReport report(*sit, contact,(*evaluate)(*sit, eDir, eNormal), normal);
                    ++okay;
                    reports.insert(report);
                }
            }
            else
            {
                total_rejected +=1;
            }
        }
    }
    return !reports.empty();
}


rbprm::sampling::T_OctreeReport rbprm::sampling::GetCandidates(const SampleContainer& sc, const fcl::Transform3f& treeTrf,
                                                               const fcl::Transform3f& treeTrf2,
                                                               const hpp::model::CollisionObjectPtr_t& o2,
                                                               const fcl::Vec3f& direction, const heuristic evaluate)
{
    rbprm::sampling::T_OctreeReport report;
    GetCandidates(sc, treeTrf, treeTrf2, o2, direction, report, evaluate);
    return report;
}
