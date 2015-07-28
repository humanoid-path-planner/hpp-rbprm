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
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/model/collision-object.hh>

#include <vector>

using namespace hpp;
using namespace hpp::model;
using namespace hpp::rbprm;
using namespace hpp::rbprm::sampling;
using namespace fcl;

 namespace
 {

    octomap::OcTree* GenerateOctree(const std::deque<Sample>& samples, const double resolution)
    {
        octomap::OcTree* octTree = new octomap::OcTree(resolution);
        for(std::deque<Sample>::const_iterator cit = samples.begin();
            cit != samples.end(); ++cit)
        {
            const fcl::Vec3f& position = cit->effectorPosition_;
            octomap::point3d endpoint((float)position[0],(float)position[1],(float)position[2]);
            octTree->updateNode(endpoint, true);
        }
        octTree->updateInnerOccupancy();
        return octTree;
    }

    std::vector<fcl::CollisionObject*> generateBoxesFromOctomap(const fcl::OcTree* tree)
    {
        std::vector<fcl::CollisionObject*> boxes;
        std::vector<boost::array<FCL_REAL, 6> > boxes_ = tree->toBoxes();
        boxes.reserve(boxes_.size());
        for(std::size_t i = 0; i < boxes_.size(); ++i)
        {
            FCL_REAL x = boxes_[i][0];
            FCL_REAL y = boxes_[i][1];
            FCL_REAL z = boxes_[i][2];
            FCL_REAL size = boxes_[i][3];
            FCL_REAL cost = boxes_[i][4];
            FCL_REAL threshold = boxes_[i][5];

            Box* box = new Box(size, size, size);
            box->cost_density = cost;
            box->threshold_occupied = threshold;
            fcl::CollisionObject* obj = new fcl::CollisionObject(boost::shared_ptr<fcl::CollisionGeometry>(box), Transform3f(Vec3f(x, y, z)));
            boxes.push_back(obj);
        }
        return boxes;
    }

    SampleContainer::T_VoxelSample SortSamples(const boost::shared_ptr<const octomap::OcTree>& octTree, const std::deque<Sample>& samples)
    {
        SampleContainer::T_VoxelSample res;
        for(std::deque<Sample>::const_iterator cit=samples.begin(); cit!=samples.end();++cit)
        {
            const fcl::Vec3f& position = cit->effectorPosition_;
            std::size_t id = octTree->search(position[0],position[1],position[2]) - octTree->getRoot();
            SampleContainer::T_VoxelSample::iterator it = res.find(id);
            if(it!=res.end())
            {
                it->second.push_back(&(*cit));
            }
            else
            {
                std::vector<const Sample*> samples;
                samples.push_back(&(*cit));
                res.insert(std::make_pair(id, samples));
            }
        }
        return res;
    }
 }

 struct rbprm::sampling::SamplePImpl
 {
     SamplePImpl(const std::deque<Sample>& samples, const double& resolution)
         : octomapTree_(GenerateOctree(samples, resolution))
         , octree_(new fcl::OcTree(octomapTree_))
         , geometry_(boost::shared_ptr<fcl::CollisionGeometry>(octree_))
     {
         // NOTHING
     }

     boost::shared_ptr<const octomap::OcTree> octomapTree_;
     fcl::OcTree* octree_; // deleted with geometry_
     const boost::shared_ptr<fcl::CollisionGeometry> geometry_;
 };

SampleContainer::SampleContainer(const model::JointPtr_t limb, const std::string& effector, const std::size_t nbSamples, const fcl::Vec3f& offset, const double resolution)
    : samples_(GenerateSamples(limb,effector,nbSamples, offset))
    , pImpl_(new SamplePImpl(samples_, resolution))
    , treeObject_(pImpl_->geometry_)
    , voxelSamples_(SortSamples(pImpl_->octomapTree_,samples_))
    , boxes_(generateBoxesFromOctomap(pImpl_->octree_))
{
    // NOTHING
}

SampleContainer::~SampleContainer()
{
    for (std::vector<fcl::CollisionObject*>::const_iterator it = boxes_.begin();
         it != boxes_.end(); ++it)
    {
        delete *it;
    }
}


OctreeReport::OctreeReport(const Sample* s, const fcl::Contact c, const double m, const fcl::Vec3f& normal)
    : sample_(s)
    , contact_(c)
    , manipulability_(m)
    , normal_(normal)
{
    // NOTHING
}

// TODO Samples should be Vec3f
bool rbprm::sampling::GetCandidates(const SampleContainer& sc, const fcl::Transform3f& treeTrf,
                                                                          const hpp::model::CollisionObjectPtr_t& o2,
                                                                          const fcl::Vec3f& direction, hpp::rbprm::sampling::T_OctreeReport &reports)
{
    fcl::CollisionRequest req(30, true);
    fcl::CollisionResult cResult;
    fcl::CollisionObjectPtr_t obj = o2->fcl();
    fcl::collide(sc.pImpl_->geometry_.get(), treeTrf, obj->collisionGeometry().get(), obj->getTransform(), req, cResult);
    sampling::SampleContainer::T_VoxelSample::const_iterator voxelIt;
    //const fcl::Vec3f rotatedDir = treeTrf.getRotation() * direction;
    Eigen::Vector3d eDir(direction[0], direction[1], direction[2]);
    //Eigen::Vector3d eRotDir(rotatedDir[0], rotatedDir[1], rotatedDir[2]);
    for(std::size_t index=0; index<cResult.numContacts(); ++index)
    {
        const Contact& contact = cResult.getContact(index);
        voxelIt = sc.voxelSamples_.find(contact.b1);
        const std::vector<const sampling::Sample*>& samples = voxelIt->second;
        /*for(std::vector<const sampling::Sample*>::const_iterator sit = samples.begin();
            sit != samples.end(); ++sit)
        {
            //find normal id
            assert(contact.o2->getObjectType() == fcl::OT_BVH); // only works with meshes
            const fcl::BVHModel<fcl::RSS>* surface = static_cast<const fcl::BVHModel<fcl::RSS>*> (contact.o2);
            fcl::Vec3f normal; // = -bestReport.contact_.normal;
            const fcl::Triangle& tr = *surface->tri_indices;
            const fcl::Vec3f& v1 = surface->vertices[tr[0]];
            const fcl::Vec3f& v2 = surface->vertices[tr[1]];
            const fcl::Vec3f& v3 = surface->vertices[tr[2]];
            normal = (v2 - v1).cross(v3 - v1);
            normal.normalize();
            Eigen::Vector3d eNormal(normal[0], normal[1], normal[2]);
            Eigen::Vector3d eDir(direction[0], direction[1], direction[2]);
            double EFORT = eDir.transpose() * (*sit)->jacobianProduct_.block<3,3>(0,0) * eDir;
            //EFORT *= (eDir.dot(eNormal));
            OctreeReport report(*sit, contact,EFORT , normal);
            reports.insert(report);
        }*/
        for(std::vector<const sampling::Sample*>::const_iterator sit = samples.begin();
            sit != samples.end(); ++sit)
        {
            //find normal id
            assert(contact.o2->getObjectType() == fcl::OT_BVH); // only works with meshes
            const fcl::BVHModel<fcl::OBBRSS>* surface = static_cast<const fcl::BVHModel<fcl::OBBRSS>*> (contact.o2);
            fcl::Vec3f normal; // = -bestReport.contact_.normal;

            const fcl::Triangle& tr = surface->tri_indices[contact.b2];
            const fcl::Vec3f& v1 = surface->vertices[tr[0]];
            const fcl::Vec3f& v2 = surface->vertices[tr[1]];
            const fcl::Vec3f& v3 = surface->vertices[tr[2]];
            normal = (v2 - v1).cross(v3 - v1);
            normal.normalize();
            double EFORT = eDir.transpose() * (*sit)->jacobianProduct_.block<3,3>(0,0) * eDir;
            EFORT *= (direction.dot(normal));
            OctreeReport report(*sit, contact,EFORT, normal);
            reports.insert(report);
        }
    }
    return !reports.empty();
}


rbprm::sampling::T_OctreeReport rbprm::sampling::GetCandidates(const SampleContainer& sc, const fcl::Transform3f& treeTrf,
                                                                          const hpp::model::CollisionObjectPtr_t& o2,
                                                                          const fcl::Vec3f& direction)
{
    rbprm::sampling::T_OctreeReport report;
    GetCandidates(sc, treeTrf, o2, direction, report);
    return report;
}
