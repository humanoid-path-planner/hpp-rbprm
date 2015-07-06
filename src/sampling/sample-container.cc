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

SampleContainer::SampleContainer(const model::JointPtr_t limb, const std::size_t nbSamples, const double resolution)
    : samples_(GenerateSamples(limb,nbSamples))
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


OctreeReport::OctreeReport(const Sample* s, const fcl::Contact c, const double m)
    : sample_(s)
    , contact_(c)
    , manipulability_(m)
{
    // NOTHING
}

// TODO include dot product with normal
rbprm::sampling::T_OctreeReport rbprm::sampling::GetCandidates(const SampleContainer& sc, const fcl::Transform3f& treeTrf,
                                                                          const hpp::model::CollisionObjectPtr_t& o2,
                                                                          const Eigen::Vector3d& direction)
{
    fcl::CollisionRequest req(10, true);
    fcl::CollisionResult cResult;
    fcl::CollisionObjectPtr_t obj = o2->fcl();
    fcl::collide(sc.pImpl_->geometry_.get(), treeTrf, obj->collisionGeometry().get(), obj->getTransform(), req, cResult);
    T_OctreeReport result;
    sampling::SampleContainer::T_VoxelSample::const_iterator voxelIt;
    for(std::size_t index=0; index<cResult.numContacts(); ++index)
    {
        const Contact& contact = cResult.getContact(index);
        voxelIt = sc.voxelSamples_.find(contact.b1);
        const std::vector<const sampling::Sample*>& samples = voxelIt->second;
        for(std::vector<const sampling::Sample*>::const_iterator sit = samples.begin();
            sit != samples.end(); ++sit)
        {
            OctreeReport report(*sit, contact, direction.transpose() * (*sit)->jacobianProduct_.block<3,3>(0,0) * direction);
            result.insert(report);
        }
    }
    return result;
}
