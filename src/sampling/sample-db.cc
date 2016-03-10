#include <hpp/rbprm/sampling/sample-db.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/collision-object.hh>

#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/BVH/BVH_model.h>

using namespace hpp;
using namespace hpp::model;
using namespace hpp::rbprm;
using namespace hpp::rbprm::sampling;
using namespace fcl;

namespace
{
   octomap::OcTree* generateOctree(const T_Sample& samples, const double resolution)
   {
       octomap::OcTree* octTree = new octomap::OcTree(resolution);
       for(std::vector<Sample>::const_iterator cit = samples.begin();
           cit != samples.end(); ++cit)
       {
           const fcl::Vec3f& position = cit->effectorPosition_;
           octomap::point3d endpoint((float)position[0],(float)position[1],(float)position[2]);
           octTree->updateNode(endpoint, true);
       }
       octTree->updateInnerOccupancy();
       return octTree;
   }

   typedef std::map<long int, std::vector<std::size_t> > T_VoxelSample;
   T_VoxelSample getSamplesPerVoxel(const boost::shared_ptr<const octomap::OcTree>& octTree, const std::vector<Sample>& samples)
   {
       T_VoxelSample res;
       std::size_t sid = 0;
       for(std::vector<Sample>::const_iterator cit=samples.begin(); cit!=samples.end();++cit, ++sid)
       {
           const fcl::Vec3f& position = cit->effectorPosition_;
           long int id = octTree->search(position[0],position[1],position[2]) - octTree->getRoot();
           T_VoxelSample::iterator it = res.find(id);
           if(it!=res.end())
               it->second.push_back(sid);
           else
           {
               std::vector<std::size_t> samples;
               samples.push_back(sid);
               res.insert(std::make_pair(id, samples));
           }
       }
       return res;
   }

    void alignSampleOrderWithOctree(SampleDB& db)
    {
        std::vector<std::size_t> realignOrderIds; // indicate how to realign each value in value vector
        // assumes samples are sorted, so they are already sorted by interest
        // within octree
        T_Sample reorderedSamples; reorderedSamples.reserve(db.samples_.size());
        T_VoxelSample samplesPerVoxel = getSamplesPerVoxel(db.octomapTree_,db.samples_);

        //now sort all voxels by id for continuity
        // so that samples in adjacent voxels are also aligned
        std::vector<long int> voxelIds;
        for(T_VoxelSample::const_iterator vit = samplesPerVoxel.begin();
            vit != samplesPerVoxel.end(); ++vit)
        {
            voxelIds.push_back(vit->first);
        }
        std::sort(voxelIds.begin(),voxelIds.end());

        T_VoxelSampleId reorderedSamplesPerVoxel;
        std::size_t currentNewIndex = 0;
        for(std::vector<long int>::const_iterator vit = voxelIds.begin();
            vit != voxelIds.end(); ++vit)
        {
            const long int& voxelId=*vit;
            const std::vector<std::size_t>& sampleIds = samplesPerVoxel[voxelId];
            std::size_t startSampleId = currentNewIndex;
            for(std::vector<std::size_t>::const_iterator sit = sampleIds.begin();
                sit != sampleIds.end(); ++sit   )
            {
                reorderedSamples.push_back(db.samples_[*sit]);
                realignOrderIds.push_back(*sit);
                ++currentNewIndex;
            }
            reorderedSamplesPerVoxel.insert(std::make_pair(voxelId, std::make_pair(startSampleId,currentNewIndex-startSampleId)));
        }
        //Now reorder all values
        T_Values reorderedValues;
        for(T_Values::const_iterator cit = db.values_.begin(); cit != db.values_.end(); ++cit)
        {
            T_Double vals; vals.reserve(realignOrderIds.size());
            for(std::vector<std::size_t>::const_iterator rit = realignOrderIds.begin();
                rit!=realignOrderIds.end(); ++rit)
            {
                vals.push_back(cit->second[*rit]);
            }
            reorderedValues.insert(std::make_pair(cit->first, vals));
        }
        db.samplesInVoxels_ = reorderedSamplesPerVoxel;
        db.values_ = reorderedValues;
        db.samples_ = reorderedSamples;
    }

    void sortDB(SampleDB& database)
    {
        sample_greater sort;
        std::sort (database.samples_.begin(), database.samples_.end(), sort);
        alignSampleOrderWithOctree(database);
    }

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

SampleDB::SampleDB(const model::JointPtr_t limb, const std::string& effector,
                   const std::size_t nbSamples, const fcl::Vec3f& offset, const double resolution, const T_evaluate& data,  const std::string& staticValue)
    : resolution_(resolution)
    , samples_(GenerateSamples(limb, effector, nbSamples, offset))
    , octomapTree_(generateOctree(samples_, resolution))
    , octree_(new fcl::OcTree(octomapTree_))
    , geometry_(boost::shared_ptr<fcl::CollisionGeometry>(octree_))
    , treeObject_(geometry_)
    , boxes_(generateBoxesFromOctomap(octomapTree_, octree_))
{
    for(T_evaluate::const_iterator cit = data.begin(); cit != data.end(); ++cit)
    {
        bool sort = staticValue == cit->first;
        addValue(*this, cit->first, cit->second, sort, false);
    }
    sortDB(*this);
}


SampleDB::~SampleDB()
{
   for (std::map<std::size_t, fcl::CollisionObject*>::const_iterator it = boxes_.begin();
        it != boxes_.end(); ++it)
   {
       delete it->second;
   }
}


OctreeReport::OctreeReport(const Sample* s, const fcl::Contact c, const double v, const fcl::Vec3f& normal)
   : sample_(s)
   , contact_(c)
   , value_(v)
   , normal_(normal)
{
   // NOTHING
}

SampleDB& hpp::rbprm::sampling::addValue(SampleDB& database, const std::string& valueName, const evaluate eval, bool isStaticValue, bool sortSamples)
{
    T_Values::const_iterator cit = database.values_.find(valueName);
    if(cit != database.values_.end())
    {
        hppDout (warning, "value already existing for database " << valueName);
        return database;
    }
    else
    {
        T_Double values; values.reserve(database.samples_.size());
        for(T_Sample::iterator it = database.samples_.begin(); it != database.samples_.end(); ++it)
        {
            double val = eval(*it);
            values.push_back(val);
            if(isStaticValue)
                it->staticValue_ = val;
        }
        database.values_.insert(std::make_pair(valueName, values));
        if(sortSamples)
            sortDB(database);
    }
    return database;
}

/*SampleDB hpp::rbprm::sampling::loadLimbDatabase(const std::string& limbId, const std::string& dbFileName)
{

}

bool hpp::rbprm::sampling::saveLimbDatabase(const SampleDB& database, const std::string& dbFileName)
{

}*/

// TODO Samples should be Vec3f
bool rbprm::sampling::GetCandidates(const SampleDB& sc, const fcl::Transform3f& treeTrf,
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
    fcl::collide(sc.geometry_.get(), treeTrf, obj->collisionGeometry().get(), obj->getTransform(), req, cResult);
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
                intersectNext = fcl::collide(box->collisionGeometry().get(), treeTrf, sc.geometry_.get(),treeTrf2, reqTrees, cResultTrees);
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
                    OctreeReport report(&(*sit), contact,(*evaluate)(*sit, eDir, eNormal), normal);
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


rbprm::sampling::T_OctreeReport rbprm::sampling::GetCandidates(const SampleDB& sc, const fcl::Transform3f& treeTrf,
                                                               const fcl::Transform3f& treeTrf2,
                                                               const hpp::model::CollisionObjectPtr_t& o2,
                                                               const fcl::Vec3f& direction, const heuristic evaluate)
{
    rbprm::sampling::T_OctreeReport report;
    GetCandidates(sc, treeTrf, treeTrf2, o2, direction, report, evaluate);
    return report;
}

