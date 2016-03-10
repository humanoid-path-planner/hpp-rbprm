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
}

SampleDB::SampleDB(const model::JointPtr_t limb, const std::string& effector,
                   const std::size_t nbSamples, const fcl::Vec3f& offset, const double resolution, const T_evaluate& data,  const std::string& staticValue)
    : resolution_(resolution)
    , samples_(GenerateSamples(limb, effector, nbSamples, offset))
    , octomapTree_(generateOctree(samples_, resolution))
    , octree_(new fcl::OcTree(octomapTree_))
    , geometry_(boost::shared_ptr<fcl::CollisionGeometry>(octree_))
{
    for(T_evaluate::const_iterator cit = data.begin(); cit != data.end(); ++cit)
    {
        bool sort = staticValue == cit->first;
        addValue(*this, cit->first, cit->second, sort, sort);
    }
    sortDB(*this);
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
