#include <hpp/rbprm/sampling/sample-db.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/collision-object.hh>

#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/BVH/BVH_model.h>

#include <hpp/rbprm/tools.hh>

#include <iostream>
#include <fstream>
#include <string>

using namespace hpp;
using namespace hpp::pinocchio;
using namespace hpp::rbprm;
using namespace hpp::rbprm::sampling;
using namespace fcl;

namespace {
octomap::OcTree* generateOctree(const T_Sample& samples, const double resolution) {
  octomap::OcTree* octTree = new octomap::OcTree(resolution);
  for (SampleVector_t::const_iterator cit = samples.begin(); cit != samples.end(); ++cit) {
    const fcl::Vec3f& position = cit->effectorPosition_;
    octomap::point3d endpoint((float)position[0], (float)position[1], (float)position[2]);
    octTree->updateNode(endpoint, true);
  }
  octTree->updateInnerOccupancy();
  return octTree;
}

typedef std::map<long int, std::vector<std::size_t> > T_VoxelSample;
T_VoxelSample getSamplesPerVoxel(const boost::shared_ptr<const octomap::OcTree>& octTree,
                                 const SampleVector_t& samples) {
  T_VoxelSample res;
  std::size_t sid = 0;
  for (SampleVector_t::const_iterator cit = samples.begin(); cit != samples.end(); ++cit, ++sid) {
    const fcl::Vec3f& position = cit->effectorPosition_;
    long int id = octTree->search(position[0], position[1], position[2]) - octTree->getRoot();
    T_VoxelSample::iterator it = res.find(id);
    if (it != res.end())
      it->second.push_back(sid);
    else {
      std::vector<std::size_t> samples;
      samples.push_back(sid);
      res.insert(std::make_pair(id, samples));
    }
  }
  return res;
}

void alignSampleOrderWithOctree(SampleDB& db) {
  std::vector<std::size_t> realignOrderIds;  // indicate how to realign each value in value vector
  // assumes samples are sorted, so they are already sorted by interest
  // within octree
  T_Sample reorderedSamples;
  reorderedSamples.reserve(db.samples_.size());
  T_VoxelSample samplesPerVoxel = getSamplesPerVoxel(db.octomapTree_, db.samples_);

  // now sort all voxels by id for continuity
  // so that samples in adjacent voxels are also aligned
  std::vector<long int> voxelIds;
  for (T_VoxelSample::const_iterator vit = samplesPerVoxel.begin(); vit != samplesPerVoxel.end(); ++vit) {
    voxelIds.push_back(vit->first);
  }
  std::sort(voxelIds.begin(), voxelIds.end());

  T_VoxelSampleId reorderedSamplesPerVoxel;
  std::size_t currentNewIndex = 0;
  for (std::vector<long int>::const_iterator vit = voxelIds.begin(); vit != voxelIds.end(); ++vit) {
    const long int& voxelId = *vit;
    const std::vector<std::size_t>& sampleIds = samplesPerVoxel[voxelId];
    std::size_t startSampleId = currentNewIndex;
    for (std::vector<std::size_t>::const_iterator sit = sampleIds.begin(); sit != sampleIds.end(); ++sit) {
      reorderedSamples.push_back(db.samples_[*sit]);
      reorderedSamples.back().id_ = currentNewIndex;
      realignOrderIds.push_back(*sit);
      ++currentNewIndex;
    }
    reorderedSamplesPerVoxel.insert(
        std::make_pair(voxelId, std::make_pair(startSampleId, currentNewIndex - startSampleId)));
  }
  // Now reorder all values
  T_Values reorderedValues;
  for (T_Values::const_iterator cit = db.values_.begin(); cit != db.values_.end(); ++cit) {
    T_Double vals;
    vals.reserve(realignOrderIds.size());
    for (std::vector<std::size_t>::const_iterator rit = realignOrderIds.begin(); rit != realignOrderIds.end(); ++rit) {
      vals.push_back(cit->second[*rit]);
    }
    reorderedValues.insert(std::make_pair(cit->first, vals));
  }
  db.samplesInVoxels_ = reorderedSamplesPerVoxel;
  db.values_ = reorderedValues;
  db.samples_ = reorderedSamples;
}

void sortDB(SampleDB& database) {
  sample_greater sort;
  std::sort(database.samples_.begin(), database.samples_.end(), sort);
  alignSampleOrderWithOctree(database);
}

std::map<std::size_t, fcl::CollisionObject*> generateBoxesFromOctomap(
    const boost::shared_ptr<const octomap::OcTree>& octTree, const fcl::OcTree* tree) {
  std::map<std::size_t, fcl::CollisionObject*> boxes;
  std::vector<boost::array<FCL_REAL, 6> > boxes_ = tree->toBoxes();
  // boxes.reserve(boxes_.size());
  for (std::size_t i = 0; i < boxes_.size(); ++i) {
    FCL_REAL x = boxes_[i][0];
    FCL_REAL y = boxes_[i][1];
    FCL_REAL z = boxes_[i][2];
    FCL_REAL size = boxes_[i][3];
    FCL_REAL cost = boxes_[i][4];
    FCL_REAL threshold = boxes_[i][5];
    std::size_t id = octTree->search(x, y, z) - octTree->getRoot();

    Box* box = new Box(size, size, size);
    box->cost_density = cost;
    box->threshold_occupied = threshold;
    fcl::CollisionObject* obj =
        new fcl::CollisionObject(boost::shared_ptr<fcl::CollisionGeometry>(box), fcl::Transform3f(Vec3f(x, y, z)));
    boxes.insert(std::make_pair(id, obj));
  }
  return boxes;
}
}  // namespace

SampleDB::SampleDB(const pinocchio::JointPtr_t limb, const std::string& effector, const std::size_t nbSamples,
                   const fcl::Vec3f& offset, const fcl::Vec3f& limbOffset, const double resolution,
                   const T_evaluate& data, const std::string& staticValue)
    : resolution_(resolution),
      samples_(GenerateSamples(limb, effector, nbSamples, offset, limbOffset)),
      octomapTree_(generateOctree(samples_, resolution)),
      octree_(new fcl::OcTree(octomapTree_)),
      geometry_(boost::shared_ptr<fcl::CollisionGeometry>(octree_)),
      treeObject_(geometry_),
      boxes_(generateBoxesFromOctomap(octomapTree_, octree_)) {
  for (T_evaluate::const_iterator cit = data.begin(); cit != data.end(); ++cit) {
    bool sort = staticValue == cit->first;
    addValue(*this, cit->first, cit->second, sort, false);
  }
  sortDB(*this);
}

SampleDB::~SampleDB() {
  for (std::map<std::size_t, fcl::CollisionObject*>::const_iterator it = boxes_.begin(); it != boxes_.end(); ++it) {
    delete it->second;
  }
}

OctreeReport::OctreeReport(const Sample* s, const fcl::Contact c, const double v, const fcl::Vec3f& normal,
                           const fcl::Vec3f& v1, const fcl::Vec3f& v2, const fcl::Vec3f& v3)
    : sample_(s), contact_(c), value_(v), normal_(normal), v1_(v1), v2_(v2), v3_(v3) {
  // NOTHING
}

SampleDB& hpp::rbprm::sampling::addValue(SampleDB& database, const std::string& valueName, const evaluate eval,
                                         bool isStaticValue, bool sortSamples) {
  T_Values::const_iterator cit = database.values_.find(valueName);
  if (cit != database.values_.end()) {
    hppDout(warning, "value already existing for database " << valueName);
    return database;
  } else {
    double maxValue = -std::numeric_limits<double>::max();
    double minValue = std::numeric_limits<double>::max();
    T_Double values;
    values.reserve(database.samples_.size());
    for (T_Sample::iterator it = database.samples_.begin(); it != database.samples_.end(); ++it) {
      double val = eval(database, *it);
      maxValue = std::max(maxValue, val);
      minValue = std::min(minValue, val);
      values.push_back(val);
    }
    database.valueBounds_.insert(std::make_pair(valueName, std::make_pair(minValue, maxValue)));
    // now normalize values
    double max_min = maxValue - minValue;
    for (T_Double::iterator it = values.begin(); it != values.end(); ++it) {
      *it = (max_min != 0) ? (*it - minValue) / max_min : 0;
    }
    if (isStaticValue) {
      for (size_t id = 0; id < database.samples_.size(); id++) database.samples_[id].staticValue_ = values[id];
    }
    database.values_.insert(std::make_pair(valueName, values));
    if (sortSamples) sortDB(database);
  }
  return database;
}

// TODO Samples should be Vec3f
bool rbprm::sampling::GetCandidates(const SampleDB& sc, const fcl::Transform3f& treeTrf,
                                    const hpp::pinocchio::CollisionObjectPtr_t& o2, const fcl::Vec3f& direction,
                                    hpp::rbprm::sampling::T_OctreeReport& reports, const HeuristicParam& params,
                                    const heuristic evaluate) {
  fcl::CollisionRequest req(fcl::CONTACT, 1000);
  fcl::CollisionResult cResult;
  fcl::CollisionObject* obj = o2->fcl();
  fcl::collide(sc.geometry_.get(), treeTrf, obj->collisionGeometry().get(), obj->getTransform(), req, cResult);
  sampling::T_VoxelSampleId::const_iterator voxelIt;
  Eigen::Vector3d eDir(direction[0], direction[1], direction[2]);
  eDir.normalize();
  std::vector<long int> visited;
  int totalSamples = 0;
  int okay = 0;
  for (std::size_t index = 0; index < cResult.numContacts(); ++index) {
    const Contact& contact = cResult.getContact(index);
    if (std::find(visited.begin(), visited.end(), contact.b1) == visited.end()) visited.push_back(contact.b1);
    // verifying that position is theoritically reachable from next position
    {
      voxelIt = sc.samplesInVoxels_.find(contact.b1);
      if (voxelIt != sc.samplesInVoxels_.end()) {
        const VoxelSampleId& voxelSampleIds = voxelIt->second;
        totalSamples += (int)voxelSampleIds.second;
        for (T_Sample::const_iterator sit = sc.samples_.begin() + voxelSampleIds.first;
             sit != sc.samples_.begin() + voxelSampleIds.first + voxelSampleIds.second; ++sit) {
          // find normal id
          assert(contact.o2->getObjectType() == fcl::OT_BVH);  // only works with meshes
          const fcl::BVHModel<fcl::OBBRSS>* surface = static_cast<const fcl::BVHModel<fcl::OBBRSS>*>(contact.o2);
          fcl::Vec3f normal;
          const fcl::Triangle& tr = surface->tri_indices[contact.b2];
          const fcl::Vec3f& v1 = surface->vertices[tr[0]];
          const fcl::Vec3f& v2 = surface->vertices[tr[1]];
          const fcl::Vec3f& v3 = surface->vertices[tr[2]];
          normal = (v2 - v1).cross(v3 - v1);
          normal.normalize();
          Eigen::Vector3d eNormal(normal[0], normal[1], normal[2]);
          OctreeReport report(&(*sit), contact, evaluate ? ((*evaluate)(*sit, eDir, eNormal, params)) : 0, eNormal, v1,
                              v2, v3);
          ++okay;
          reports.insert(report);
        }
      } else
        hppDout(warning, "no voxels in specified triangle : " << contact.b1);
    }
  }
  return !reports.empty();
}

rbprm::sampling::T_OctreeReport rbprm::sampling::GetCandidates(const SampleDB& sc, const fcl::Transform3f& treeTrf,
                                                               const hpp::pinocchio::CollisionObjectPtr_t& o2,
                                                               const fcl::Vec3f& direction,
                                                               const HeuristicParam& params,
                                                               const heuristic evaluate) {
  rbprm::sampling::T_OctreeReport report;
  GetCandidates(sc, treeTrf, o2, direction, report, params, evaluate);
  return report;
}

using std::ostringstream;
using namespace tools::io;

void writeSample(const Sample& sample, std::ostream& output) {
  output << "sample" << std::endl;
  output << sample.id_ << std::endl;
  output << sample.length_ << std::endl;
  output << sample.startRank_ << std::endl;
  output << sample.staticValue_ << std::endl;
  writeVecFCL(sample.effectorPosition_, output);
  output << std::endl;
  writeVecFCL(sample.effectorPositionInLimbFrame_, output);
  output << std::endl;
  writeMatrix(sample.configuration_, output);
  output << std::endl;
  writeMatrix(sample.jacobian_, output);
  output << std::endl;
  writeMatrix(sample.jacobianProduct_, output);
  output << std::endl;
}

void writeSamples(const SampleDB& database, std::ostream& output) {
  for (T_Sample::const_iterator cit = database.samples_.begin(); cit != database.samples_.end(); ++cit) {
    writeSample(*cit, output);
  }
}

void writeValues(const SampleDB& database, std::ostream& output) {
  for (T_Values::const_iterator cit = database.values_.begin(); cit != database.values_.end(); ++cit) {
    output << "value" << std::endl << cit->first << std::endl;
    for (T_Double::const_iterator dit = cit->second.begin(); dit != cit->second.end(); ++dit) {
      output << *dit << std::endl;
    }
    output << std::endl;
  }
}

Sample readSample(std::ifstream& myfile, std::string& line) {
  std::size_t id, length, startRank;
  double staticValue;
  fcl::Vec3f effpos, effPosInLimbFrame;
  Eigen::VectorXd conf;
  Eigen::MatrixXd jav, ajcprod;
  id = StrToI(myfile);
  length = StrToI(myfile);
  startRank = StrToI(myfile);
  staticValue = StrToD(myfile);
  effpos = readVecFCL(myfile, line);
  effPosInLimbFrame = readVecFCL(myfile, line);
  conf = readMatrix(myfile, line);
  jav = readMatrix(myfile, line);
  ajcprod = readMatrix(myfile, line);
  return Sample(id, length, startRank, staticValue, effpos, effPosInLimbFrame, conf, jav, ajcprod);
}

void readValue(T_Values& values, const std::size_t& size, std::ifstream& myfile, std::string& line) {
  getline(myfile, line);  // name
  std::string valueName = line;
  T_Double vals;
  vals.reserve(size);
  for (std::size_t i = 0; i < size; ++i) {
    getline(myfile, line);
    vals.push_back(StrToD(line));
  }
  values.insert(std::make_pair(valueName, vals));
}

bool hpp::rbprm::sampling::saveLimbDatabase(const SampleDB& database, std::ofstream& fp) {
  fp << database.samples_.size() << std::endl;
  fp << database.resolution_ << std::endl;
  writeSamples(database, fp);
  writeValues(database, fp);
  return true;
}

SampleDB::SampleDB(std::ifstream& myfile, bool loadValues)
    : treeObject_(boost::shared_ptr<CollisionGeometry>(new fcl::Box(1, 1, 1))) {
  if (!myfile.good()) throw std::runtime_error("Impossible to open database");
  if (myfile.is_open()) {
    std::string line;
    getline(myfile, line);
    std::size_t size = StrToI(line);
    samples_.reserve(size);
    getline(myfile, line);
    resolution_ = StrToD(line);
    while (myfile.good()) {
      getline(myfile, line);
      if (line.find("sample") != std::string::npos)
        samples_.push_back(readSample(myfile, line));
      else if (line.find("value") != std::string::npos && loadValues)
        readValue(values_, size, myfile, line);
    }
  }
  octomapTree_ = boost::shared_ptr<const octomap::OcTree>(generateOctree(samples_, resolution_));
  octree_ = new fcl::OcTree(octomapTree_);
  geometry_ = boost::shared_ptr<fcl::CollisionGeometry>(octree_);
  treeObject_ = fcl::CollisionObject(geometry_);
  boxes_ = generateBoxesFromOctomap(octomapTree_, octree_);
  alignSampleOrderWithOctree(*this);
}
