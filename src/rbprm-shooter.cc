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

#include <hpp/core/collision-validation-report.hh>
#include <hpp/rbprm/rbprm-shooter.hh>
#include <hpp/rbprm/rbprm-validation-report.hh>
#include <hpp/pinocchio/liegroup-space.hh>
#include <hpp/pinocchio/liegroup-element.hh>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <hpp/core/configuration-shooter/uniform.hh>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/core/collision-validation.hh>
#include <Eigen/Geometry>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/util/timer.hh>
#include <hpp/core/problem.hh>

namespace hpp {
using namespace core;
using namespace fcl;
namespace {
static const int SIZE_EULER = 6;
typedef fcl::BVHModel<OBBRSS> BVHModelOB;
typedef shared_ptr<const BVHModelOB> BVHModelOBConst_Ptr_t;

BVHModelOBConst_Ptr_t GetModel(const pinocchio::FclConstCollisionObjectPtr_t object) {
  if (object->collisionGeometry()->getNodeType() != BV_OBBRSS) {
    hppDout(warning, "Collision geometry in shooter is not a BV_OBBRSS, cannot get the model.");
    return BVHModelOBConst_Ptr_t();
  }
  // assert(object->collisionGeometry()->getNodeType() == BV_OBBRSS);
  const BVHModelOBConst_Ptr_t model = boost::static_pointer_cast<const BVHModelOB>(object->collisionGeometry());
  // assert(model->getModelType() == BVH_MODEL_TRIANGLES);
  if (model->getModelType() != BVH_MODEL_TRIANGLES) {
    hppDout(warning, "Collision model is not of type BVH_MODEL_TRIANGLES.");
    return BVHModelOBConst_Ptr_t();
  }
  return model;
}

double TriangleArea(rbprm::TrianglePoints& tri) {
  double a, b, c;
  a = (tri.p1 - tri.p2).norm();
  b = (tri.p2 - tri.p3).norm();
  c = (tri.p3 - tri.p1).norm();
  double s = 0.5 * (a + b + c);
  return sqrt(s * (s - a) * (s - b) * (s - c));
}

std::vector<double> getTranslationBounds(const pinocchio::RbPrmDevicePtr_t robot) {
  const JointPtr_t root = robot->Device::rootJoint();
  std::vector<double> res;
  for (std::size_t i = 0; i < 3; ++i) {
    if (root->isBounded(i)) {
      res.push_back(root->lowerBound(i));
      res.push_back(root->upperBound(i));
    } else {
      res.push_back(-std::numeric_limits<double>::max());
      res.push_back(std::numeric_limits<double>::max());
    }
  }
  return res;
}

void SetConfigTranslation(const pinocchio::RbPrmDevicePtr_t robot, Configuration_t& config, const Vec3f& translation) {
  std::vector<double> bounds = getTranslationBounds(robot);
  for (std::size_t i = 0; i < 3; ++i) {
    config(i) = std::min(bounds[2 * i + 1], std::max(bounds[2 * i], translation[i]));
  }
}

void Translate(const pinocchio::RbPrmDevicePtr_t robot, Configuration_t& config, const Vec3f& translation) {
  // bound to positions limits
  std::vector<double> bounds = getTranslationBounds(robot);
  for (int i = 0; i < 3; ++i) {
    config(i) = std::min(bounds[2 * i + 1], std::max(bounds[2 * i], config(i) + translation[i]));
  }
}

/*void SampleRotationRec(ConfigurationPtr_t config, JointVector_t& jv, std::size_t& current)
{
    JointPtr_t joint = jv[current++];
    std::size_t rank = joint->rankInConfiguration ();
    const hpp::pinocchio::LiegroupSpacePtr_t jm = joint->configurationSpace();
    jm->neutral().setNeutral(); setRandom ();
    hpp::pinocchio::LiegroupSpace toto;
    //toto/
    joint->jointModel()-> configuration ()->uniformlySample (rank, *config);
    if(current<jv.size())
        SampleRotationRec(config,jv,current);
}*/

void SampleRotation(const std::vector<double>& so3, Configuration_t& config) {
  if (so3.empty()) return;
  assert(so3.size() == SIZE_EULER);
  Eigen::Vector3d rot;
  for (int i = 0; i < 6; i += 2) {
    rot[i / 2] = so3[i] + (so3[i + 1] - so3[i]) * rand() / RAND_MAX;
    // std::cout << "rot i " << rot [i/2] << " i " << i/2 << std::endl;
  }

  Eigen::Quaterniond qt = Eigen::AngleAxisd(rot(0), Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(rot(1), Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(rot(2), Eigen::Vector3d::UnitX());
  std::size_t rank = 3;
  /*for(std::size_t i = 0; i <4; ++i)
  {
      config(rank+i) = qt.coeffs()(i);
  }*/
  config.segment<4>(rank) = qt.coeffs();
}

/*void SampleRotation(pinocchio::DevicePtr_t so3, ConfigurationPtr_t config, JointVector_t& jv)
{
    std::size_t id = 1;
    if(so3->rootJoint())
    {
        Eigen::Matrix <value_type, 3, 1> confso3;
        id+=1;
        pinocchio::JointPtr_t joint = so3->rootJoint();
        for(int i =0; i <3; ++i)
        {
            joint->configuration()->uniformlySample (i, confso3);
            joint = joint->numberChildJoints() > 0 ? joint->childJoint(0) : 0;
        }
        Eigen::Quaterniond qt = Eigen::AngleAxisd(confso3(0), Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(confso3(1), Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(confso3(2), Eigen::Vector3d::UnitX());
        std::size_t rank = 3;
        (*config)(rank+0) = qt.w();
        (*config)(rank+1) = qt.x();
        (*config)(rank+2) = qt.y();
        (*config)(rank+3) = qt.z();
    }
    if(id < jv.size())
        SampleRotationRec(config,jv,id);
}*/

/*std::vector<double> initSo3()
{
    std::vector<double> res;
    int sign = -1;
    for (int i =0; i<6; ++i)
    {
        res.push_back(sign * std::numeric_limits<double>::infinity());
        sign *= -1;
    }
    //DevicePtr_t so3Robot = pinocchio::Device::create("so3Robot");
    //so3Robot->rootJoint(0);
    return res;
}*/

void seRotationtLimits(std::vector<double>& so3Robot, const std::vector<double>& limitszyx) {
  assert(SIZE_EULER == limitszyx.size());
  so3Robot = limitszyx;
  /*pinocchio::Joint* previous = so3Robot->rootJoint();
  if(previous == 0)
  {
      // init joints
      previous = new pinocchio::jointRotation::Bounded(fcl::Transform3f());
      pinocchio::Joint * jy = new pinocchio::jointRotation::Bounded(fcl::Transform3f());
      pinocchio::Joint * jx = new pinocchio::jointRotation::Bounded(fcl::Transform3f());
      so3Robot->rootJoint(previous);
      previous->addChildJoint (jy);
      jy->addChildJoint (jx);
      previous->name("so3z");
      jy->name("so3y");
      jx->name("so3x");
  }
  // set limits pinocchio::JointPtr_t
  assert(limitszyx.size() == 6);
  int i = 0;
  pinocchio::JointPtr_t current = previous;
  for(std::vector<double>::const_iterator cit = limitszyx.begin();
      cit != limitszyx.end(); ++cit, ++i)
  {
      if(i % 2 == 0)
      {
          current->lowerBound(0, *cit);
      }
      else
      {
          current->upperBound(0, *cit);
          current = current->numberChildJoints() > 0 ? current->childJoint(0) : 0;
      }
  }*/
}

}  // namespace

namespace rbprm {

RbPrmShooterPtr_t RbPrmShooter::create(const pinocchio::RbPrmDevicePtr_t& robot, const ObjectStdVector_t& geometries,
                                       const affMap_t& affordances, const std::vector<std::string>& filter,
                                       const std::map<std::string, std::vector<std::string> >& affFilters,
                                       const std::size_t shootLimit, const std::size_t displacementLimit) {
  unsigned int seed = (unsigned int)(time(NULL));
  srand(seed);
  hppDout(notice, "&&&&&& SEED = " << seed);
  RbPrmShooter* ptr =
      new RbPrmShooter(robot, geometries, affordances, filter, affFilters, shootLimit, displacementLimit);

  RbPrmShooterPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

void RbPrmShooter::init(const RbPrmShooterPtr_t& self) {
  ConfigurationShooter::init(self);
  weak_ = self;
}

void RbPrmShooter::BoundSO3(const std::vector<double>& limitszyx) { seRotationtLimits(eulerSo3_, limitszyx); }

/**
 * @brief getUsedSurfaces produce a list of CollisionObject from the affordances list :
 *  use all objects corresponding to at least one affordance filter set.
 * @param affordances
 * @param affFilters
 * @return
 */
hpp::core::ObjectStdVector_t getUsedSurfaces(const affMap_t& affordances,
                                             const std::map<std::string, std::vector<std::string> >& affFilters) {
  core::ObjectStdVector_t surfaces;
  std::set<std::string> addedTypes;
  hppDout(notice, "Begin getUsedSurfaces from affordances");
  for (std::map<std::string, std::vector<std::string> >::const_iterator itFilter = affFilters.begin();
       itFilter != affFilters.end(); ++itFilter) {  // for each roms
    hppDout(notice, "For rom : " << itFilter->first);
    for (std::vector<std::string>::const_iterator itType = itFilter->second.begin(); itType != itFilter->second.end();
         ++itType) {
      hppDout(notice, "aff type : " << *itType);
      if (addedTypes.empty() || (addedTypes.find(*itType) == addedTypes.end())) {
        hppDout(notice, "new type of affordance, add corresponding collision objects to the list");
        addedTypes.insert(*itType);
        if (affordances.map.find(*itType) != affordances.map.end()) {
          affMap_t::const_iterator itAff = affordances.map.find(*itType);
          for (AffordanceObjects_t::const_iterator itObj = itAff->second.begin(); itObj != itAff->second.end();
               ++itObj) {
            surfaces.push_back(itObj->second);
          }
        }
      }
    }
  }
  hppDout(notice, "final size of surfaces list : " << surfaces.size());
  return surfaces;
}

// TODO: outward

RbPrmShooter::RbPrmShooter(const pinocchio::RbPrmDevicePtr_t& robot, const hpp::core::ObjectStdVector_t& geometries,
                           const affMap_t& affordances, const std::vector<std::string>& filter,
                           const std::map<std::string, std::vector<std::string> >& affFilters,
                           const std::size_t shootLimit, const std::size_t displacementLimit)
    : shootLimit_(shootLimit),
      displacementLimit_(displacementLimit),
      filter_(filter),
      weights_(),
      triangles_(),
      robot_(robot),
      validator_(rbprm::RbPrmValidation::create(robot_, filter, affFilters, affordances, geometries)),
      uniformShooter_(core::configurationShooter::Uniform::create(robot)),
      ratioWeighted_(0.3) {
  for (hpp::core::ObjectStdVector_t::const_iterator cit = geometries.begin(); cit != geometries.end(); ++cit) {
    validator_->addObstacle(*cit);
  }
  this->InitWeightedTriangles(getUsedSurfaces(affordances, affFilters));
}

void RbPrmShooter::InitWeightedTriangles(const core::ObjectStdVector_t& geometries) {
  double sum = 0;
  for (core::ObjectStdVector_t::const_iterator objit = geometries.begin(); objit != geometries.end(); ++objit) {
    const pinocchio::FclConstCollisionObjectPtr_t colObj = (*objit)->fcl();
    BVHModelOBConst_Ptr_t model = GetModel(colObj);  // TODO NOT TRIANGLES
    for (int i = 0; i < model->num_tris; ++i) {
      TrianglePoints tri;
      Triangle fcltri = model->tri_indices[i];
      tri.p1 = colObj->getRotation() * model->vertices[fcltri[0]] + colObj->getTranslation();
      tri.p2 = colObj->getRotation() * model->vertices[fcltri[1]] + colObj->getTranslation();
      tri.p3 = colObj->getRotation() * model->vertices[fcltri[2]] + colObj->getTranslation();
      double weight = TriangleArea(tri);
      hppDout(notice, "Area of triangle = " << weight);
      sum += weight;
      weights_.push_back(weight);
      fcl::Vec3f normal = (tri.p2 - tri.p1).cross(tri.p3 - tri.p1);
      normal.normalize();
      triangles_.push_back(std::make_pair(normal, tri));
    }
  }
  double previousWeight = 0;
  hppDout(notice, "Sum of all areas of triangles : " << sum);
  for (std::vector<double>::iterator wit = weights_.begin(); wit != weights_.end(); ++wit) {
    previousWeight += (*wit) / sum;
    (*wit) = previousWeight;
    hppDout(notice, "current weight = " << previousWeight);
  }
  hppDout(notice, "number of triangle for the shooter : " << triangles_.size());
}

const RbPrmShooter::T_TriangleNormal& RbPrmShooter::RandomPointIntriangle() const {
  return triangles_[rand() % triangles_.size()];
}

const RbPrmShooter::T_TriangleNormal& RbPrmShooter::WeightedTriangle() const {
  double r = ((double)rand() / (RAND_MAX));
  std::vector<T_TriangleNormal>::const_iterator trit = triangles_.begin();
  for (std::vector<double>::const_iterator wit = weights_.begin(); wit != weights_.end(); ++wit, ++trit) {
    if (*wit >= r) return *trit;
  }
  return triangles_[triangles_.size() - 1];  // not supposed to happen
}

void RbPrmShooter::randConfigAtPos(const pinocchio::RbPrmDevicePtr_t robot, const std::vector<double>& eulerSo3,
                                   Configuration_t& config, const Vec3f p) const {
  uniformShooter_->shoot(config);
  SetConfigTranslation(robot, config, p);
  SampleRotation(eulerSo3, config);
}

fcl::Vec3f normalFromTriangleContact(const Contact& c, hpp::core::CollisionObjectConstPtr_t colObj) {
  int i = c.b2;
  TrianglePoints tri;
  BVHModelOBConst_Ptr_t model = GetModel(colObj->fcl());  // TODO NOT TRIANGLES
  fcl::Vec3f normal;
  if (model) {
    Triangle fcltri = model->tri_indices[i];
    tri.p1 = colObj->fcl()->getRotation() * model->vertices[fcltri[0]] + colObj->fcl()->getTranslation();
    tri.p2 = colObj->fcl()->getRotation() * model->vertices[fcltri[1]] + colObj->fcl()->getTranslation();
    tri.p3 = colObj->fcl()->getRotation() * model->vertices[fcltri[2]] + colObj->fcl()->getTranslation();
    normal = (tri.p2 - tri.p1).cross(tri.p3 - tri.p1);
  } else {
    hppDout(warning, "In shooter : cannot get contact normal, use z axis by default.");
    normal = fcl::Vec3f(0, 0, 1);
  }
  return normal.normalized();
}

void RbPrmShooter::impl_shoot(hpp::core::Configuration_t& config) const {
  hppDout(notice, "!!! Random shoot");
  HPP_DEFINE_TIMECOUNTER(SHOOT_COLLISION);
  uniformShooter_->shoot(config);
  std::size_t limit = shootLimit_;
  bool found(false);
  while (limit > 0 && !found) {
    // pick one triangle randomly
    const T_TriangleNormal* sampled(0);
    double r = ((double)rand() / (RAND_MAX));
    if (r > ratioWeighted_)
      sampled = &RandomPointIntriangle();
    else
      sampled = &WeightedTriangle();
    const TrianglePoints& tri = sampled->second;
    // http://stackoverflow.com/questions/4778147/sample-random-point-in-triangle
    double r1, r2;
    r1 = ((double)rand() / (RAND_MAX));
    r2 = ((double)rand() / (RAND_MAX));
    Vec3f p = (1 - sqrt(r1)) * tri.p1 + (sqrt(r1) * (1 - r2)) * tri.p2 + (sqrt(r1) * r2) * tri.p3;
    const Vec3f& n = sampled->first;

    // set configuration position to sampled point
    randConfigAtPos(robot_, eulerSo3_, config, p);
    // rotate and translate randomly until valid configuration found or
    // no obstacle is reachable
    ValidationReportPtr_t reportShPtr(new RbprmValidationReport);
    std::size_t limitDis = displacementLimit_;
    Vec3f lastDirection = n;
    while (!found && limitDis > 0) {
      HPP_START_TIMECOUNTER(SHOOT_COLLISION);
      found = validator_->validate(config, reportShPtr, filter_);
      RbprmValidationReportPtr_t report = std::dynamic_pointer_cast<RbprmValidationReport>(reportShPtr);
      bool valid = found || !report->trunkInCollision;
      HPP_STOP_TIMECOUNTER(SHOOT_COLLISION);

      if (valid & !found) {
        // try to rotate to reach rom
        for (; limitDis > 0 && !found && valid; --limitDis) {
          // SampleRotation(eulerSo3_, config);
          randConfigAtPos(robot_, eulerSo3_, config, p);
          HPP_START_TIMECOUNTER(SHOOT_COLLISION);
          found = validator_->validate(config, reportShPtr, filter_);
          HPP_STOP_TIMECOUNTER(SHOOT_COLLISION);
          if (!found) {
            Translate(robot_, config, -lastDirection * 0.2 * ((double)rand() / (RAND_MAX)));
          }
          HPP_START_TIMECOUNTER(SHOOT_COLLISION);
          found = validator_->validate(config, reportShPtr, filter_);
          report = std::dynamic_pointer_cast<RbprmValidationReport>(reportShPtr);
          valid = found || !report->trunkInCollision;
          // found = validator_->validate(config, filter_);
          HPP_STOP_TIMECOUNTER(SHOOT_COLLISION);
        }
        if (!found) break;
      } else if (!valid)  // move out of collision
      {
        // retrieve Contact information
        report = std::dynamic_pointer_cast<RbprmValidationReport>(reportShPtr);
        lastDirection = normalFromTriangleContact(report->result.getContact(0), report->object2);
        Translate(robot_, config, lastDirection * (std::abs(report->result.getContact(0).penetration_depth) + 0.03));
        limitDis--;
      }
    }
    limit--;
  }
  if (!found) std::cout << "no config found" << std::endl;
  hppDout(info, "shoot : " << pinocchio::displayConfig(config));
  HPP_DISPLAY_TIMECOUNTER(SHOOT_COLLISION);
}

void RbPrmShooter::sampleExtraDOF(bool sampleExtraDOF) { uniformShooter_->sampleExtraDOF(sampleExtraDOF); }

HPP_START_PARAMETER_DECLARATION(RbprmShooter)
Problem::declareParameter(
    core::ParameterDescription(core::Parameter::FLOAT, "RbprmShooter/ratioWeighted",
                               "The ratio used to select a random triangle with a weight proportional to it's area. "
                               "Otherwise the triangles are choosed uniformly. ",
                               core::Parameter(0.3)));
HPP_END_PARAMETER_DECLARATION(RbprmShooter)

}  // namespace rbprm
}  // namespace hpp
