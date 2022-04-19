// Copyright (C) 2014 LAAS-CNRS
// Author: Steve Tonneau
//
// This file is part of the hpp-rbprm.
//
// hpp-core is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// test-hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-core.  If not, see <http://www.gnu.org/licenses/>.

#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <boost/assign.hpp>
#include <hpp/fcl/fwd.hh>
#include <hpp/model/body.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/model/joint.hh>
#include <hpp/rbprm/rbprm-device.hh>
#include <hpp/rbprm/rbprm-validation.hh>

using fcl::BVHModel;
using fcl::CollisionGeometry;
using fcl::CollisionGeometryPtr_t;
using hpp::core::ConfigurationPtr_t;
using hpp::model::Body;
using hpp::model::BodyPtr_t;
using hpp::model::CollisionObject;
using hpp::model::CollisionObjectPtr_t;
using hpp::model::Configuration_t;
using hpp::model::Device;
using hpp::model::DevicePtr_t;
using hpp::model::JointPtr_t;
using hpp::model::JointSO3;
using hpp::model::JointTranslation;
using hpp::model::RbPrmDevice;
using hpp::model::RbPrmDevicePtr_t;
using hpp::rbprm::RbPrmValidation;
using hpp::rbprm::RbPrmValidationPtr_t;

namespace {
void InitGeometries(JointPtr_t romJoint, JointPtr_t romJoint2,
                    JointPtr_t trunkJoint) {
  CollisionGeometryPtr_t trunk(new fcl::Box(1, 1, 1));
  CollisionObjectPtr_t obstacleTrunk =
      CollisionObject::create(trunk, fcl::Transform3f(), "trunkbox");

  CollisionGeometryPtr_t rom(new fcl::Box(1, 1, 1));
  CollisionObjectPtr_t obstacleRom =
      CollisionObject::create(rom, fcl::Transform3f(), "rombox");

  CollisionGeometryPtr_t rom2(new fcl::Box(2.3, 1, 1));
  CollisionObjectPtr_t obstacleRom2 =
      CollisionObject::create(rom2, fcl::Transform3f(), "rombox2");

  obstacleTrunk->move(fcl::Vec3f(0, 0, 0));
  obstacleRom->move(fcl::Vec3f(0.5, 0, 0));
  obstacleRom2->move(fcl::Vec3f(-0.5, 0, 0));
  BodyPtr_t body = new Body;
  body->name("trunk");
  trunkJoint->setLinkedBody(body);
  body->addInnerObject(obstacleTrunk, true, true);
  body = new Body;
  body->name("rom");
  romJoint->setLinkedBody(body);
  body->addInnerObject(obstacleRom, true, true);
  body = new Body;
  body->name("rom2");
  romJoint2->setLinkedBody(body);
  body->addInnerObject(obstacleRom2, true, true);
}

RbPrmDevicePtr_t initRbPrmDeviceTest() {
  DevicePtr_t rom = Device::create("rom");
  DevicePtr_t rom2 = Device::create("rom2");
  hpp::model::T_Rom roms;
  roms.insert(std::make_pair(rom->name(), rom));
  roms.insert(std::make_pair(rom2->name(), rom2));
  RbPrmDevicePtr_t trunk = RbPrmDevice::create("trunk", roms);
  JointSO3* jointSO3Trunk = new JointSO3(fcl::Transform3f());
  JointSO3* jointSO3Rom = new JointSO3(fcl::Transform3f());
  jointSO3Trunk->isBounded(0, true);
  jointSO3Trunk->isBounded(1, true);
  jointSO3Trunk->isBounded(2, true);
  jointSO3Trunk->lowerBound(0, -3.);
  jointSO3Trunk->upperBound(0, 3.);
  jointSO3Trunk->lowerBound(1, -3.);
  jointSO3Trunk->upperBound(1, 3.);
  jointSO3Trunk->lowerBound(2, -3.);
  jointSO3Trunk->upperBound(2, 3.);

  jointSO3Rom->isBounded(0, true);
  jointSO3Rom->isBounded(1, true);
  jointSO3Rom->isBounded(2, true);
  jointSO3Rom->lowerBound(0, -3.);
  jointSO3Rom->upperBound(0, 3.);
  jointSO3Rom->lowerBound(1, -3.);
  jointSO3Rom->upperBound(1, 3.);
  jointSO3Rom->lowerBound(2, -3.);
  jointSO3Rom->upperBound(2, 3.);

  JointTranslation<3>* jointTrTrunk =
      new JointTranslation<3>(fcl::Transform3f());
  JointTranslation<3>* jointTrRom = new JointTranslation<3>(fcl::Transform3f());

  jointTrTrunk->isBounded(0, true);
  jointTrTrunk->isBounded(1, true);
  jointTrTrunk->isBounded(2, true);
  jointTrTrunk->lowerBound(0, -3.);
  jointTrTrunk->upperBound(0, 12.);
  jointTrTrunk->lowerBound(1, -3.);
  jointTrTrunk->upperBound(1, 3.);
  jointTrTrunk->lowerBound(2, -3.);
  jointTrTrunk->upperBound(2, 3.);

  jointTrRom->isBounded(0, true);
  jointTrRom->isBounded(1, true);
  jointTrRom->isBounded(2, true);
  jointTrRom->lowerBound(0, -3.);
  jointTrRom->upperBound(0, 3.);
  jointTrRom->lowerBound(1, -3.);
  jointTrRom->upperBound(1, 3.);
  jointTrRom->lowerBound(2, -3.);
  jointTrRom->upperBound(2, 3.);

  JointSO3* jointSO3Rom2 = new JointSO3(*jointSO3Rom);
  JointTranslation<3>* jointTrRom2 = new JointTranslation<3>(*jointTrRom);

  rom->rootJoint(jointTrRom);
  rom2->rootJoint(jointTrRom2);
  trunk->rootJoint(jointTrTrunk);
  jointTrRom->addChildJoint(jointSO3Rom);
  jointTrRom2->addChildJoint(jointSO3Rom2);
  jointTrTrunk->addChildJoint(jointSO3Trunk);
  InitGeometries(jointTrRom, jointTrRom2, jointTrTrunk);
  return trunk;
}

CollisionObjectPtr_t MeshObstacleBox() {
  BVHModel<fcl::OBBRSS>* m1 = new BVHModel<fcl::OBBRSS>;
  std::vector<fcl::Vec3f> p1;
  p1.push_back(fcl::Vec3f(1, -1, -1));
  p1.push_back(fcl::Vec3f(1, -1, 1));
  p1.push_back(fcl::Vec3f(-1, -1, 1));
  p1.push_back(fcl::Vec3f(-1, -1, -1));
  p1.push_back(fcl::Vec3f(1, 1, -1));
  p1.push_back(fcl::Vec3f(1, 1, 1));
  p1.push_back(fcl::Vec3f(-1, 1, 1));
  p1.push_back(fcl::Vec3f(-1, 1, -1));
  std::vector<fcl::Triangle> t1;
  t1.push_back(fcl::Triangle(1, 2, 3));
  t1.push_back(fcl::Triangle(7, 6, 5));
  t1.push_back(fcl::Triangle(4, 5, 1));
  t1.push_back(fcl::Triangle(5, 6, 2));
  t1.push_back(fcl::Triangle(2, 6, 7));
  t1.push_back(fcl::Triangle(0, 3, 7));
  t1.push_back(fcl::Triangle(0, 1, 3));
  t1.push_back(fcl::Triangle(4, 7, 5));
  t1.push_back(fcl::Triangle(0, 4, 1));
  t1.push_back(fcl::Triangle(1, 5, 2));
  t1.push_back(fcl::Triangle(3, 2, 7));
  t1.push_back(fcl::Triangle(4, 0, 7));
  m1->beginModel();
  m1->addSubModel(p1, t1);
  m1->endModel();
  CollisionGeometryPtr_t colGeom(m1);
  return CollisionObject::create(colGeom, fcl::Transform3f(), "obstacle");
}

DevicePtr_t initDevice() {
  DevicePtr_t rom = Device::create("rom");

  JointSO3* jointSO3Rom2 = new JointSO3(fcl::Transform3f(fcl::Vec3f(1, 0, 0)));

  jointSO3Rom2->name("elbow");
  jointSO3Rom2->isBounded(0, true);
  jointSO3Rom2->isBounded(1, true);
  jointSO3Rom2->isBounded(2, true);
  jointSO3Rom2->lowerBound(0, -3.);
  jointSO3Rom2->upperBound(0, 3.);
  jointSO3Rom2->lowerBound(1, -3.);
  jointSO3Rom2->upperBound(1, 3.);
  jointSO3Rom2->lowerBound(2, -3.);
  jointSO3Rom2->upperBound(2, 3.);

  JointSO3* jointSO3Rom = new JointSO3(fcl::Transform3f());

  jointSO3Rom->name("arm");
  jointSO3Rom->isBounded(0, true);
  jointSO3Rom->isBounded(1, true);
  jointSO3Rom->isBounded(2, true);
  jointSO3Rom->lowerBound(0, -3.);
  jointSO3Rom->upperBound(0, 3.);
  jointSO3Rom->lowerBound(1, -3.);
  jointSO3Rom->upperBound(1, 3.);
  jointSO3Rom->lowerBound(2, -3.);
  jointSO3Rom->upperBound(2, 3.);

  JointTranslation<3>* jointTrRom = new JointTranslation<3>(fcl::Transform3f());

  jointTrRom->isBounded(0, true);
  jointTrRom->isBounded(1, true);
  jointTrRom->isBounded(2, true);
  jointTrRom->lowerBound(0, 0.);
  jointTrRom->upperBound(0, 0.);
  jointTrRom->lowerBound(1, 0.);
  jointTrRom->upperBound(1, 0.);
  jointTrRom->lowerBound(2, 0.);
  jointTrRom->upperBound(2, 0.);

  rom->rootJoint(jointTrRom);
  jointTrRom->addChildJoint(jointSO3Rom);
  jointSO3Rom->addChildJoint(jointSO3Rom2);

  CollisionGeometryPtr_t romcg(new fcl::Box(1, 1, 1));
  CollisionObjectPtr_t obstacleRom =
      CollisionObject::create(romcg, fcl::Transform3f(), "rombox");

  obstacleRom->move(fcl::Vec3f(0, 0, 0));
  BodyPtr_t body = new Body;
  body->name("rom");
  jointTrRom->setLinkedBody(body);
  body->addInnerObject(obstacleRom, true, true);
  return rom;
}

}  // namespace
