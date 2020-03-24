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

#include "test-tools.hh"
#include "hpp/rbprm/rbprm-fullbody.hh"

#include "hpp/rbprm/stability/stability.hh"

#define BOOST_TEST_MODULE test-fullbody
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(test_rbprm)

using namespace hpp;
using namespace hpp::model;
using namespace hpp::rbprm;
using namespace hpp::rbprm::sampling;

hpp::rbprm::RbPrmFullBodyPtr_t initFullBodyDevice(const core::ObjectStdVector_t& collisionObjects) {
  DevicePtr_t device = initDevice();
  fcl::Vec3f offset(0, 0, 0);
  fcl::Vec3f limbOffset(0, 0, 0);
  fcl::Vec3f normal(0, 0, 1);
  hpp::rbprm::RbPrmFullBodyPtr_t robot = RbPrmFullBody::create(device);
  robot->AddLimb("1", "elbow", "", offset, limbOffset, normal, 0.1, 0.1, collisionObjects, 1000, "EFORT", 0.1);
  return robot;
}

BOOST_AUTO_TEST_CASE(fullbody) {
  // TODO This test can not pass as is (cannot project configuration)
  // UPDATE
  CollisionObjectPtr_t colObject = MeshObstacleBox();
  core::ObjectStdVector_t objects;
  objects.push_back(colObject);
  RbPrmFullBodyPtr_t fb = initFullBodyDevice(objects);
  const Configuration_t& configuration = fb->device_->currentConfiguration();
  hpp::rbprm::State state = ComputeContacts(fb, configuration, objects, fcl::Vec3f(1, 0, 0));
  BOOST_CHECK_MESSAGE(state.contacts_["elbow"], "limb should be in contact");
  BOOST_CHECK_MESSAGE(state.configuration_ != configuration, "Configuration should be altered by contact generation");
}
BOOST_AUTO_TEST_SUITE_END()
