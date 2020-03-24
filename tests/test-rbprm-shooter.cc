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
#include <hpp/rbprm/rbprm-shooter.hh>

#include <Eigen/Geometry>

#define BOOST_TEST_MODULE test - rbprm - shooter
#include <boost/test/included/unit_test.hpp>

using namespace hpp;
using namespace rbprm;

BOOST_AUTO_TEST_SUITE(test_rbprm_shooter)

BOOST_AUTO_TEST_CASE(shooterCreation) {
  RbPrmDevicePtr_t robot = initRbPrmDeviceTest();
  RbPrmValidationPtr_t validator(RbPrmValidation::create(robot));

  CollisionObjectPtr_t colObject = MeshObstacleBox();
  colObject->move(fcl::Vec3f(11.3, 0, 0));
  validator->addObstacle(colObject);

  core::ObjectStdVector_t collisionObjects;
  collisionObjects.push_back(colObject);
  RbPrmShooterPtr_t shooter = RbPrmShooter::create(robot, collisionObjects);
  for (int i = 0; i < 100; ++i) {
    BOOST_CHECK_MESSAGE(validator->validate(*(shooter->shoot())),
                        "Reachability condition should be verified by shooter");
  }
}

BOOST_AUTO_TEST_CASE(shooterCreationWithFilters) {
  RbPrmDevicePtr_t robot = initRbPrmDeviceTest();
  RbPrmValidationPtr_t validator(RbPrmValidation::create(robot));

  CollisionObjectPtr_t colObject = MeshObstacleBox();
  colObject->move(fcl::Vec3f(11.3, 0, 0));
  validator->addObstacle(colObject);

  core::ObjectStdVector_t collisionObjects;
  collisionObjects.push_back(colObject);

  std::vector<std::string> filter;

  RbPrmShooterPtr_t shooter = RbPrmShooter::create(robot, collisionObjects, filter);
  for (int i = 0; i < 100; ++i) {
    BOOST_CHECK_MESSAGE(validator->validate(*(shooter->shoot()), validationReport, filter),
                        "Reachability condition should be verified by shooter");
  }

  filter.push_back("rom");
  shooter = RbPrmShooter::create(robot, collisionObjects, filter);
  for (int i = 0; i < 100; ++i) {
    BOOST_CHECK_MESSAGE(validator->validate(*(shooter->shoot()), validationReport, filter),
                        "Reachability condition should be verified by shooter");
  }

  filter.push_back("rom2");
  shooter = RbPrmShooter::create(robot, collisionObjects, filter);
  for (int i = 0; i < 100; ++i) {
    BOOST_CHECK_MESSAGE(validator->validate(*(shooter->shoot()), validationReport, filter),
                        "Reachability condition should be verified by shooter");
  }

  filter.clear();
  filter.push_back("rom2");
  shooter = RbPrmShooter::create(robot, collisionObjects, filter);
  for (int i = 0; i < 100; ++i) {
    BOOST_CHECK_MESSAGE(validator->validate(*(shooter->shoot()), validationReport, filter),
                        "Reachability condition should be verified by shooter");
  }
}

// TODO: how to assert other than 0 angle?
BOOST_AUTO_TEST_CASE(shooterCreationWithSO3Limits) {
  RbPrmDevicePtr_t robot = initRbPrmDeviceTest();

  CollisionObjectPtr_t colObject = MeshObstacleBox();
  colObject->move(fcl::Vec3f(11.3, 0, 0));

  core::ObjectStdVector_t collisionObjects;
  collisionObjects.push_back(colObject);

  std::vector<std::string> filter;

  RbPrmShooterPtr_t shooter = RbPrmShooter::create(robot, collisionObjects, filter);
  std::vector<double> bounds;
  bounds.push_back(0);
  bounds.push_back(0);
  // bounds.push_back(-1);bounds.push_back(1);
  bounds.push_back(0);
  bounds.push_back(0);
  bounds.push_back(-1);
  bounds.push_back(1);
  // bounds.push_back(0);bounds.push_back(0);
  shooter->BoundSO3(bounds);
  for (int i = 0; i < 100; ++i) {
    Eigen::VectorXd config = *(shooter->shoot());
    Eigen::Quaterniond q(config(3), config(4), config(5), config(6));
    // Eigen::Vector3d ea = q.toRotationMatrix().eulerAngles(2, 1, 0);
    // std::cout << "q " << config << "\n ea " << ea << std::endl;
  }
}

BOOST_AUTO_TEST_SUITE_END()
