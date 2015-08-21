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

#define BOOST_TEST_MODULE test-device
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_SUITE( test_rbprm )


BOOST_AUTO_TEST_CASE (dualCreation) {
    initRbPrmDeviceTest();
}

/*BOOST_AUTO_TEST_CASE (dualCreationDifferentDofsInRobotFail) {
    DevicePtr_t trunk = Device::create("trunk");
    DevicePtr_t rom = Device::create("rom");
    JointSO3* jointSO3 = new JointSO3 (fcl::Transform3f());
    rom->rootJoint(jointSO3);
    BOOST_CHECK_THROW(RbPrmDevice::create(trunk, rom), std::exception);
    delete jointSO3;
}*/

BOOST_AUTO_TEST_CASE (dualCreationReachabilityCondition) {
    RbPrmDevicePtr_t robot = initRbPrmDeviceTest();
    RbPrmValidationPtr_t validator(RbPrmValidation::create(robot));

    CollisionGeometryPtr_t colGeom (new fcl::Box (1, 1, 1));
    CollisionObjectPtr_t colObject = CollisionObject::create(colGeom, fcl::Transform3f (), "obstacle");
    colObject->move(fcl::Vec3f(1.3,0,0));
    validator->addObstacle(colObject);
    BOOST_CHECK_MESSAGE (validator->validate(robot->Device::currentConfiguration()), "Reachability condition should be verified");

    colObject->move(fcl::Vec3f(0.5,0,0));
    BOOST_CHECK_MESSAGE (!validator->validate(robot->Device::currentConfiguration()),
                                              "Reachability condition should not be verified: collision with trunk");
    colObject->move(fcl::Vec3f(-0.5,0,0));
    BOOST_CHECK_MESSAGE (!validator->validate(robot->Device::currentConfiguration()),
                                              "Reachability condition should not be verified: no collision with rom");
}

BOOST_AUTO_TEST_CASE (dualCreationReachabilityConditionWithFilters) {
    RbPrmDevicePtr_t robot = initRbPrmDeviceTest();
    RbPrmValidationPtr_t validator(RbPrmValidation::create(robot));

    CollisionGeometryPtr_t colGeom (new fcl::Box (1, 1, 1));
    CollisionObjectPtr_t colObject = CollisionObject::create(colGeom, fcl::Transform3f (), "obstacle");
    colObject->move(fcl::Vec3f(1.3,0,0));
    validator->addObstacle(colObject);
    hpp::core::CollisionValidationReport validationReport;
    std::vector<std::string> filter;
    BOOST_CHECK_MESSAGE (validator->validate(robot->Device::currentConfiguration(),validationReport, filter), "Reachability condition should be verified");

    filter.push_back("rom2");
    BOOST_CHECK_MESSAGE (!validator->validate(robot->Device::currentConfiguration(),validationReport, filter), "Reachability condition should not be verified");

    colObject->move(fcl::Vec3f(-1.3,0,0));
    BOOST_CHECK_MESSAGE (validator->validate(robot->Device::currentConfiguration(), validationReport, filter),
                                              "Reachability condition should be verified: collision with rom2");

    filter.push_back("rom");
    BOOST_CHECK_MESSAGE (!validator->validate(robot->Device::currentConfiguration(), validationReport, filter),
                                              "Reachability condition should not be verified: no collision with rom");

    colObject->move(fcl::Vec3f(0,0,0));
    BOOST_CHECK_MESSAGE (!validator->validate(robot->Device::currentConfiguration(), validationReport, filter),
                                              "Reachability condition should not be verified: collision with trunk");

    BOOST_CHECK_MESSAGE (validator->validateRoms(robot->Device::currentConfiguration(), filter),
                                              "Reachability condition should be verified: collision with rom AND rom2");

    colObject->move(fcl::Vec3f(1.1,0,0));
    BOOST_CHECK_MESSAGE (validator->validate(robot->Device::currentConfiguration(), validationReport, filter),
                                              "Reachability condition should be verified: collision with rom and rom2");
}

BOOST_AUTO_TEST_CASE (dualCreationReachabilityConditionWithNormalFilters) {
    RbPrmDevicePtr_t robot = initRbPrmDeviceTest();
    std::vector<std::string> filter;
    std::map<std::string, hpp::rbprm::NormalFilter> normalFilter;
    hpp::rbprm::NormalFilter nFilter;
    normalFilter.insert(std::make_pair("rom2",nFilter));
    RbPrmValidationPtr_t validator(RbPrmValidation::create(robot, filter, normalFilter));

    CollisionObjectPtr_t colObject = MeshObstacleBox();
    validator->addObstacle(colObject);
    filter.push_back("rom2");
    colObject->move(fcl::Vec3f(0,0,1.3));
    BOOST_CHECK_MESSAGE (validator->validateRoms(robot->Device::currentConfiguration(), filter),
                                              "Reachability condition should be verified: collision with rom2");

    normalFilter.clear();
    nFilter.unConstrained_ = false;
    nFilter.normal_ = fcl::Vec3f(0,0,1);
    nFilter.range_ = 0.9;
    normalFilter.insert(std::make_pair("rom2",nFilter));
    validator = RbPrmValidation::create(robot, filter, normalFilter);
    validator->addObstacle(colObject);
    BOOST_CHECK_MESSAGE (!validator->validateRoms(robot->Device::currentConfiguration(), filter),
                                              "Reachability condition should not be verified: collision with rom2 but non z normal");


    colObject->move(fcl::Vec3f(-0.5,0,-0.5));
    BOOST_CHECK_MESSAGE (validator->validateRoms(robot->Device::currentConfiguration(), filter),
                                              "Reachability condition should be verified: collision with rom2 with z normal");
}

BOOST_AUTO_TEST_SUITE_END()



