// Copyright (C) 2014 LAAS-CNRS
// Author: Mathieu Geisert
//
// This file is part of the hpp-core.
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

#include <boost/assign.hpp>

#include <hpp/rbprm/rbprm-device.hh>
#include <hpp/rbprm/rbprm-validation.hh>
#include <hpp/model/joint.hh>
#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/model/collision-object.hh>
#include <hpp/model/body.hh>

#define BOOST_TEST_MODULE test-device
#include <boost/test/included/unit_test.hpp>

using hpp::model::Configuration_t;
using hpp::core::ConfigurationPtr_t;
using hpp::model::Device;
using hpp::model::DevicePtr_t;
using hpp::model::RbPrmDevice;
using hpp::model::RbPrmDevicePtr_t;
using hpp::model::JointPtr_t;
using hpp::model::JointSO3;
using hpp::model::Body;
using hpp::model::BodyPtr_t;
using hpp::model::JointTranslation;
using hpp::model::CollisionObject;
using hpp::model::CollisionObjectPtr_t;
using fcl::CollisionGeometry;
using fcl::CollisionGeometryPtr_t;
using hpp::rbprm::RbPrmValidation;
using hpp::rbprm::RbPrmValidationPtr_t;


namespace
{
    void InitGeometries(JointPtr_t romJoint, JointPtr_t trunkJoint)
    {
        CollisionGeometryPtr_t trunk (new fcl::Box (1, 1, 1));
        CollisionObjectPtr_t obstacleTrunk = CollisionObject::create
            (trunk, fcl::Transform3f (), "trunkbox");

        CollisionGeometryPtr_t rom (new fcl::Box (1, 1, 1));
        CollisionObjectPtr_t obstacleRom = CollisionObject::create
            (rom, fcl::Transform3f (), "rombox");

        obstacleTrunk->move(fcl::Vec3f(0,0,0));
        obstacleRom->move(fcl::Vec3f(0.5,0,0));
        BodyPtr_t body = new Body;
        body->name ("trunk");
        trunkJoint->setLinkedBody (body);
        body->addInnerObject(obstacleTrunk, true, true);
        body = new Body;
        body->name ("rom");
        romJoint->setLinkedBody (body);
        body->addInnerObject(obstacleRom, true, true);
    }

    RbPrmDevicePtr_t initRbPrmDeviceTest()
    {
        DevicePtr_t trunk = Device::create("trunk");
        DevicePtr_t rom = Device::create("rom");
        JointSO3* jointSO3Trunk = new JointSO3 (fcl::Transform3f());
        JointSO3* jointSO3Rom = new JointSO3 (fcl::Transform3f());
        JointTranslation<3>* jointTrTrunk = new JointTranslation<3> (fcl::Transform3f());
        JointTranslation<3>* jointTrRom = new JointTranslation<3> (fcl::Transform3f());
        rom->rootJoint(jointTrRom);
        trunk->rootJoint(jointTrTrunk);
        jointTrRom->addChildJoint (jointSO3Rom);
        jointTrTrunk->addChildJoint (jointSO3Trunk);
        InitGeometries(jointTrRom, jointTrTrunk);
        RbPrmDevicePtr_t rbPrmDevice = RbPrmDevice::create(trunk, rom);
        return rbPrmDevice;
    }
} // namespace

BOOST_AUTO_TEST_SUITE( test_rbprm )


BOOST_AUTO_TEST_CASE (dualCreation) {
    initRbPrmDeviceTest();
}

BOOST_AUTO_TEST_CASE (dualCreationDifferentDofsInRobotFail) {
    DevicePtr_t trunk = Device::create("trunk");
    DevicePtr_t rom = Device::create("rom");
    JointSO3* jointSO3 = new JointSO3 (fcl::Transform3f());
    rom->rootJoint(jointSO3);
    BOOST_CHECK_THROW(RbPrmDevice::create(trunk, rom), std::exception);
    delete jointSO3;
}

BOOST_AUTO_TEST_CASE (dualCreationReachabilityCondition) {
    RbPrmDevicePtr_t robot = initRbPrmDeviceTest();
    RbPrmValidationPtr_t validator(RbPrmValidation::create(robot));

    CollisionGeometryPtr_t colGeom (new fcl::Box (1, 1, 1));
    CollisionObjectPtr_t colObject = CollisionObject::create(colGeom, fcl::Transform3f (), "obstacle");
    colObject->move(fcl::Vec3f(1.3,0,0));
    validator->addObstacle(colObject);
    BOOST_CHECK_MESSAGE (validator->validate(robot->currentConfiguration()), "Reachability condition should be verified");

    colObject->move(fcl::Vec3f(0.5,0,0));
    BOOST_CHECK_MESSAGE (!validator->validate(robot->currentConfiguration()),
                                              "Reachability condition should not be verified: collision with trunk");
    colObject->move(fcl::Vec3f(-0.5,0,0));
    BOOST_CHECK_MESSAGE (!validator->validate(robot->currentConfiguration()),
                                              "Reachability condition should not be verified: no collision with rom");
}

BOOST_AUTO_TEST_SUITE_END()



