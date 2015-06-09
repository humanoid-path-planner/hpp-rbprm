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

#define BOOST_TEST_MODULE test-rbprm-shooter
#include <boost/test/included/unit_test.hpp>

using namespace hpp;
using namespace rbprm;

BOOST_AUTO_TEST_SUITE( test_rbprm_shooter )


BOOST_AUTO_TEST_CASE (shooterCreation) {
    RbPrmDevicePtr_t robot = initRbPrmDeviceTest();
    RbPrmValidationPtr_t validator(RbPrmValidation::create(robot));

    CollisionObjectPtr_t colObject = MeshObstacleBox();
    colObject->move(fcl::Vec3f(11.3,0,0));
    validator->addObstacle(colObject);

    T_CollisionObject collisionObjects;
    collisionObjects.push_back(colObject->fcl());
    RbPrmShooterPtr_t shooter = RbPrmShooter::create(robot, collisionObjects, validator);
    for(int i =0; i< 100; ++i)
    {
        BOOST_CHECK_MESSAGE (validator->validate(*(shooter->shoot()), false),
                                                  "Reachability condition should be verified by shooter");
    }


}

BOOST_AUTO_TEST_SUITE_END()



