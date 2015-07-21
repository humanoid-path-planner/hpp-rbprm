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
#include "hpp/rbprm/rbprm-path-interpolation.hh"
#include "hpp/rbprm/rbprm-fullbody.hh"
#include "hpp/core/straight-path.hh"

#define BOOST_TEST_MODULE test-fullbody
#include <boost/test/included/unit_test.hpp>

//BOOST_AUTO_TEST_SUITE( test_rbprm )

using namespace hpp;
using namespace hpp::model;
using namespace hpp::rbprm;


/*hpp::rbprm::RbPrmInterpolationPtr_t initInterpolation(const ObjectVector_t& collisionObjects)
{
    DevicePtr_t device = initDevice();
    fcl::Vec3f offset(0,0,0);
    hpp::rbprm::RbPrmFullBodyPtr_t robot =
            RbPrmFullBody::create(device);
    robot->AddLimb("elbow",offset, collisionObjects, 1000, 0.1);

    State start, end;
    start.configuration_ = robot->device_->currentConfiguration();
    end.configuration_ = robot->device_->currentConfiguration();
    end.configuration_.head(3) = Eigen::Vector3d(1,0,0);
    core::PathPtr_t path = core::StraightPath::create(robot->device_,start.configuration_,end.configuration_,1);
    return RbPrmInterpolation::create(path, robot, start, end);
}

BOOST_AUTO_TEST_CASE (initInterpolationTest) {
    CollisionObjectPtr_t colObject = MeshObstacleBox();
    ObjectVector_t objects;
    objects.push_back(colObject);
    initInterpolation(objects);
}
BOOST_AUTO_TEST_SUITE_END()*/



