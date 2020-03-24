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
#include "hpp/rbprm/interpolation/rbprm-path-interpolation.hh"
#include "hpp/rbprm/rbprm-fullbody.hh"
#include "hpp/rbprm/rbprm-state.hh"
#include "hpp/core/straight-path.hh"
#include "hpp/rbprm/tools.hh"

#define BOOST_TEST_MODULE test - fullbody
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(test_rbprm)

using namespace hpp;
using namespace hpp::model;
using namespace hpp::rbprm;
using namespace hpp::rbprm::interpolation;

/*hpp::rbprm::RbPrmInterpolationPtr_t initInterpolation(const core::ObjectStdVector_t& collisionObjects)
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
    core::ObjectStdVector_t objects;
    objects.push_back(colObject);
    initInterpolation(objects);*/

void AddToState(const std::string& name, const fcl::Transform3f& transform, const fcl::Vec3f normal, State& state) {
  state.contacts_[name] = true;
  state.contactPositions_[name] = transform.getTranslation();
  state.contactNormals_[name] = normal;
  state.contactRotation_[name] = transform.getRotation();
}

void addState(const State& s1, T_StateFrame& states) { states.push_back(std::make_pair(states.size(), s1)); }

BOOST_AUTO_TEST_CASE(FilteringStates) {
  fcl::Vec3f nz(0, 0, 1);
  fcl::Vec3f ny(0, 1, 0);
  fcl::Transform3f id, x, y, z, rx;
  x.setTranslation(fcl::Vec3f(1, 0, 0));
  y.setTranslation(ny);
  z.setTranslation(nz);
  rx.setRotation(tools::GetRotationMatrix(nz, ny));

  T_StateFrame states;

  State s0;
  AddToState("2", y, nz, s0);

  /*// add x, then reposition x (normal)
  State s1;
  AddToState("1", x, nz, s1);
  AddToState("2", y, nz, s1);

  State s2;
  AddToState("1", x, ny, s2);
  AddToState("2", y, nz, s2);
  addState(s0, states);
  addState(s1, states);
  addState(s2, states);
  std::cout << FilterStates(states).size() << std::endl;
  assert(FilterStates(states).size() == 2);
  assert(FilterStates(states).back().first == 2); // middle state is removed */

  // add x, then reposition x (transform)
  State s3;
  AddToState("1", x, nz, s3);
  AddToState("2", y, nz, s3);

  State s4;
  AddToState("1", rx, nz, s4);
  AddToState("2", y, nz, s4);
  states.clear();
  addState(s0, states);
  addState(s3, states);
  addState(s4, states);
  BOOST_CHECK_MESSAGE(FilterStates(states, true).size() == 2, "State list not filtered");
  BOOST_CHECK_MESSAGE((FilterStates(states, true).back().first == 2),
                      "middle state not filtered");  // middle state is removed

  // add x two times, then reposition x (transform)
  State s4a;
  AddToState("1", x, nz, s4a);
  AddToState("2", y, nz, s4a);

  addState(s4a, states);
  BOOST_CHECK(FilterStates(states, true).size() == 2);
  BOOST_CHECK(FilterStates(states, true).back().first == 3);  // middle state is removed

  // x then y => no shortcut
  State s5;
  AddToState("1", x, nz, s5);
  AddToState("2", y, nz, s5);

  State s6;
  AddToState("1", x, nz, s6);
  AddToState("2", rx, nz, s6);
  states.clear();
  addState(s0, states);
  addState(s5, states);
  addState(s6, states);
  BOOST_CHECK(FilterStates(states, true).size() == 3);

  // break, then recreate
  State s7, s8;
  AddToState("2", x, nz, s8);
  states.clear();
  addState(s0, states);
  addState(s7, states);
  addState(s8, states);
  BOOST_CHECK(FilterStates(states, true).size() == 2);
  BOOST_CHECK(FilterStates(states, true).back().first == 2);  // middle state is removed

  // break, then recreate with other
  State s9, s10;
  AddToState("1", x, nz, s10);
  states.clear();
  addState(s0, states);
  addState(s9, states);
  addState(s10, states);
  BOOST_CHECK(FilterStates(states, true).size() == 3);

  // No contact variation
  states.clear();
  addState(s0, states);
  addState(s0, states);
  addState(s0, states);
  addState(s0, states);
  addState(s0, states);
  BOOST_CHECK(FilterStates(states, true).size() == 2);
}
BOOST_AUTO_TEST_SUITE_END()
