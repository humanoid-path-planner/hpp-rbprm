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
#include "hpp/rbprm/contact_generation/contact_generation.hh"

#define BOOST_TEST_MODULE test-fullbody
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_SUITE( test_rbprm )

using namespace hpp;
using namespace hpp::model;
using namespace hpp::rbprm;
using namespace hpp::rbprm::contact;


void AddToState(const std::string& name, State& state)
{
    state.contacts_[name] = true;
    state.contactPositions_[name] = fcl::Vec3f(0,0,0);
    state.contactNormals_[name] = fcl::Vec3f(0,0,1);
    state.contactRotation_[name] = fcl::Matrix3f();
    state.contactOrder_.push(name);
}

void addState(const State& s1,T_StateFrame& states)
{
    states.push_back(std::make_pair(states.size(), s1));
}

BOOST_AUTO_TEST_CASE (maintain_combinatorial) {
    State state;
    AddToState("c1", state);
    AddToState("c2", state);
    AddToState("c3", state);

    Q_State q = contact::maintain_contacts_combinatorial(state,0);
    BOOST_CHECK_MESSAGE(q.size() == 1, "Expected 1 combinations");

    q = contact::maintain_contacts_combinatorial(state,1);
    BOOST_CHECK_MESSAGE(q.size() == 4, "Expected 4 combinations");

    q = contact::maintain_contacts_combinatorial(state,2);
    BOOST_CHECK_MESSAGE(q.size() == 10, "Expected 10 combinations");

    q = contact::maintain_contacts_combinatorial(state,3);
    BOOST_CHECK_MESSAGE(q.size() == 16, "Expected 16 combinations");

    q = contact::maintain_contacts_combinatorial(state,4);
    BOOST_CHECK_MESSAGE(q.size() == 16, "Expected 16 combinations");
}
BOOST_AUTO_TEST_SUITE_END()



