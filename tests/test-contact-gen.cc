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

#define BOOST_TEST_MODULE test - fullbody
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(test_rbprm)

using namespace hpp;
using namespace hpp::model;
using namespace hpp::rbprm;
using namespace hpp::rbprm::contact;

void AddToState(const std::string& name, State& state) {
  state.contacts_[name] = true;
  state.contactPositions_[name] = fcl::Vec3f(0, 0, 0);
  state.contactNormals_[name] = fcl::Vec3f(0, 0, 1);
  state.contactRotation_[name] = fcl::Matrix3f();
  state.contactOrder_.push(name);
}

void addState(const State& s1, T_StateFrame& states) { states.push_back(std::make_pair(states.size(), s1)); }

void __debug_print(const T_ContactState& que) {
  T_ContactState q = que;
  std::cout << "******** printing q of size " << q.size() << std::endl;
  while (!q.empty()) {
    ContactState cs = q.front();
    q.pop();
    std::cout << "\t one state " << std::endl;
    for (std::vector<std::string>::const_iterator cit = cs.second.begin(); cit != cs.second.end(); ++cit) {
      std::cout << "\t \t " << *cit << std::endl;
    }
    std::cout << "\t END one state " << std::endl;
  }
  std::cout << "end printing q of size ******** " << q.size() << std::endl;
}

BOOST_AUTO_TEST_CASE(maintain_combinatorial) {
  State state;
  AddToState("c1", state);
  AddToState("c2", state);
  AddToState("c3", state);

  Q_State q = contact::maintain_contacts_combinatorial(state, 0);
  BOOST_CHECK_MESSAGE(q.size() == 1, "Expected 1 combinations");

  q = contact::maintain_contacts_combinatorial(state, 1);
  BOOST_CHECK_MESSAGE(q.size() == 4, "Expected 4 combinations");

  q = contact::maintain_contacts_combinatorial(state, 2);
  BOOST_CHECK_MESSAGE(q.size() == 7, "Expected 7 combinations");

  q = contact::maintain_contacts_combinatorial(state, 3);
  BOOST_CHECK_MESSAGE(q.size() == 8, "Expected 8 combinations");

  q = contact::maintain_contacts_combinatorial(state, 4);
  BOOST_CHECK_MESSAGE(q.size() == 8, "Expected 8 combinations");
}

BOOST_AUTO_TEST_CASE(gen_combinatorial) {
  std::vector<std::string> allcontacts;
  State state;
  AddToState("c1", state);
  AddToState("c2", state);

  T_ContactState q = contact::gen_contacts_combinatorial(allcontacts, state, 1);
  //__debug_print(q);
  BOOST_CHECK_MESSAGE(q.size() == 1, "Expected 1 combinations");

  allcontacts.push_back("c3");
  allcontacts.push_back("c4");

  q = contact::gen_contacts_combinatorial(allcontacts, state, 0);
  //__debug_print(q);
  BOOST_CHECK_MESSAGE(q.size() == 1, "Expected 1 combinations");

  q = contact::gen_contacts_combinatorial(allcontacts, state, 1);
  //__debug_print(q);
  BOOST_CHECK_MESSAGE(q.size() == 3, "Expected 3 combinations");

  q = contact::gen_contacts_combinatorial(allcontacts, state, 2);
  //__debug_print(q);
  BOOST_CHECK_MESSAGE(q.size() == 5, "Expected 5 combinations");

  q = contact::gen_contacts_combinatorial(allcontacts, state, 3);
  //__debug_print(q);
  BOOST_CHECK_MESSAGE(q.size() == 5, "Expected 5 combinations");
}

BOOST_AUTO_TEST_SUITE_END()
