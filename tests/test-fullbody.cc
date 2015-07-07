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

#define BOOST_TEST_MODULE test-fullbody
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_SUITE( test_rbprm )

using namespace hpp;
using namespace hpp::model;
using namespace hpp::rbprm;
using namespace hpp::rbprm::sampling;


hpp::rbprm::RbPrmFullBodyPtr_t initFullBodyDevice()
{
    DevicePtr_t device = initDevice();
    hpp::rbprm::RbPrmFullBodyPtr_t robot =
            RbPrmFullBody::create(device);
    robot->AddLimb("elbow", 100, 0.1);
    return robot;
}

BOOST_AUTO_TEST_CASE (fullbody) {
    RbPrmFullBodyPtr_t fb = initFullBodyDevice();
}
BOOST_AUTO_TEST_SUITE_END()



