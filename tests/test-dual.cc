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

#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/weighed-distance.hh>

#define BOOST_TEST_MODULE test-dual
#include <boost/test/included/unit_test.hpp>

using hpp::model::Configuration_t;
using hpp::core::ConfigurationPtr_t;
using hpp::model::Device;
using hpp::model::DevicePtr_t;
using hpp::model::RbPrmDevice;
using hpp::model::RbPrmDevicePtr_t;

BOOST_AUTO_TEST_SUITE( test_rbprm )


BOOST_AUTO_TEST_CASE (dualCreation) {
    DevicePtr_t device1 = Device::create("trunk");
    DevicePtr_t device2 = Device::create("rom");
    RbPrmDevicePtr_t rbPrmDevice = RbPrmDevice::create(device1, device2);

}
BOOST_AUTO_TEST_SUITE_END()



