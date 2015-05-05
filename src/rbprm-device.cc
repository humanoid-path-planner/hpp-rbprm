// Copyright (c) 2014, LAAS-CNRS
// Authors: Steve Tonneau (steve.tonneau@laas.fr)
//
// This file is part of hpp-rbprm.
// hpp-rbprm is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-rbprm is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-rbprm. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/rbprm/rbprm-device.hh>

namespace hpp {
  namespace model {

    RbPrmDevicePtr_t RbPrmDevice::create (const DevicePtr_t& robotTrunk, const DevicePtr_t& robotRom) throw()
    {
        RbPrmDevice* res = new RbPrmDevice(robotTrunk, robotRom);
        return RbPrmDevicePtr_t(res);
    }

    RbPrmDevice::RbPrmDevice (const DevicePtr_t& robotTrunk, const DevicePtr_t& robotRom) throw()
        : robotTrunk_(robotTrunk)
        , robotRom_(robotRom)
    {
        if(robotTrunk->configSize() != robotRom->configSize())
            throw std::runtime_error(
                    "In RbPrmDevice initialization; trunk and rom must have the same dimensionality.");
    }

    bool RbPrmDevice::currentConfiguration (ConfigurationIn_t configuration)
    {
        robotTrunk_->currentConfiguration(configuration);
        return robotRom_->currentConfiguration(configuration);
    }

    const Configuration_t& RbPrmDevice::currentConfiguration () const
    {
        return robotTrunk_->currentConfiguration();
    }
  } // model
} //hpp
