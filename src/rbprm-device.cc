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

      /*class rbprmexception : public std::exception
      {
      public:
          rbprmexception(const std::string& message)
              : exception()
              , message_(message) {}

          ~rbprmexception() throw(){}

          const char * what () const throw ()
          {
              return message_.c_str();
          }
      public:
          const std::string message_;
      };*/

    RbPrmDevicePtr_t RbPrmDevice::create (const std::string& name, DevicePtr_t& robotRom, DevicePtr_t &fullBody)
    {
        RbPrmDevice* rbprmDevice = new RbPrmDevice(name, robotRom, fullBody);
        RbPrmDevicePtr_t res (rbprmDevice);
        res->init (res);
        return res;
    }

    RbPrmDevice::~RbPrmDevice()
    {
        // NOTHING
    }

    // ========================================================================

    void RbPrmDevice::init(const RbPrmDeviceWkPtr_t& weakPtr)
    {
        Device::init (weakPtr);
        weakPtr_ = weakPtr;
    }

    bool RbPrmDevice::currentConfiguration (ConfigurationIn_t configuration)
    {
        Device::currentConfiguration(configuration);
        return robotRom_->currentConfiguration(configuration);
    }

    /*bool RbPrmDevice::setCurrentConfiguration (ConfigurationIn_t configuration)
    {
        Device::setCurrentConfiguration(configuration);
        return robotRom_->setCurrentConfiguration(configuration);
    }*/

    RbPrmDevice::RbPrmDevice (const std::string& name, const DevicePtr_t& robotRom, const DevicePtr_t& fullBody)
        : Device(name)
        , robotRom_(robotRom)
        , fullBody_(fullBody)
        , weakPtr_()
    {
        /*if(robotTrunk->configSize() != robotRom->configSize())
            throw rbprmexception(
                    "In RbPrmDevice initialization; trunk and rom must have the same dimensionality.");*/
    }
  } // model
} //hpp
