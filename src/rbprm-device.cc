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
namespace pinocchio {

RbPrmDevicePtr_t RbPrmDevice::create(const std::string& name, DevicePtr_t& robotRom) {
  hpp::pinocchio::T_Rom roms;
  roms.insert(std::make_pair(robotRom->name(), robotRom));
  RbPrmDevice* rbprmDevice = new RbPrmDevice(name, roms);
  RbPrmDevicePtr_t res(rbprmDevice);
  res->init(res);
  return res;
}

RbPrmDevicePtr_t RbPrmDevice::create(const std::string& name, const hpp::pinocchio::T_Rom& robotRoms) {
  RbPrmDevice* rbprmDevice = new RbPrmDevice(name, robotRoms);
  RbPrmDevicePtr_t res(rbprmDevice);
  res->init(res);
  return res;
}

RbPrmDevice::~RbPrmDevice() {
  // NOTHING
}

// ========================================================================

void RbPrmDevice::init(const RbPrmDeviceWkPtr_t& weakPtr) {
  Device::init(weakPtr);
  weakPtr_ = weakPtr;
}

bool RbPrmDevice::currentConfiguration(ConfigurationIn_t configuration) {
  for (hpp::pinocchio::T_Rom::const_iterator cit = robotRoms_.begin(); cit != robotRoms_.end(); ++cit) {
    cit->second->currentConfiguration(configuration);
  }
  return Device::currentConfiguration(configuration);
}

void RbPrmDevice::setDimensionExtraConfigSpace(const size_type& dimension) {
  Device::setDimensionExtraConfigSpace(dimension);  // call inherited method
  // call method for each robotRoms :
  for (hpp::pinocchio::T_Rom::const_iterator cit = robotRoms_.begin(); cit != robotRoms_.end(); ++cit) {
    cit->second->setDimensionExtraConfigSpace(dimension);
  }
}

vector3_t RbPrmDevice::getEffectorReference(std::string romName) {
  std::map<std::string, vector3_t>::iterator it = effectorsReferences_.find(romName);
  if (it == effectorsReferences_.end())
    return vector3_t();
  else
    return it->second;
}

void RbPrmDevice::setEffectorReference(std::string romName, vector3_t ref) {
  std::map<std::string, vector3_t>::iterator it = effectorsReferences_.find(romName);
  if (it == effectorsReferences_.end())
    effectorsReferences_.insert(std::make_pair(romName, ref));
  else
    it->second = ref;
}

RbPrmDevice::RbPrmDevice(const std::string& name, const hpp::pinocchio::T_Rom& robotRoms)
    : Device(name), robotRoms_(robotRoms), effectorsReferences_(), weakPtr_() {
  // NOTHING
}
}  // namespace pinocchio
}  // namespace hpp
