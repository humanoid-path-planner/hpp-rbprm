//
// Copyright (c) 2017 CNRS
// Authors: Pierre Fernbach
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/rbprm/interpolation/spline/bezier-path.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/exception.hh>

namespace hpp {
namespace rbprm {

using core::ConfigurationIn_t;
using core::ConfigurationOut_t;
using core::DevicePtr_t;
using core::interval_t;
using core::Path;
using pinocchio::value_type;

BezierPath::BezierPath(const DevicePtr_t& robot, const bezier_Ptr& curve,
                       core::ConfigurationIn_t init,
                       core::ConfigurationIn_t end, interval_t timeRange)
    : parent_t(timeRange, robot->configSize(), robot->numberDof()),
      device_(robot),
      curve_(curve),
      initial_(init),
      end_(end) {
  hppDout(notice, "Create a bezier path, with init config : "
                      << pinocchio::displayConfig(initial_));
  hppDout(notice, "                            end config : "
                      << pinocchio::displayConfig(end_));
  assert(timeRange.first >= curve_->min() &&
         "The time range is outside the curve definition");
  assert(timeRange.second <= curve_->max() &&
         "The time range is outside the curve definition");
}

BezierPath::BezierPath(const core::DevicePtr_t& robot,
                       std::vector<bezier_t::point_t>::const_iterator wpBegin,
                       std::vector<bezier_t::point_t>::const_iterator wpEnd,
                       core::ConfigurationIn_t init,
                       core::ConfigurationIn_t end, core::interval_t timeRange)
    : parent_t(timeRange, robot->configSize(), robot->numberDof()),
      device_(robot),
      curve_(bezier_Ptr(
          new bezier_t(wpBegin, wpEnd, timeRange.second - timeRange.first))),
      initial_(init),
      end_(end) {
  hppDout(notice, "Create a bezier path, with init config : "
                      << pinocchio::displayConfig(initial_));
  hppDout(notice, "                            end config : "
                      << pinocchio::displayConfig(end_));
  assert(timeRange.first == 0 &&
         "Bezier path cannot be created from waypoint with initiale time "
         "different from 0");
}

BezierPath::BezierPath(const BezierPath& path)
    : parent_t(path),
      device_(path.device_),
      curve_(path.curve_),
      initial_(path.initial_),
      end_(path.end_) {}

BezierPath::BezierPath(const BezierPath& path,
                       const core::ConstraintSetPtr_t& constraints)
    : parent_t(path, constraints),
      device_(path.device_),
      curve_(path.curve_),
      initial_(path.initial_),
      end_(path.end_)

{}

/// Get the initial configuration
core::Configuration_t BezierPath::initial() const {
  core::Configuration_t result(device_->configSize());
  core::value_type u =
      curve_->min() + timeRange().first / (curve_->max() - curve_->min());
  if (u < 0.)  // may happen because of float precision
    u = 0.;
  pinocchio::interpolate(device_, initial_, end_, u, result);
  result.head<3>() = (*curve_)(timeRange().first);
  return result;
}

/// Get the final configuration
core::Configuration_t BezierPath::end() const {
  core::Configuration_t result(device_->configSize());
  core::value_type u =
      curve_->min() + timeRange().second / (curve_->max() - curve_->min());
  if (u > 1.)  // may happen because of float precision
    u = 1.;
  pinocchio::interpolate(device_, initial_, end_, u, result);
  result.head<3>() = (*curve_)(timeRange().second);
  return result;
}

bool BezierPath::impl_compute(ConfigurationOut_t result, value_type t) const {
  if (t < timeRange().first) {
    hppDout(warning, "Bezier path called with time outside time definition");
    result = initial();
    return true;
  }
  if (t > timeRange().second) {
    hppDout(warning, "Bezier path called with time outside time definition");
    result = end();
    return true;
  }
  value_type u = curve_->min() + t / (curve_->max() - curve_->min());
  if (timeRange().second == 0) u = 0;
  pinocchio::interpolate(device_, initial_, end_, u, result);

  result.head<3>() = (*curve_)(t);
  return true;
}

}  // namespace rbprm
}  // namespace hpp
