//
// Copyright (c) 2014 CNRS
// Authors: Mylene Campana
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

#include <hpp/core/config-projector.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/rbprm/planner/parabola-path.hh>
#include <hpp/util/debug.hh>

namespace hpp {
namespace rbprm {
using core::interval_t;
using core::size_type;
using core::value_type;
using core::vector_t;
using pinocchio::displayConfig;

ParabolaPath::ParabolaPath(const core::DevicePtr_t& device,
                           core::ConfigurationIn_t init,
                           core::ConfigurationIn_t end, value_type length,
                           vector_t coefs)
    : parent_t(interval_t(0, length), device->configSize(),
               device->numberDof()),
      V0_(vector_t(3)),
      Vimp_(vector_t(3)),
      device_(device),
      initial_(init),
      end_(end),
      coefficients_(vector_t(coefs.size())),
      length_(length)

{
  assert(device);
  coefficients(coefs);
  initialROMnames_.reserve(10);
  endROMnames_.reserve(10);
}

ParabolaPath::ParabolaPath(const core::DevicePtr_t& device,
                           core::ConfigurationIn_t init,
                           core::ConfigurationIn_t end, value_type length,
                           vector_t coefs, vector_t V0, vector_t Vimp,
                           std::vector<std::string> initialROMnames,
                           std::vector<std::string> endROMnames)
    : parent_t(interval_t(0, length), device->configSize(),
               device->numberDof()),
      V0_(V0),
      Vimp_(Vimp),
      initialROMnames_(initialROMnames),
      endROMnames_(endROMnames),
      device_(device),
      initial_(init),
      end_(end),
      coefficients_(vector_t(coefs.size())),
      length_(length) {
  assert(device);
  coefficients(coefs);
  hppDout(info, "V0_= " << V0_.transpose() << " Vimp_= " << Vimp_.transpose());
  hppDout(info, "initialROMnames size= " << initialROMnames_.size());
}

ParabolaPath::ParabolaPath(const ParabolaPath& path)
    : parent_t(path),
      V0_(path.V0_),
      Vimp_(path.Vimp_),
      initialROMnames_(path.initialROMnames_),
      endROMnames_(path.endROMnames_),
      device_(path.device_),
      initial_(path.initial_),
      end_(path.end_),
      coefficients_(path.coefficients_),
      length_(path.length_) {
  hppDout(info, "V0_= " << V0_.transpose() << " Vimp_= " << Vimp_.transpose());
  hppDout(info, "initialROMnames size= " << initialROMnames_.size());
}

bool ParabolaPath::impl_compute(core::ConfigurationOut_t result,
                                value_type param) const {
  if (param == 0 || initial_(0) == end_(0)) {
    result = initial_;
    return true;
  }
  if (param >= length_) {
    result = end_;
    return true;
  }

  const size_type nbConfig = device_->configSize();
  const size_type ecsDim = device_->extraConfigSpace().dimension();
  // param = x_theta
  const value_type u = param / length_;
  const value_type theta = coefficients_(3);
  /* const value_type x_theta_max = - 0.5 *
       coefficients_ (1) / coefficients_ (0);
   const value_type x_theta_initial = cos(theta)*initial_ (0) +
       sin(theta)*initial_ (1);
   const value_type x_theta_end = cos(theta)*end_ (0) +
       sin(theta)*end_ (1);
   const bool tanThetaNotDefined = (theta < M_PI/2 + 1e-2 && theta > M_PI/2 -
   1e-2) || (theta > -M_PI/2 - 1e-2 && theta < -M_PI/2 + 1e-2);
*/
  result(0) = initial_(0) + u * length_ * cos(theta);
  result(1) = initial_(1) + u * length_ * sin(theta);
  const value_type x_theta = cos(theta) * result(0) + sin(theta) * result(1);
  result(2) = coefficients_(0) * x_theta * x_theta +
              coefficients_(1) * x_theta + coefficients_(2);

  pinocchio::interpolate(device_, initial_, end_, u, result);
  /* Quaternions interpolation */
  /*const core::JointPtr_t SO3joint = device_->getJointByName
  ("base_joint_SO3"); const std::size_t rank = SO3joint->rankInConfiguration ();
  const core::size_type dimSO3 = SO3joint->configSize ();
  SO3joint->configuration ()->interpolate
      (initial_, end_, u, rank, result);

  // if robot-trunk has internal DoF (except freeflyer ones)
  // then linear interpolation on them
  const std::size_t freeflyerDim = 3 + dimSO3;
  const bool hasInternalDof = nbConfig > ecsDim + freeflyerDim;
  if (hasInternalDof) {
    for (core::size_type i = freeflyerDim; i<nbConfig-ecsDim; i++) {
      result (i) = (1 - u) * initial_ (i) + u * end_ (i);
    }
  }*/

  /* Set to zero extra-configs (only on path, not on extremities */
  const std::size_t indexECS = nbConfig - ecsDim;
  for (size_type i = 0; i < ecsDim; i++) result(indexECS + i) = 0;
  return true;
}

core::PathPtr_t ParabolaPath::extract(const interval_t& subInterval) const {
  hppDout(error, "path extract is not recommended on parabola path");
  bool success;
  core::Configuration_t q1((*this)(subInterval.first, success));   // straight
  core::Configuration_t q2((*this)(subInterval.second, success));  // straight
  ParabolaPathPtr_t result = rbprm::ParabolaPath::create(
      device_, q1, q2, computeLength(q1, q2), coefficients_, V0_, Vimp_,
      initialROMnames_, endROMnames_);
  hppDout(info, "initialROMnames size= " << (*result).initialROMnames_.size());
  return result;
}

core::PathPtr_t ParabolaPath::reverse() const {
  hppDout(notice, " ~ reverse path parabola !!!!!!!!!!!!!!!!!!!!!!");
  bool success;
  core::Configuration_t q1((*this)(length_, success));
  core::Configuration_t q2((*this)(0, success));
  ParabolaPathPtr_t result =
      ParabolaPath::create(device_, q1, q2, length_, coefficients_, Vimp_, V0_,
                           endROMnames_, initialROMnames_);
  hppDout(info, "V0_= " << V0_.transpose() << " Vimp_= " << Vimp_.transpose());
  hppDout(info, "result V0_= " << (*result).V0_.transpose() << " result Vimp_= "
                               << (*result).Vimp_.transpose());
  hppDout(info,
          "path->initialROMnames size= " << (*result).initialROMnames_.size());
  hppDout(info,
          "this->initialROMnames size= " << (*this).initialROMnames_.size());
  hppDout(info, "result->initialROMnames size= "
                    << (*result).initialROMnames_.size());
  return result;
}

core::DevicePtr_t ParabolaPath::device() const { return device_; }

/*  value_type ParabolaPath::computeLength
  (const core::ConfigurationIn_t q1, const core::ConfigurationIn_t q2) const {
    const int N = 6; // number -1 of interval sub-divisions
    // for N = 4, computation error ~= 1e-5.
    // for N = 20, computation error ~= 1e-11.
    value_type length = 0;
    const value_type theta = coefficients_ (3);
    value_type x1 = cos(theta) * q1 (0)  + sin(theta) * q1 (1); // x_theta_0
    value_type x2 = cos(theta) * q2 (0) + sin(theta) * q2 (1); // x_theta_imp
    hppDout(notice,"xTheta0 = "<<x1 << " , "<<coefficients_[6]<<"   xThetaImp =
  "<<x2);

    // Define integration bounds
    if (x1 > x2) { // re-order integration bounds
      const value_type xtmp = x1;
      x1 = x2;
      x2 = xtmp;
    }

    const value_type dx = (x2 - x1) / N; // integration step size
    for (int i=0; i<N; i++) {
      length += dx*( 0.166666667*lengthFunction (x1 + i*dx)
                     + 0.666666667*lengthFunction (x1 + (i+0.5)*dx )
                     + 0.166666667*lengthFunction (x1 + (i+1)*dx ));
      // apparently, 1/6 and 2/3 are not recognized as floats ...
    }
    hppDout (notice, "length = " << length);
    return length;
  }*/

// test (pierre) :
value_type ParabolaPath::computeLength(const core::ConfigurationIn_t q1,
                                       const core::ConfigurationIn_t q2) const {
  const value_type theta = coefficients_(3);
  const value_type X = q2[0] - q1[0];
  const value_type Y = q2[1] - q1[1];
  ;
  // theta = coef[3]
  const value_type X_theta = X * cos(theta) + Y * sin(theta);
  return X_theta;
}

// Function equivalent to sqrt( 1 + f'(x)^2 )
value_type ParabolaPath::lengthFunction(const value_type x) const {
  const value_type y =
      sqrt(1 + (2 * coefficients_(0) * x + coefficients_(1)) *
                   (2 * coefficients_(0) * x + coefficients_(1)));
  return y;
}

vector_t ParabolaPath::evaluateVelocity(const value_type t) const {
  vector_t vel(3);
  bool success;
  const value_type theta = coefficients_(3);
  const value_type alpha = coefficients_(4);
  const value_type x_theta_0_dot = coefficients_(5);
  const value_type inv_x_theta_0_dot_sq = 1 / (x_theta_0_dot * x_theta_0_dot);
  const value_type x_theta_0 = coefficients_(6);
  const core::Configuration_t q = (*this)(t, success);
  const value_type x_theta = q[0] * cos(theta) + q[1] * sin(theta);
  vel[0] = x_theta_0_dot * cos(theta);
  vel[1] = x_theta_0_dot * sin(theta);
  vel[2] = x_theta_0_dot *
           (-9.81 * (x_theta - x_theta_0) * inv_x_theta_0_dot_sq + tan(alpha));
  return vel;
}

}  //   namespace rbprm
}  // namespace hpp
