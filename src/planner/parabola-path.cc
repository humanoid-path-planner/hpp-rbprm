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

#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/rbprm/planner/parabola-path.hh>
#include <hpp/core/straight-path.hh>

namespace hpp {
  namespace rbprm {
    using model::displayConfig;
    using core::value_type;
    using core::vector_t;
    using core::interval_t;
    using core::size_type;

    ParabolaPath::ParabolaPath (const core::DevicePtr_t& device,
                                core::ConfigurationIn_t init,
                                core::ConfigurationIn_t end,
                                value_type length,
                                vector_t coefs) :
      parent_t (interval_t (0, length), device->configSize (),
                device->numberDof ()), device_ (device), initial_ (init),
      end_ (end), coefficients_ (vector_t(3)), length_ (length),
      workspaceDim_ (false)
    {
      assert (device);
      /* Define dimension: 2D or 3D */
      std::string name = device->getJointVector () [0]->name ();
      if (name == "base_joint_xyz") {// 3D (2D by default)
        workspaceDim_ = true;
        coefficients_.resize (5);
      }
      coefficients (coefs);
    }

    ParabolaPath::ParabolaPath (const ParabolaPath& path) :
      parent_t (path), device_ (path.device_), initial_ (path.initial_),
      end_ (path.end_), coefficients_ (path.coefficients_),
      length_ (path.length_), workspaceDim_ (path.workspaceDim_)
    {
    }

    bool ParabolaPath::impl_compute (core::ConfigurationOut_t result,
                                     value_type param) const
    {

      //hppDout (info, "param: " << param);
      if (param == 0 || initial_(0) == end_(0)) {
        result = initial_;
        return true;
      }
      if (param == length_) {
        result = end_;
        return true;
      }

      const size_type nbConfig = device_->configSize();
      const size_type dirDim = device_->extraConfigSpace ().dimension ();
      const value_type u = param/length_;
      
      result (0) = (1 - u)*initial_(0) + u*end_(0);

      if (!workspaceDim_) { /* 2D */
        result (1) = coefficients_(0)*result (0)*result (0)
            + coefficients_(1)*result (0) + coefficients_(2);
        //hppDout (info, "x: " << result (0));
        //hppDout (info, "f(x) = " << result (1));

        result (nbConfig-dirDim) = (1 - u)
            * initial_(nbConfig-dirDim) + u*end_(nbConfig-dirDim);
        result (nbConfig-dirDim+1) = (1 - u)
            * initial_(nbConfig-dirDim+1) + u*end_(nbConfig-dirDim+1);
      }
      else { /* 3D */
        result (1) = tan(coefficients_(3))*result (0) + coefficients_(4);

        const value_type theta = coefficients_(3);
        const value_type x_theta = cos(theta)*result (0) +
            sin(theta)*result (1);
        const value_type x_theta_max = - 0.5 *
            coefficients_ (1) / coefficients_ (0);
        const value_type x_theta_initial = cos(theta)*initial_ (0) +
            sin(theta)*initial_ (1);
        const value_type x_theta_end = cos(theta)*end_ (0) +
            sin(theta)*end_ (1);
        const value_type u_max = (x_theta_max - x_theta_initial)
            / (x_theta_end - x_theta_initial);

        result (2) = coefficients_(0)*x_theta*x_theta
            + coefficients_(1)*x_theta + coefficients_(2);

        /* Quaternions interpolation */
        const core::JointPtr_t SO3joint = device_->getJointByName ("base_joint_SO3");
        const std::size_t rank = SO3joint->rankInConfiguration ();
        const core::size_type dimSO3 = SO3joint->configSize ();
        SO3joint->configuration ()->interpolate
            (initial_, end_, u, rank, result);

        /* if robot has internal DoF (except freeflyer ones) */
        // translation dimension of freeflyer hardcoded...
        // min value (to reach for u = u_max) hardcoded...
        // manual interpolation since joint not available with index...
        const value_type maxVal = 0; // because here initial_ = end_ ...
        if (nbConfig > dirDim + 3 + dimSO3) {
          for (core::size_type i = 7; i<nbConfig-dirDim; i++)
          {
            if (u <= u_max) {
              const value_type u_prime = u / u_max;
              result (i) = (1 - u_prime) * initial_ (i) + u_prime * maxVal;
            }
            else {
              const value_type u_prime = (u - u_max) / (1 - u_max);
              result (i) = (1 - u_prime) * maxVal + u_prime * end_ (i);
            }
          }
        }

        /* Normal vector interpolation
     result (nbConfig-dirDim) = (1 - u) *
     initial_(nbConfig-dirDim) + u*end_(nbConfig-dirDim);
     result (nbConfig-dirDim+1) = (1 - u) *
     initial_(nbConfig-dirDim+1) + u*end_(nbConfig-dirDim+1);
     result (nbConfig-dirDim+2) = (1 - u) *
     initial_(nbConfig-dirDim+2) + u*end_(nbConfig-dirDim+2);*/
      }
      return true;
    }


    core::PathPtr_t ParabolaPath::extract (const interval_t& subInterval) const throw (hpp::core::projection_error)
    {
      bool success;
      core::Configuration_t q1 ((*this) (subInterval.first, success)); // straight
      core::Configuration_t q2 ((*this) (subInterval.second, success)); // straight
      core::PathPtr_t result = rbprm::ParabolaPath::create(device_,q1,q2,computeLength(q1,q2),coefficients_);
      return result;
    }

    core::PathPtr_t ParabolaPath::reverse () const{
      hppDout(notice, "reverse path parabola !!!!!!!!!!!!!!!!!!!!!!!!");
      core::Configuration_t q1 ((*this) (length_));
      core::Configuration_t q2 ((*this) (0));
      core::PathPtr_t result = ParabolaPath::create (device_, q1, q2, length_,
                                                     coefficients_);
      return result;
    }

    core::DevicePtr_t ParabolaPath::device () const
    {
      return device_;
    }

    value_type ParabolaPath::computeLength
    (const core::ConfigurationIn_t q1, const core::ConfigurationIn_t q2) const {
      const int N = 6; // number -1 of interval sub-divisions
      // for N = 4, computation error ~= 1e-5.
      // for N = 20, computation error ~= 1e-11.
      value_type length = 0;
      value_type x1 = q1 (0);
      value_type x2 = q2 (0);

      if (workspaceDim_) { // 3D
        const value_type theta = coefficients_ (3);
        x1 = cos(theta) * q1 (0)  + sin(theta) * q1 (1); // x_theta_0
        x2 = cos(theta) * q2 (0) + sin(theta) * q2 (1); // x_theta_imp
      }

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
    }

    // Function equivalent to sqrt( 1 + f'(x)^2 )
    value_type ParabolaPath::lengthFunction (const value_type x)
    const {
      const value_type y = sqrt (1+(2*coefficients_ (0)*x+coefficients_(1))
                                 * (2*coefficients_ (0)*x+coefficients_(1)));
      return y;
    }
  } //   namespace rbprm
} // namespace hpp

