//
// Copyright (c) 2016 CNRS
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

#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/rbprm/planner/timed-parabola-path.hh>
#include <hpp/core/straight-path.hh>


// coefficient[3] = theta
// coefficient[4] = alpha

namespace hpp {
  namespace rbprm {
    using model::displayConfig;
    using core::value_type;
    using core::vector_t;
    using core::interval_t;
    using core::size_type;

    /// Constructor
    TimedParabolaPath::TimedParabolaPath (const core::DevicePtr_t& robot,
                  core::ConfigurationIn_t init,
                  core::ConfigurationIn_t end, ParabolaPathPtr_t parabolaPath):
    parent_t(interval_t (0, computeLength(parabolaPath)), robot->configSize (), robot->numberDof ()),
    device_ (robot), initial_ (init), end_ (end),parabolaPath_(parabolaPath),length_(computeLength(parabolaPath))
    {
    }


    /// Constructor
    TimedParabolaPath::TimedParabolaPath (const core::DevicePtr_t& robot,
                  core::ConfigurationIn_t init,
                  core::ConfigurationIn_t end, core::value_type length,
                  core::vector_t coefficients):
    parent_t(interval_t (0, length), robot->configSize (), robot->numberDof ()),
    device_ (robot), initial_ (init), end_ (end),
    parabolaPath_(ParabolaPath::create(robot,init,end,length,coefficients)),
    length_(computeLength(parabolaPath_))
    {
    }





    /// Constructor with velocities and ROMnames
    TimedParabolaPath::TimedParabolaPath (const core::DevicePtr_t& robot,
                  core::ConfigurationIn_t init,
                  core::ConfigurationIn_t end,
                  core::value_type length,
                  core::vector_t coefs,
                  core::vector_t V0, core::vector_t Vimp,
                  std::vector <std::string> initialROMnames,
                  std::vector <std::string> endROMnames):
      parent_t(interval_t (0, length), robot->configSize (), robot->numberDof ()),
      device_ (robot), initial_ (init), end_ (end),
      parabolaPath_(ParabolaPath::create(robot,init,end,length,coefs,V0,Vimp,initialROMnames,endROMnames)),
      length_(computeLength(parabolaPath_))
    {
    }


    /// Copy constructor
    TimedParabolaPath::TimedParabolaPath (const TimedParabolaPath& path):
      parent_t (path), device_ (path.device_), initial_ (path.initial_),
      end_ (path.end_), parabolaPath_(path.parabolaPath_),length_(path.length_)
    {
    }


    /// Extraction/Reversion of a sub-path
    /// \param subInterval interval of definition of the extract path
    /// If upper bound of subInterval is smaller than lower bound,
    /// result is reversed.
    core::PathPtr_t TimedParabolaPath::extract (const core::interval_t& subInterval) const throw (core::projection_error){
      //TODO
      return core::PathPtr_t();
    }

    /// Reversion of a path
    /// \return a new path that is this one reversed.
    core::PathPtr_t TimedParabolaPath::reverse () const{
      hppDout(notice, " ~ reverse timed path parabola !!!!!!!!!!!!!!!!!!!!!!");
      bool success;
      ParabolaPathPtr_t paraReverse = parabolaPath_->reverse();
      return TimedParabolaPath::create(device_,end_,initial_,paraReverse);
    }





    double TimedParabolaPath::computeLength(double x_theta, double v0, double alpha0){
      return x_theta/(v0*cos(alpha0));
    }

    double TimedParabolaPath::computeLength(ParabolaPathPtr_t parabolaPath){
      const value_type X = parabolaPath->end()[0] - parabolaPath->initial()[0];
      const value_type Y = parabolaPath->end()[1] - parabolaPath->initial()[1];;
      // theta = coef[3]
      const value_type X_theta = X*cos(parabolaPath->coefficients()[3]) + Y*sin(parabolaPath->coefficients()[3]);
      return computeLength(X_theta , parabolaPath->V0_.norm(),parabolaPath->coefficients()[4]);
    }


    bool TimedParabolaPath::impl_compute (core::ConfigurationOut_t result,
                                     value_type t) const
    {
      if (t == 0 || initial_(0) == end_(0)) {
        result = initial_;
        return true;
      }
      if (t >= length_) {
        result = end_;
        return true;
      }

      // TODO : compute u and call parabolaPath



      // TODO : compute extraDOF

    } // impl_compute


  } //rbprm
}//hpp
