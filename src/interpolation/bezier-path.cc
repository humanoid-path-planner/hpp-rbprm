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

#include <hpp/rbprm/interpolation/spline/bezier-path.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/exception.hh>
#include <hpp/model/device.hh>
#include <hpp/model/configuration.hh>


namespace hpp {
  namespace rbprm {

  using core::Path;
  using core::DevicePtr_t;
  using core::ConfigurationIn_t;
  using core::ConfigurationOut_t;
  using model::value_type;
  using core::interval_t;


  BezierPath::BezierPath (const DevicePtr_t& /*robot*/, const bezier_Ptr& curve,interval_t timeRange):
      parent_t(timeRange,3,3),curve_(curve)
  {
    assert(timeRange.first>=curve_->min() && "The time range is outside the curve definition");
    assert(timeRange.second<=curve_->max() && "The time range is outside the curve definition");
  }


  BezierPath::BezierPath (const DevicePtr_t& robot, bezier_t::cit_point_t   wpBegin,bezier_t::cit_point_t wpEnd,interval_t timeRange):
      parent_t(timeRange,3,3),curve_(bezier_Ptr(new bezier_t(wpBegin,wpEnd,timeRange.second-timeRange.first)))
  {
    assert (timeRange.first == 0 && "Bezier path cannot be created from waypoint with initiale time different from 0");
  }


  BezierPath::BezierPath (const BezierPath& path) :
    parent_t (path),curve_(path.curve_)
    {}

  BezierPath::BezierPath (const BezierPath& path,
        const core::ConstraintSetPtr_t& constraints):
      parent_t (path,constraints),curve_(path.curve_)

  {}

  bool BezierPath::impl_compute(ConfigurationOut_t result,value_type t) const{
      if(t<timeRange().first){
          hppDout(warning,"Bezier path called with time outside time definition");
          result = initial();
          return true;
      }
      if(t>timeRange().second){
          hppDout(warning,"Bezier path called with time outside time definition");
          result =  end();
          return true;
      }
      result = (*curve_)(t);
      return true;
  }



  }//rbprm
}//hpp
