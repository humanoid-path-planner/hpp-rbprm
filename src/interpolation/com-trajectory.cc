//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
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

#include <hpp/rbprm/interpolation/com-trajectory.hh>
#include <hpp/rbprm/interpolation/time-constraint-utils.hh>
#include <hpp/model/device.hh>
#include <hpp/model/configuration.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/locked-joint.hh>

using namespace hpp::core;

namespace hpp {
  namespace rbprm {
  namespace interpolation{
    ComTrajectory::ComTrajectory (model::vector3_t init,
                                  model::vector3_t end,
                                  model::vector3_t initSpeed,
                                  model::vector3_t acceleration,
                                  core::value_type length) :
        parent_t (interval_t (0, length), 3,3),
        initial_ (init), end_ (end),
        initSpeed_(initSpeed),
        half_acceleration_(acceleration / 2),
        length_(length)
      {
        assert (length >= 0);
        assert (!constraints ());
      }


    model::value_type normalize(const ComTrajectory& path, model::value_type param)
    {
        value_type u;
        if (path.timeRange ().second == 0)
            u = 0;
        else
            u = (param - path.timeRange ().first) / (path.timeRange ().second - path.timeRange().first);
        return u;
    }

    ComTrajectory::ComTrajectory (const ComTrajectory& path) :
        parent_t (interval_t (0, path.length_), 3,3),
        initial_ (path.initial_), end_ (path.end_),
        initSpeed_(path.initSpeed_),
        half_acceleration_(path.half_acceleration_),
        length_(path.length_){}

    bool ComTrajectory::impl_compute (ConfigurationOut_t result,
				     value_type param) const
    {
        if (param == timeRange ().first || timeRange ().second == 0)
        {
            result = initial();
        }
        else if (param == timeRange ().second)
        {
            result = end();
        }
        else
        {
            value_type u = normalize(*this, param) * length_;
            result = half_acceleration_ * u*u + initSpeed_ * u + initial_;
        }
        return true;
    }

    PathPtr_t ComTrajectory::extract (const interval_t& subInterval) const
      throw (projection_error)
    {
        // Length is assumed to be proportional to interval range
        value_type l = fabs (subInterval.second - subInterval.first);

        bool success;
        Configuration_t q1 ((*this) (subInterval.first, success));
        Configuration_t q2 ((*this) (subInterval.second, success));
        value_type u = normalize(*this, subInterval.first) * length_;
        model::vector3_t acceleration = half_acceleration_ * 2;
        model::vector3_t speedAtQ1 = acceleration * u + initSpeed_;
        PathPtr_t result = ComTrajectory::create (q1, q2, speedAtQ1, acceleration, l);
        return result;
    }
  } //   namespace interpolation
  } //   namespace rbprm
} // namespace hpp

