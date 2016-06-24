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

#include <hpp/rbprm/interpolation/limb-rrt-path.hh>
#include <hpp/model/device.hh>
#include <hpp/model/configuration.hh>

using namespace hpp::core;

namespace hpp {
  namespace rbprm {
  namespace interpolation{
    LimbRRTPath::LimbRRTPath (const DevicePtr_t& device,
                              ConfigurationIn_t init,
                              ConfigurationIn_t end,
                              value_type length,
                              const std::size_t pathDofRank) :
        parent_t (interval_t (0, length), device->configSize (),
                  device->numberDof ()),
        device_ (device), initial_ (init), end_ (end),
        pathDofRank_(pathDofRank)
      {
        assert (device);
        assert (length >= 0);
        assert (!constraints ());
      }

    LimbRRTPath::LimbRRTPath (const DevicePtr_t& device,
				ConfigurationIn_t init,
                ConfigurationIn_t end,
                value_type length,
                ConstraintSetPtr_t constraints,
                const std::size_t pathDofRank) :
        parent_t (interval_t (0, length), device->configSize (),
                  device->numberDof (), constraints),
        device_ (device), initial_ (init), end_ (end),
        pathDofRank_(pathDofRank)
    {
        assert (device);
        assert (length >= 0);
    }

    LimbRRTPath::LimbRRTPath (const LimbRRTPath& path) :
        parent_t (path), device_ (path.device_), initial_ (path.initial_),
        end_ (path.end_), pathDofRank_(path.pathDofRank_)
    {
    }

    LimbRRTPath::LimbRRTPath (const LimbRRTPath& path,
                const ConstraintSetPtr_t& constraints) :
        parent_t (path, constraints), device_ (path.device_),
        initial_ (path.initial_), end_ (path.end_), pathDofRank_(path.pathDofRank_)
    {
        // NOTHING
    }

    model::value_type ComputeExtraDofValue(const std::size_t dofRank,
                              const Configuration_t init,
                              const Configuration_t end,
                              const model::value_type normalizedValue)
    {
        double a = init[dofRank];
        double b = end[dofRank];
        return (b-a)* normalizedValue + a;
    }

    bool LimbRRTPath::impl_compute (ConfigurationOut_t result,
				     value_type param) const
    {
        if (param == timeRange ().first || timeRange ().second == 0)
        {
            result = initial_;
            return true;
        }
        if (param == timeRange ().second)
        {
            result = end_;
            return true;
        }
        value_type u = param/timeRange ().second;
        if (timeRange ().second == 0)
            u = 0;
        model::interpolate (device_, initial_, end_, u, result);
        result[pathDofRank_] = ComputeExtraDofValue(pathDofRank_,initial_, end_, u);
        return true;
    }

    PathPtr_t LimbRRTPath::extract (const interval_t& subInterval) const
      throw (projection_error)
    {
        // Length is assumed to be proportional to interval range
        value_type l = fabs (subInterval.second - subInterval.first);

        bool success;
        Configuration_t q1 ((*this) (subInterval.first, success));
        if (!success) throw projection_error
                ("Failed to apply constraints in StraightPath::extract");        
        q1[pathDofRank_] = ComputeExtraDofValue(pathDofRank_,initial_, end_, (subInterval.first - timeRange().first)  / (timeRange().second - timeRange().first));
        Configuration_t q2 ((*this) (subInterval.second, success));
        if (!success) throw projection_error
                ("Failed to apply constraints in StraightPath::extract");
        q2[pathDofRank_] = ComputeExtraDofValue(pathDofRank_,initial_, end_, (subInterval.second - timeRange().first)  / (timeRange().second - timeRange().first));
        PathPtr_t result = LimbRRTPath::create (device_, q1, q2, l,
                           constraints (), pathDofRank_);
        return result;
    }

    DevicePtr_t LimbRRTPath::device () const
    {
      return device_;
    }
  } //   namespace interpolation
  } //   namespace rbprm
} // namespace hpp

