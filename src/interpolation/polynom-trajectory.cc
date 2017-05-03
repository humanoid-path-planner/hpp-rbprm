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

#include <hpp/rbprm/interpolation/polynom-trajectory.hh>
#include <hpp/rbprm/interpolation/time-constraint-utils.hh>
#include <hpp/model/device.hh>
#include <hpp/model/configuration.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/locked-joint.hh>

using namespace hpp::core;

namespace hpp {
  namespace rbprm {
  namespace interpolation{
    PolynomTrajectory::PolynomTrajectory (PolynomPtr_t polynom,
                                          core::value_type subSetStart,
                                          core::value_type subSetEnd) :
        parent_t (interval_t (0, subSetEnd - subSetStart), 3,3),
        polynom_ (polynom),
        subSetStart_(subSetStart),
        subSetEnd_(subSetEnd),
        length_(subSetEnd - subSetStart)
      {
        assert (length_ >= 0);
        assert (!constraints ());
      }


    model::value_type normalize(const PolynomTrajectory& path, model::value_type param)
    {
        value_type u;
        if (path.timeRange ().second == 0)
            u = 0;
        else
            u = (param - path.timeRange ().first) / (path.timeRange ().second - path.timeRange().first);
        return u;
    }

    PolynomTrajectory::PolynomTrajectory (const PolynomTrajectory& path) :
        parent_t (interval_t (0, path.length_), 3,3),
        polynom_ (path.polynom_),
        subSetStart_(path.subSetStart_),
        subSetEnd_(path.subSetEnd_),
        length_(path.length_){}

    bool PolynomTrajectory::impl_compute (ConfigurationOut_t result,
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
            result = polynom_->operator ()(u);
        }
        return true;
    }

    PathPtr_t PolynomTrajectory::extract (const interval_t& subInterval) const
      throw (projection_error)
    {
        return PolynomTrajectory::create (polynom_,subInterval.first, subInterval.second);
    }
  } //   namespace interpolation
  } //   namespace rbprm
} // namespace hpp

