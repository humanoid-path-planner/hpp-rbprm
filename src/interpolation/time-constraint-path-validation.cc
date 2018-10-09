//
// Copyright (c) 2015 CNRS
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

#include <hpp/pinocchio/device.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/joint-bound-validation.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/path.hh>
#include <hpp/rbprm/interpolation/time-constraint-path-validation.hh>

namespace hpp {
  using namespace core;
  namespace rbprm {
  namespace interpolation{

    TimeConstraintPathValidationPtr_t
    TimeConstraintPathValidation::create (const DevicePtr_t& robot,
                                   const value_type& stepSize,
                                   const std::size_t pathDofRank)
    {
        TimeConstraintPathValidation* ptr =
        new TimeConstraintPathValidation(robot, stepSize, pathDofRank);
        return TimeConstraintPathValidationPtr_t (ptr);
    }

    bool TimeConstraintPathValidation::validate (const PathPtr_t& path,
                                          bool reverse, PathPtr_t& validPart,
                                          PathValidationReportPtr_t& validationReport)
    {
        if(path->initial()[pathDofRank_] > path->end()[pathDofRank_])
        {
            validPart = path->extract(interval_t(path->timeRange().first, path->timeRange().first));
            return false;
        }
        // to limit discontinuities, try to check that variation is not too important
        Configuration_t init = path->initial();
        Configuration_t end = path->end();
        std::size_t dim =  init.rows() - 7;
        double totalDistance = (end.tail(dim) - init.tail(dim)).norm();
        double length = path->length();
        //checking 10 points
        Configuration_t last_q = init;
        Configuration_t q = end;
        for(double i = 1; i < 10; ++i)
        {
            q = path->operator ()(i/10. * length);
            double distance = (last_q.tail(dim) - q.tail(dim)).norm();
            last_q =q;
            if(distance / totalDistance > 0.2)
            {
                validPart = path->extract(interval_t(path->timeRange().first, path->timeRange().first));
                return false;
            }
        }
        return DiscretizedPathValidation::validate(path,reverse,validPart,validationReport);
    }

    TimeConstraintPathValidation::TimeConstraintPathValidation(const DevicePtr_t& robot,
                                                 const value_type& stepSize,
                                                 const std::size_t pathDofRank)
        : DiscretizedPathValidation(robot,stepSize)
        , pathDofRank_(pathDofRank)
    {
        add (CollisionValidation::create (robot));
        add (JointBoundValidation::create (robot));
    }
  } // namespace interpolation
  } // namespace rbprm
} // namespace hpp
