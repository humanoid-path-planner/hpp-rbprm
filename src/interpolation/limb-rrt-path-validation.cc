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

#include <hpp/model/device.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/joint-bound-validation.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/path.hh>
#include <hpp/rbprm/interpolation/limb-rrt-path-validation.hh>

namespace hpp {
  using namespace core;
  using namespace model;
  namespace rbprm {
  namespace interpolation{

    LimbRRTPathValidationPtr_t
    LimbRRTPathValidation::create (const DevicePtr_t& robot,
                                   const value_type& stepSize,
                                   const std::size_t pathDofRank)
    {
        LimbRRTPathValidation* ptr =
        new LimbRRTPathValidation(robot, stepSize, pathDofRank);
        return LimbRRTPathValidationPtr_t (ptr);
    }

    bool LimbRRTPathValidation::validate (const PathPtr_t& path,
                                          bool reverse, PathPtr_t& validPart,
                                          PathValidationReportPtr_t& validationReport)
    {
        if(path->initial()[pathDofRank_] > path->end()[pathDofRank_])
        {
            validPart = path->extract(interval_t(path->timeRange().first, path->timeRange().first));
            return false;
        }
        return DiscretizedPathValidation::validate(path,reverse,validPart,validationReport);
    }

    LimbRRTPathValidation::LimbRRTPathValidation(const DevicePtr_t& robot,
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
