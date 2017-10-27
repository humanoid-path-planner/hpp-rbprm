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

#include <hpp/rbprm/interpolation/time-constraint-path.hh>
#include <hpp/rbprm/interpolation/time-constraint-utils.hh>
#include <hpp/model/device.hh>
#include <hpp/model/configuration.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/locked-joint.hh>

using namespace hpp::core;

namespace hpp {
  namespace rbprm {
  namespace interpolation{
    TimeConstraintPath::TimeConstraintPath (const DevicePtr_t& device,
                              ConfigurationIn_t init,
                              ConfigurationIn_t end,
                              value_type length,
                              const std::size_t pathDofRank,
                              const T_TimeDependant& tds) :
        parent_t (interval_t (0, length), device->configSize (),
                  device->numberDof ()),
        device_ (device), initial_ (init), end_ (end),
        pathDofRank_(pathDofRank),
        tds_(tds)
      {
        assert (device);
        assert (length >= 0);
        assert (!constraints ());
      }

    TimeConstraintPath::TimeConstraintPath (const DevicePtr_t& device,
				ConfigurationIn_t init,
                ConfigurationIn_t end,
                value_type length,
                ConstraintSetPtr_t constraints,
                const std::size_t pathDofRank,
                const T_TimeDependant& tds) :
        parent_t (interval_t (0, length), device->configSize (),
                  device->numberDof (), constraints),
        device_ (device), initial_ (init), end_ (end),
        pathDofRank_(pathDofRank), tds_(tds)
    {
        assert (device);
        assert (length >= 0);
    }

    TimeConstraintPath::TimeConstraintPath (const TimeConstraintPath& path) :
        parent_t (path), device_ (path.device_), initial_ (path.initial_),
        end_ (path.end_)    , pathDofRank_(path.pathDofRank_), tds_(path.tds_) {}

    TimeConstraintPath::TimeConstraintPath (const TimeConstraintPath& path,
                const ConstraintSetPtr_t& constraints) :
        parent_t (path, constraints), device_ (path.device_),
        initial_ (path.initial_), end_ (path.end_), pathDofRank_(path.pathDofRank_), tds_(path.tds_) {}

    model::value_type ComputeExtraDofValue(const std::size_t dofRank,
                              const Configuration_t init,
                              const Configuration_t end,
                              const model::value_type normalizedValue)
    {
        double a = init[dofRank];
        double b = end[dofRank];
        return (b-a)* normalizedValue + a;
    }

    void TimeConstraintPath::updateConstraints(core::ConfigurationOut_t configuration) const
    {
        if (constraints() && constraints()->configProjector ())
        {
            UpdateConstraints(configuration, constraints()->configProjector (), tds_, pathDofRank_);
        }
    }

    bool TimeConstraintPath::impl_compute (ConfigurationOut_t result,
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
            value_type u;
            if (timeRange ().second == 0)
                u = 0;
            else
                u = (param - timeRange ().first) / (timeRange ().second - timeRange().first);
            model::interpolate (device_, initial_, end_, u, result);
            model::value_type dof = ComputeExtraDofValue(pathDofRank_,initial_, end_, u);
            result[pathDofRank_] = dof;
        }
        updateConstraints(result);
        return true;
    }

    PathPtr_t TimeConstraintPath::extract (const interval_t& subInterval) const
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
        PathPtr_t result = TimeConstraintPath::create (device_, q1, q2, l,
                           constraints (), pathDofRank_, tds_);
        return result;
    }

    DevicePtr_t TimeConstraintPath::device () const
    {
      return device_;
    }

    void TimeConstraintPath::checkPath () const
    {
      Configuration_t initc = initial();
      Configuration_t endc = end();
      vector_t errr;
      updateConstraints(initc);
      if (constraints()) {
        if (!constraints()->isSatisfied (initial(),errr)) {
            device_->currentConfiguration(initc);
            device_->computeForwardKinematics();
std::cout << "init conf " <<  device_->positionCenterOfMass() << "\n error \n" << errr << std::endl;
/*device_->currentConfiguration(initc);
device_->computeForwardKinematics();
std::cout << "rf_foot_joint  " << std::endl;
std::cout <<  device_->getJointByName("rf_foot_joint")->currentTransformation().getTranslation() << std::endl;

std::cout << "lf_foot_joint "  << std::endl;
std::cout <<  device_->getJointByName("lf_foot_joint")->currentTransformation().getTranslation() << std::endl;

std::cout << "rh_foot_joint  " << std::endl;
std::cout <<  device_->getJointByName("rh_foot_joint")->currentTransformation().getTranslation() << std::endl;

std::cout << "lh_foot_joint  " << std::endl;
std::cout <<  device_->getJointByName("lh_foot_joint")->currentTransformation().getTranslation() << std::endl;*/
          hppDout (error,"Initial configuration of path does not satisfy the constraints" << initial().transpose ());
          throw projection_error ("Initial configuration of path does not satisfy "
              "the constraints");
        }
        updateConstraints(endc);
        if (constraints() && !constraints()->isSatisfied (end())) {
//std::cout << "end conf " <<  initc << std::endl;
          hppDout (error,"End configuration of path does not satisfy the constraints"<< model::displayConfig(end()));
          throw projection_error ("End configuration of path does not satisfy "
              "the constraints");
        }
      }
    }
  } //   namespace interpolation
  } //   namespace rbprm
} // namespace hpp

