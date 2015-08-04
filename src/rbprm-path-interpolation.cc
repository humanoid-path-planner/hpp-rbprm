// Copyright (c) 2014, LAAS-CNRS
// Authors: Steve Tonneau (steve.tonneau@laas.fr)
//
// This file is part of hpp-rbprm.
// hpp-rbprm is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-rbprm is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-rbprm. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/rbprm/rbprm-path-interpolation.hh>
#include <hpp/rbprm/stability/stability.hh>

namespace hpp {
  namespace rbprm {

    RbPrmInterpolationPtr_t RbPrmInterpolation::create (const core::PathVectorConstPtr_t path, const hpp::rbprm::RbPrmFullBodyPtr_t robot, const hpp::rbprm::State &start, const hpp::rbprm::State &end)
    {
        RbPrmInterpolation* rbprmDevice = new RbPrmInterpolation(path, robot, start, end);
        RbPrmInterpolationPtr_t res (rbprmDevice);
        res->init (res);
        return res;
    }

    RbPrmInterpolation::~RbPrmInterpolation()
    {
        // NOTHING
    }

    // ========================================================================

    std::vector<State> RbPrmInterpolation::Interpolate(const model::ObjectVector_t &collisionObjects, const double timeStep)
    {
        int nbFailures = 0;
        std::cout << "interpolation " << std::endl;
        std::vector<State> states;
        states.push_back(this->start_);
        const core::interval_t& range = path_->timeRange();
        std::size_t nbRecontacts = 0;
        bool allowFailure = true;
        for(double i = range.first + timeStep; i< range.second; i+= timeStep)
        {
            const State& previous = states.back();
            core::Configuration_t configuration = previous.configuration_;
            const core::Configuration_t configPosition = path_->operator ()(i);
            Eigen::Vector3d dir = configPosition.head<3>() - previous.configuration_.head<3>();
            fcl::Vec3f direction(dir[0], dir[1], dir[2]);
            bool nonZero(false);
            direction.normalize(&nonZero);
            if(!nonZero) direction = fcl::Vec3f(0,0,1.);

{
    direction = fcl::Vec3f(0.1,0,1);
}
            configuration.head<7>() = configPosition.head<7>();
            // TODO Direction 6d
            bool sameAsPrevious(true);
            bool multipleBreaks(false);
            State newState = ComputeContacts(previous, robot_,configuration,collisionObjects,direction,sameAsPrevious,multipleBreaks,allowFailure);
            if(allowFailure && multipleBreaks)
            {
                ++ nbFailures;
                std::cout << "failed at state " << states.size() +1 << std::endl;
            }
            if(multipleBreaks && !allowFailure)
            {
                ++nbRecontacts;
                i -= timeStep;
            }
            else
            {
                nbRecontacts = 0;
            }
            if(sameAsPrevious)
            {
                states.pop_back();
            }
            states.push_back(newState);
            allowFailure = nbRecontacts > robot_->GetLimbs().size();
        }
        states.push_back(this->end_);
        std::cout << "nbfailure " << nbFailures <<std::endl;
        return states;
    }

    std::vector<State> RbPrmInterpolation::Interpolate(const std::vector<core::ConfigurationIn_t>& configurations, const model::ObjectVector_t &collisionObjects)
    {
        bool sameAsPrevious;
        bool multipleBreaks(false);
        std::vector<State> states;
        states.push_back(this->start_);
        bool allowFailure = false;
        for(std::vector<core::ConfigurationIn_t>::const_iterator cit = configurations.begin();
            cit != configurations.end(); ++cit)
        {
            const State& previous = states.back();
            core::Configuration_t configuration = previous.configuration_;
            const core::Configuration_t& configPosition = *cit;
            Eigen::Vector3d dir = configPosition.head<3>() - previous.configuration_.head<3>();
            fcl::Vec3f direction(dir[0], dir[1], dir[2]);
            bool nonZero(false);
            direction.normalize(&nonZero);
            if(!nonZero) direction = fcl::Vec3f(0,0,1.);
            configuration.head<7>() = configPosition.head<7>();
            // TODO Direction 6d
            states.push_back(ComputeContacts(previous, robot_,configuration,collisionObjects,direction,sameAsPrevious,multipleBreaks,allowFailure));
        }
        states.push_back(this->end_);
        return states;
    }

    void RbPrmInterpolation::init(const RbPrmInterpolationWkPtr_t& weakPtr)
    {
        weakPtr_ = weakPtr;
    }

    RbPrmInterpolation::RbPrmInterpolation (const core::PathVectorConstPtr_t path, const hpp::rbprm::RbPrmFullBodyPtr_t robot, const hpp::rbprm::State &start, const hpp::rbprm::State &end)
        : path_(path)
        , start_(start)
        , end_(end)
        , robot_(robot)
    {
        // TODO
    }
  } // model
} //hpp
