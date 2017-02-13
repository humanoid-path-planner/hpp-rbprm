/// Copyright (c) 2015 CNRS
/// Authors: Joseph Mirabel
///
///
// This file is part of hpp-wholebody-step.
// hpp-wholebody-step-planner is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-wholebody-step-planner is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-wholebody-step-planner. If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_RBPRM_PROJECTION_HH
# define HPP_RBPRM_PROJECTION_HH

# include <hpp/rbprm/rbprm-state.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>

namespace hpp {
namespace rbprm {
namespace projection{

struct ProjectionReport
{
     ProjectionReport(): success_ (false){}
    ~ProjectionReport(){}
    bool success_;
    hpp::rbprm::State result_;
};

/// Project a configuration to a target position, while maintaining contact constraints.
/// If required, up to maxBrokenContacts can be broken in the process.
/// root position is assumed to be at the 3 first dof.
/// \param fullBody target Robot
/// \param target, desired root position
/// \param currentState current state of the robot (configuration and contacts)
/// \return projection report containing the state projected
ProjectionReport projectToRootPosition(hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const fcl::Vec3f& target,
                                           const hpp::rbprm::State& currentState);

/// Project a configuration to a target root configuration, while maintaining contact constraints.
/// If required, up to maxBrokenContacts can be broken in the process.
/// root position is assumed to be at the 3 first dof.
/// \param fullBody target Robot
/// \param target, desired root position
/// \param currentState current state of the robot (configuration and contacts)
/// \return projection report containing the state projected
ProjectionReport projectToRootConfiguration(hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const model::ConfigurationIn_t conf,
                                           const hpp::rbprm::State& currentState);

/// Project a configuration such that a given limb configuration is collision free
/// \param fullBody target Robot
/// \param limb considered limb
/// \return projection report containing the state projected
ProjectionReport setCollisionFree(hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const core::CollisionValidationPtr_t &validation, const std::string& limb, const hpp::rbprm::State& currentState);
    } // namespace projection
  } // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_PROJECTION_HH
