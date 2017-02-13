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

#ifndef HPP_RBPRM_CONTACT_GENERATION_HH
# define HPP_RBPRM_CONTACT_GENERATION_HH

# include <hpp/rbprm/rbprm-state.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>
# include <hpp/rbprm/projection/projection.hh>

# include <queue>

namespace hpp {
namespace rbprm {
namespace contact{

typedef std::queue<hpp::rbprm::State> Q_State;

struct ContactGenHelper
{
    hpp::rbprm::RbPrmFullBodyPtr_t fullBody_;
    hpp::rbprm::State previousState_;
    Q_State& candidates_;
    model::Configuration_t targetRootConfiguration_;
    bool checkStability_;
    fcl::Vec3f acceleration_;
};


/// Generates all potentially valid cases of valid contact maintenance
/// given a previous configuration.
/// \param fullBody target Robot
/// \param target, desired root position
/// \param currentState current state of the robot (configuration and contacts)
/// \param maxBrokenContacts max number of contacts that can be broken in the process
/// \return a queue of contact states candidates for maintenance, ordered by number of contacts broken
/// and priority in the list wrt the contact order
Q_State maintain_contacts_combinatorial(const hpp::rbprm::State& currentState, const unsigned int maxBrokenContacts=1);

/// Generates all potentially valid cases of valid contact creation
/// given a current State.
/// \param fullBody target Robot
/// \param target, desired root position
/// \param currentState current state of the robot (configuration and contacts)
/// \param maxBrokenContacts max number of contacts that can be broken in the process
/// \return a queue of contact states candidates for maintenance, ordered by number of contacts broken
/// and priority in the list wrt the contact order
//Q_State gen_contacts_combinatorial(const hpp::rbprm::State& currentState, const unsigned int maxCreatedContacts=1);

/// Given a combinatorial of possible contacts, generate
/// the first "valid" configuration, that the first kinematic
/// configuration that removes the minimum number of contacts and
/// is collision free.
/// \param fullBody target Robot
/// \param previousState, desired root configuration
/// \param targetRootConfiguration, desired root configuration
/// \param candidates queue of candidates. The queue elements are decreased as trials go on.
/// \param checkStability if true, among the solution candidates of similar rank, will return
/// the first one that provides stability.
/// \param acceleration current COM acceleration
/// \return the best candidate wrt the priority in the list and the contact order
projection::ProjectionReport maintain_contacts(ContactGenHelper& contactGenHelper);

    } // namespace projection
  } // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_CONTACT_GENERATION_HH
