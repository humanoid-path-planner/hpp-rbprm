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

# include <queue>

namespace hpp {
namespace rbprm {
namespace contact{

struct ContactGenHelper
{
    hpp::rbprm::State prevState_;
    hpp::rbprm::State result_;
    std::vector<std::string> contactAltered_;
};

typedef std::queue<hpp::rbprm::State> Q_State;

/// Generates all potentially valid cases of valid contact maintenance
/// given a previous configuration.
/// \param fullBody target Robot
/// \param target, desired root position
/// \param currentState current state of the robot (configuration and contacts)
/// \param maxBrokenContacts max number of contacts that can be broken in the process
/// \return a queue of contact states candidates for maintenance, ordered by number of contacts broken
/// and priority in the list wrt the contact order
Q_State maintain_contacts_combinatorial(const hpp::rbprm::State& currentState, const unsigned int maxBrokenContacts=1);

    } // namespace projection
  } // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_CONTACT_GENERATION_HH
