/// Copyright (c) 2017 CNRS
/// Authors: stonneau
///
///
// This file is part of hpp-rbprm.
// hpp-rbprm is free software: you can redistribute it
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

#ifndef HPP_RBPRM_ALGORITHM_HH
# define HPP_RBPRM_ALGORITHM_HH

# include <hpp/rbprm/rbprm-state.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>
# include <hpp/rbprm/projection/projection.hh>
# include <hpp/rbprm/contact_generation/contact_generation.hh>

# include <queue>

namespace hpp {
namespace rbprm {
namespace contact{

/// Generates one step of the contact planner.
/// First, generates all possible cases of contact maintenance (feasible).
/// Starting with the preferred case, generates all admissible contact combinatorial.
/// Again starting with the preferred one, tries to generate a feasible contact.
/// iterates like this until either all solution failed or a feasible contact is found.
/// \param ContactGenHelper parametrization of the planner
/// \return whether a step was successfully generated
projection::ProjectionReport HPP_RBPRM_DLLAPI oneStep(ContactGenHelper& helper);


    } // namespace projection
  } // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_ALGORITHM_HH
