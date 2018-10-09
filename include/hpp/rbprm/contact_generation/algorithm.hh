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

# include <hpp/rbprm/reports.hh>
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
ContactReport HPP_RBPRM_DLLAPI oneStep(ContactGenHelper& helper);


/// Generates a balanced contact configuration, considering the
/// given current configuration of the robot, and a direction of motion.
/// Typically used to generate a start and / or goal configuration automatically for a planning problem.
///
/// \param body The considered FullBody robot for which to generate contacts
/// \param configuration Current full body configuration.
/// \param affordances the set of 3D objects to consider for contact creation.
/// \param affFilters a vector of strings determining which affordance
///  types are to be used in generating contacts for each limb.
/// \param direction An estimation of the direction of motion of the character.
/// \param robustnessTreshold minimum value of the static equilibrium robustness criterion required to accept the configuration (0 by default).
/// \return a State describing the computed contact configuration, with relevant contact information and balance information.
hpp::rbprm::State HPP_RBPRM_DLLAPI ComputeContacts(
  const hpp::rbprm::RbPrmFullBodyPtr_t& body, pinocchio::ConfigurationIn_t configuration,
  const affMap_t& affordances,
  const std::map<std::string, std::vector<std::string> >& affFilters, const fcl::Vec3f& direction,
  const double robustnessTreshold = 0, const fcl::Vec3f& acceleration = fcl::Vec3f(0,0,0));

/// Generates a balanced contact configuration, considering the
/// given current configuration of the robot, and a previous, balanced configuration.
/// Existing contacts are maintained provided joint limits and balance remains respected.
/// Otherwise a contact generation strategy is employed.
///
/// \param previous The previously computed State of the robot
/// \param body The considered FullBody robot for which to generate contacts
/// \param configuration Current full body configuration.
/// \param affordances the set of 3D objects to consider for contact creation.
/// \param affFilters a vector of strings determining which affordance
///  types are to be used in generating contacts for each limb.
/// \param direction An estimation of the direction of motion of the character.
/// \param robustnessTreshold minimum value of the static equilibrium robustness criterion required to accept the configuration (0 by default).
/// \param acceleration acceleration on the CoM estimated by the planning
/// \param comPath : path found by the planning
/// \param currentPathId : timing inside comPath such that comPath(currentPathId) == configuration (for the freeflyer DOFs)
/// \return a State describing the computed contact configuration, with relevant contact information and balance information.
hpp::rbprm::contact::ContactReport HPP_RBPRM_DLLAPI ComputeContacts(
        const hpp::rbprm::State& previous, const hpp::rbprm::RbPrmFullBodyPtr_t& body,
        pinocchio::ConfigurationIn_t configuration,
            const affMap_t& affordances,
        const std::map<std::string, std::vector<std::string> >& affFilters, const fcl::Vec3f& direction,
  const double robustnessTreshold = 0,const fcl::Vec3f& acceleration = fcl::Vec3f(0,0,0),
        const core::PathConstPtr_t& comPath = core::PathPtr_t(),const double currentPathId=0,const bool testReachability=true, const bool quasiStatic=false);


    } // namespace contact
  } // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_ALGORITHM_HH
