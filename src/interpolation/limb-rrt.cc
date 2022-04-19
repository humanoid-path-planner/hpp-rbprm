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

#include <hpp/core/bi-rrt-planner.hh>
#include <hpp/rbprm/interpolation/interpolation-constraints.hh>
#include <hpp/rbprm/interpolation/limb-rrt.hh>
#include <hpp/rbprm/interpolation/time-constraint-utils.hh>

namespace hpp {
using namespace core;
namespace rbprm {
namespace interpolation {
void SetLimbRRTConstraints::operator()(LimbRRTHelper &helper, const State &from,
                                       const State &to) const {
  CreateContactConstraints<LimbRRTHelper>(helper, from, to);
}

core::PathPtr_t limbRRT(RbPrmFullBodyPtr_t fullbody,
                        core::ProblemPtr_t referenceProblem,
                        const rbprm::CIT_State &startState,
                        const rbprm::CIT_State &endState,
                        const std::size_t numOptimizations,
                        const std::size_t maxIteration) {
  LimbRRTShooterFactory shooterFactory;
  SetLimbRRTConstraints constraintFactory;
  return interpolateStates<LimbRRTHelper, LimbRRTShooterFactory,
                           SetLimbRRTConstraints, CIT_State>(
      fullbody, referenceProblem, shooterFactory, constraintFactory, startState,
      endState, numOptimizations, false, 0.001, maxIteration);
}

core::PathPtr_t limbRRTFromPath(RbPrmFullBodyPtr_t fullbody,
                                core::ProblemPtr_t referenceProblem,
                                const PathPtr_t refPath,
                                const CIT_StateFrame &startState,
                                const CIT_StateFrame &endState,
                                const std::size_t numOptimizations) {
  LimbRRTShooterFactory shooterFactory;
  SetLimbRRTConstraints constraintFactory;
  return interpolateStatesFromPath<LimbRRTHelper, LimbRRTShooterFactory,
                                   SetLimbRRTConstraints>(
      fullbody, referenceProblem, shooterFactory, constraintFactory, refPath,
      startState, endState, numOptimizations);
}

}  // namespace interpolation
}  // namespace rbprm
}  // namespace hpp
