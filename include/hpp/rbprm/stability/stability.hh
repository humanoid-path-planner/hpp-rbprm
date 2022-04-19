//
// Copyright (c) 2014 CNRS
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
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_RBPRM_STABILITY_HH
#define HPP_RBPRM_STABILITY_HH

#include <hpp/centroidal-dynamics/centroidal_dynamics.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/rbprm/rbprm-fullbody.hh>
#include <hpp/rbprm/rbprm-state.hh>
#include <map>
#include <memory>

namespace hpp {

namespace rbprm {
namespace stability {

typedef Eigen::Matrix<pinocchio::value_type, Eigen::Dynamic, Eigen::Dynamic,
                      Eigen::RowMajor>
    MatrixXX;
typedef Eigen::Matrix<pinocchio::value_type, Eigen::Dynamic, 1> VectorX;

/// Using the polytope computation of the gravito inertial wrench cone, performs
/// a static equilibrium test on the robot.
///
/// \param fullbody The considered robot for static equilibrium
/// \param state The current State of the robots, in terms of contact creation
/// \param acc acceleration of the COM of the robot
/// \return Whether the configuration is statically balanced

double IsStable(const RbPrmFullBodyPtr_t fullbody, State& state,
                fcl::Vec3f acc = fcl::Vec3f(0, 0, 0),
                fcl::Vec3f com = fcl::Vec3f(0, 0, 0),
                const centroidal_dynamics::EquilibriumAlgorithm =
                    centroidal_dynamics::EQUILIBRIUM_ALGORITHM_DLP);

/// Using the polytope computation of the gravito inertial wrench cone,
/// returns the CWC of the robot at a given state
///
/// \param fullbody The considered robot for static equilibrium
/// \param state The current State of the robots, in terms of contact creation
std::pair<MatrixXX, VectorX> ComputeCentroidalCone(
    const RbPrmFullBodyPtr_t fullbody, State& state,
    const core::value_type friction = 0.5);

centroidal_dynamics::Equilibrium initLibrary(const RbPrmFullBodyPtr_t fullbody);

centroidal_dynamics::Vector3 setupLibrary(
    const RbPrmFullBodyPtr_t fullbody, State& state,
    centroidal_dynamics::Equilibrium& sEq,
    centroidal_dynamics::EquilibriumAlgorithm& alg,
    core::value_type friction = 0.6, const double feetX = 0,
    const double feetY = 0);

}  // namespace stability
}  // namespace rbprm
}  // namespace hpp
#endif  // HPP_RBPRM_STABILITY_HH
