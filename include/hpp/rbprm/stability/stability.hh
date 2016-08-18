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
# define HPP_RBPRM_STABILITY_HH

#include <hpp/model/device.hh>
#include <hpp/rbprm/rbprm-state.hh>
#include <hpp/rbprm/rbprm-fullbody.hh>

#include <map>
#include <memory>

namespace hpp {

  namespace rbprm {
  namespace stability{


    typedef Eigen::Matrix <model::value_type, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXX;
    typedef Eigen::Matrix <model::value_type, Eigen::Dynamic, 1>                               VectorX;

    /// Using the polytope computation of the gravito inertial wrench cone, performs
    /// a static equilibrium test on the robot.
    ///
    /// \param fullbody The considered robot for static equilibrium
    /// \param state The current State of the robots, in terms of contact creation
    /// \return Whether the configuration is statically balanced
    double IsStable(const RbPrmFullBodyPtr_t fullbody, State& state);


    /// Using the polytope computation of the gravito inertial wrench cone,
    /// returns the CWC of the robot at a given state
    ///
    /// \param fullbody The considered robot for static equilibrium
    /// \param state The current State of the robots, in terms of contact creation
    std::pair<MatrixXX, VectorX> ComputeCentroidalCone(const RbPrmFullBodyPtr_t fullbody, State& state, const core::value_type friction = 0.5);
  } // namespace stability
} // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_STABILITY_HH
