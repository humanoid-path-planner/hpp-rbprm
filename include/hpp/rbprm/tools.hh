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

#ifndef HPP_RBPRM_TOOLS_HH
# define HPP_RBPRM_TOOLS_HH

# include <hpp/rbprm/config.hh>
# include <hpp/model/collision-object.hh>
# include <Eigen/Core>

namespace hpp {
  namespace tools {
  /// Uses Rodriguez formula to find transformation between two vectors.
  Eigen::Matrix3d GetRotationMatrix(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
  fcl::Matrix3f GetRotationMatrix(const fcl::Vec3f& from, const fcl::Vec3f& to);
  } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_TOOLS_HH
