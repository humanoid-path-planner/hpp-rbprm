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

#ifndef _CLASS_IKSOLVER
#define _CLASS_IKSOLVER

#include "hpp/rbprm/rbprm-limb.hh"

namespace hpp
{
namespace rbprm
{
namespace ik
{
    typedef Eigen::Ref<Eigen::Vector3d> Vector3dRef;

    bool apply  (RbPrmLimb& limb, const fcl::Vec3f &target, const fcl::Matrix3f &targetRotation, model::ConfigurationOut_t configuration);
} // namespace ik
} // namespace rbprm
} // namespace hpp
#endif //_CLASS_IKSOLVER
