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

#ifndef HPP_RBPRM_LIMB_RRT_SHOOTER_HH
# define HPP_RBPRM_LIMB_RRT_SHOOTER_HH

# include <hpp/rbprm/config.hh>
# include <hpp/rbprm/interpolation/time-constraint-shooter.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>
# include <hpp/core/path.hh>
# include <hpp/model/device.hh>

# include <vector>


namespace hpp {
    namespace rbprm {
    namespace interpolation {
    struct LimbRRTShooterFactory
    {
        TimeConstraintShooterPtr_t operator()(const RbPrmFullBodyPtr_t fullBody, const hpp::core::PathPtr_t path,
                        const std::size_t pathDofRank, const hpp::rbprm::State &from, const hpp::rbprm::State &to,
                                              const T_TimeDependant& tds, core::ConfigProjectorPtr_t projector) const;
    };

/// \}
    } // namespace interpolation
    } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_LIMB_RRT_SHOOTER_HH
