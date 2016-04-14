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

#ifndef HPP_RBPRM_LIMB_RRT_HELPER_HH
# define HPP_RBPRM_LIMB_RRT_HELPER_HH

# include <hpp/rbprm/config.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>
# include <hpp/rbprm/rbprm-state.hh>
# include <hpp/rbprm/rbprm-device.hh>
# include <hpp/core/path.hh>

# include <vector>
# include <map>


namespace hpp {
    namespace rbprm {
    namespace interpolation {

    typedef std::map<std::string, core::DevicePtr_t> T_LimbDevice;

    struct HPP_CORE_DLLAPI LimbRRTHelper
    {
         LimbRRTHelper(model::RbPrmDevicePtr_t rootDevice, RbPrmFullBodyPtr_t fullbody);
        ~LimbRRTHelper();

         model::RbPrmDevicePtr_t rootDevice_;
         T_LimbDevice prototypes_;
    };

    core::PathPtr_t interpolateStates(const LimbRRTHelper& helper, const State& from, const State& to);
    core::PathPtr_t interpolateStates(const LimbRRTHelper& helper, const std::vector<State>& states);

    } // namespace interpolation
    } // namespace rbprm
  } // namespace hpp

#endif // HPP_RBPRM_LIMB_RRT_HELPER_HH
