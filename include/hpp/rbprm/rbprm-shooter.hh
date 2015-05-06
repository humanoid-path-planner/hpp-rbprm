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

#ifndef HPP_RBPRM_SHOOTER_HH
# define HPP_RBPRM_SHOOTER_HH

# include <hpp/rbprm/config.hh>
# include <hpp/model/joint.hh>
# include <hpp/model/joint-configuration.hh>
# include <hpp/core/configuration-shooter.hh>

namespace hpp {
/// \addtogroup configuration_sampling
/// \{

/// Samples configuration which respect the reachability condition
    class HPP_RBPRM_DLLAPI RbPrmShooter : public core::ConfigurationShooter{
    ///
    /// Note that translation joints have to be bounded.
    RbPrmShooter (const core::DevicePtr_t& robot);
    virtual core::ConfigurationPtr_t shoot () const;

  private:
    const core::DevicePtr_t& robot_;
    }; // class RbprmShooter
/// \}
} // namespace hpp
#endif // HPP_RBPRM_SHOOTER_HH

