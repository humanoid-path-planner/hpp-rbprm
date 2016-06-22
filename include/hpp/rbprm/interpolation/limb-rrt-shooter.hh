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
# include <hpp/core/configuration-shooter.hh>
# include <hpp/core/basic-configuration-shooter.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>
# include <hpp/core/path.hh>
# include <hpp/model/device.hh>

# include <vector>


namespace hpp {
    namespace rbprm {
    namespace interpolation {
    HPP_PREDEF_CLASS (LimbRRTShooter);
    typedef boost::shared_ptr <LimbRRTShooter>
    LimbRRTShooterPtr_t;

/// \addtogroup configuration_sampling
/// \{
/// Configuration shooter for the limb RRT.
/// will generate a configuration for a given limb, and sample
/// a root position extracted from a normalized path
    class HPP_RBPRM_DLLAPI LimbRRTShooter : public core::ConfigurationShooter{
    ///
    public:
        /// Creates an instance of LimbRRTShooter for interpolating a limb motion along a root path.
        ///
        /// \param limb A device for which the root joint path is constrained
        /// the root joint is necessarily a free flyer
        /// \param path A constrained path for the root of the limb
        /// \return a pointer to an instance of LimbRRTShooter
        static HPP_RBPRM_DLLAPI LimbRRTShooterPtr_t create (const hpp::rbprm::RbPrmLimbPtr_t limb,
                                                            const hpp::core::PathPtr_t path,
                                                            const std::size_t pathDofRank);

        virtual core::ConfigurationPtr_t shoot () const;

    public:
        const hpp::rbprm::RbPrmLimbPtr_t limb_;
        const hpp::core::PathPtr_t path_;
        const std::size_t pathDofRank_;

    private:
        LimbRRTShooterWkPtr_t weak_;


    protected:
        LimbRRTShooter (const hpp::rbprm::RbPrmLimbPtr_t limb,
                        const hpp::core::PathPtr_t path,
                        const std::size_t pathDofRank);

        void init (const LimbRRTShooterPtr_t& self);
    }; // class LimbRRTShooter
/// \}
    } // namespace interpolation
    } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_LIMB_RRT_SHOOTER_HH
