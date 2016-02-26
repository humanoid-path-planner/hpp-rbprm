//
// Copyright (c) 2014 CNRS
// Authors: Steve Tonneau
//
// This file is part of hpp-rbprm
// hpp-rbprm is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_RBPRM_ROM_VALIDATION_HH
# define HPP_RBPRM_ROM_VALIDATION_HH

# include <hpp/core/collision-validation.hh>
# include <hpp/rbprm/rbprm-device.hh>
# include <hpp/rbprm/config.hh>

namespace hpp {
  namespace rbprm {

    class RbPrmRomValidation;
    typedef boost::shared_ptr <RbPrmRomValidation> RbPrmRomValidationPtr_t;

    /// \addtogroup validation
    /// \{

    /// Validate a Rom configuration with respect to the reachability condition;
    /// a Rom Configuration is valid the object linked to it is colliding
    /// with the environment. A normal filter can be optionnaly specified
    /// to only accept collision with obstacles aligned with special normal surfaces.
    ///
    class HPP_RBPRM_DLLAPI RbPrmRomValidation : public core::CollisionValidation
    {
    public:
      static RbPrmRomValidationPtr_t create (const model::DevicePtr_t& robot,
                                             const std::vector<std::string>& affFilters 
																						 	= std::vector<std::string>());

      /// Compute whether the configuration is valid
      ///
      /// \param config the config to check for validity
      /// \return whether the whole config is valid.
      virtual bool validate (const core::Configuration_t& config);

      /// Compute whether the configuration is valid
      ///
      /// \param config the config to check for validity,
      /// \retval validationReport report on validation (used only for rom shape). This parameter will
      ///         dynamically cast into CollisionValidationReport type,
      /// \return whether the whole config is valid.
      virtual bool validate (const core::Configuration_t& config, core::ValidationReportPtr_t& validationReport);

    public:
      const std::vector<std::string> filter_;

    protected:
      RbPrmRomValidation (const model::DevicePtr_t &robot,
                       const std::vector<std::string>& affFilters);
    private:
      core::ValidationReportPtr_t unusedReport_;
    }; // class RbPrmValidation
    /// \}
  } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_ROM_VALIDATION_HH
