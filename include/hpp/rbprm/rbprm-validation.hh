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

#ifndef HPP_RBPRM_VALIDATION_HH
# define HPP_RBPRM_VALIDATION_HH

# include <hpp/core/collision-validation-report.hh>
# include <hpp/core/config-validation.hh>
# include <hpp/core/collision-validation.hh>
# include <hpp/core/problem-solver.hh>
# include <hpp/rbprm/rbprm-rom-validation.hh>
# include <hpp/rbprm/rbprm-device.hh>
# include <hpp/rbprm/rbprm-validation-report.hh>
# include <hpp/rbprm/config.hh>

namespace hpp {
  namespace rbprm {

    class RbPrmValidation;
    typedef boost::shared_ptr <RbPrmValidation> RbPrmValidationPtr_t;
    typedef std::map<std::string, RbPrmRomValidationPtr_t> T_RomValidation;    
    typedef hpp::core::Container <hpp::core::AffordanceObjects_t> affMap_t;

    /// \addtogroup validation
    /// \{

    /// Validate a configuration with respect to the reachability condition;
    /// a Configuration is valid if the trunk robot is collision free while
    /// the Range Of Motion of the is colliding.
    ///
    class HPP_RBPRM_DLLAPI RbPrmValidation : public core::CollisionValidation
    {
    public:
      static RbPrmValidationPtr_t create (const pinocchio::RbPrmDevicePtr_t& robot,
                                          const std::vector<std::string>& filter = std::vector<std::string>(),
                                          const std::map<std::string, std::vector<std::string> >& affFilters =
																						std::map<std::string, std::vector<std::string> >(),
                                                                                    const affMap_t & affordances =affMap_t(),
                                                                                    const core::ObjectStdVector_t& geometries =
                                                                                        core::ObjectStdVector_t());

      /// Compute whether the configuration is valid
      ///
      /// \param config the config to check for validity,
      /// \param throwIfInValid if true throw an exception if config is invalid.
      /// \return whether the whole config is valid.
      virtual bool validate (const core::Configuration_t& config);

      /// Compute whether the configuration is valid
      ///
      /// \param config the config to check for validity,
      /// \retval validationReport report on validation (used only for rom shape). This parameter will
      ///         dynamically cast into CollisionValidationReport type,
      /// \return whether the whole config is valid.
      virtual bool validate (const core::Configuration_t& config,
                 core::ValidationReportPtr_t& validationReport);

      /// Compute whether the configuration is valid
      ///
      /// \param config the config to check for validity,
      /// \retval validationReport report on validation (used only for rom shape). This parameter will
      ///         dynamically cast into CollisionValidationReport type,
      /// \return whether the whole config is valid.
      virtual bool validate (const core::Configuration_t& config,
                 const std::vector<std::string>& filter);

      /// Compute whether the configuration is valid
      ///
      /// \param config the config to check for validity,
      /// \retval validationReport report on validation (used only for rom shape). This parameter will
      ///         dynamically cast into CollisionValidationReport type,
      /// \param filter specify constraints on all roms required to be in contact, will return
      /// false if all specified roms are not colliding
      /// \return whether the whole config is valid.
      virtual bool validate (const core::Configuration_t& config,
                 core::ValidationReportPtr_t& validationReport,
                 const std::vector<std::string>& filter);


      /// Add an obstacle to validation
      /// \param object obstacle added
      /// Store obstacle and build a collision pair with each body of the robot.
      /// \notice this function has to be called for trunk validation and rom 
			/// validation separately unless they use same obstacles (not usually the case)
            virtual void addObstacle (const core::CollisionObjectConstPtr_t& object);

      /// Remove a collision pair between a joint and an obstacle
      /// \param the joint that holds the inner objects,
      /// \param the obstacle to remove.
      /// \notice This is applied for both trunk and rom shapes. This can be done for a single
      /// shape through trunkValidation_ and romValidation_ attributes.
      virtual void removeObstacleFromJoint
    (const core::JointPtr_t& joint, const core::CollisionObjectPtr_t& obstacle);


      /// Rearrange the collisions pairs of all configValidation in a random manner
      /// \brief randomnizeCollisionPairs
      ///
      virtual void randomnizeCollisionPairs();

      /// \brief set if the collision validation should compute all the possible
      /// contacts or stop after the first pairs in collision
      /// This method set the parameter for all the romValidations_ objects (but not the trunk)
      ///
      void computeAllContacts(bool computeAllContacts);

    private:
      /// Compute whether the configuration is valid for the root (collision and joint-bound)
      ///
      /// \param config the config to check for validity,
      /// \retval validationReport report on validation (used only for rom shape). This parameter will
      ///         dynamically cast into CollisionValidationReport type,
      /// \return whether the whole config is valid.
      virtual bool validateTrunk(const core::Configuration_t& config,
                                             hpp::core::ValidationReportPtr_t &validationReport);

      /// Compute whether the roms configurations are valid
      /// \param config the config to check for validity,
      /// \return whether the whole config is valid.
      bool validateRoms(const core::Configuration_t& config);

      /// Compute whether the roms configurations are valid
      /// \param config the config to check for validity,
      /// \param validationReport the report (can be cast to rbprmValidationReport) with info on the trunk and ROM states,
      /// \return whether the whole config is valid.
      bool validateRoms(const core::Configuration_t& config,
                         core::RbprmValidationReportPtr_t &validationReport);

      /// Compute whether the roms configurations are valid
      /// \param config the config to check for validity,
      /// \param filter specify constraints on all roms required to be in contact, will return
      /// false if all specified roms are not colliding
      /// \return whether the whole config is valid.
      bool validateRoms(const core::Configuration_t& config,
                        const std::vector<std::string>& filter);

      /// \param config the config to check for validity,
      /// \param filter specify constraints on all roms required to be in contact, will return
      /// \param validationReport the report (can be cast to rbprmValidationReport) with info on the trunk and ROM states,
      /// \return whether the whole config is valid.
      bool validateRoms(const core::Configuration_t& config,
                        const std::vector<std::string>& filter,
                         core::RbprmValidationReportPtr_t &validationReport);

    public:
      /// CollisionValidation for the trunk
      const core::CollisionValidationPtr_t trunkValidation_;
      const core::JointBoundValidationPtr_t boundValidation_;
      /// CollisionValidation for the range of motion of the limbs
      const T_RomValidation romValidations_;
      std::vector<std::string> defaultFilter_;

    protected:
      RbPrmValidation (const pinocchio::RbPrmDevicePtr_t& robot,
                       const std::vector<std::string>& filter,
                       const std::map<std::string,
											 	std::vector<std::string> >& affFilters,
                                             const affMap_t& affordances,
                                             const core::ObjectStdVector_t& geometries);

											 
    private:
      core::ValidationReportPtr_t unusedReport_;

    }; // class RbPrmValidation
    /// \}
  } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_VALIDATION_HH
