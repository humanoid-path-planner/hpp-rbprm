//
// Copyright (c) 2017 CNRS
// Authors: Fernbach Pierre
//
// This file is part of hpp-rbprm
// hpp-core is free software: you can redistribute it
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

#ifndef HPP_RBPRM_DYNAMIC_VALIDATION_HH
# define HPP_RBPRM_DYNAMIC_VALIDATION_HH

# include <hpp/core/config-validation.hh>
#include <centroidal-dynamics-lib/centroidal_dynamics.hh>
# include <hpp/rbprm/rbprm-validation-report.hh>
namespace hpp {
  namespace rbprm {


    /// Exception thrown when a configuration is not within the bounds
    class DynamicValidationReport : public core::ValidationReport
    {
    public:
      DynamicValidationReport (centroidal_dynamics::Vector3 acc) :
  ValidationReport (),acc_(acc)
  {
  }
      /// Print report in a stream
      virtual std::ostream& print (std::ostream& os) const
      {
  os << "Acceleration "<<acc_.transpose()<<" invalid with current contacts.";
  return os;
      }

      centroidal_dynamics::Vector3 acc_;
    };



    HPP_PREDEF_CLASS(DynamicValidation);
    typedef boost::shared_ptr <DynamicValidation> DynamicValidationPtr_t;

    class DynamicValidation : public core::ConfigValidation
    {
    public:
      static DynamicValidationPtr_t create (bool rectangularContact, double sizeFootX, double sizeFootY, double mass, double mu);


      /// Compute whether the configuration is valid
      ///
      /// \param config the config to check for validity,
      /// \retval validationReport report on validation. Must be a valid rbprmReport with the latest collision informations.
      ///         If non valid, a new validation report will be allocated
      ///         and returned via this shared pointer.
      /// \return whether the whole config is valid.
      virtual bool validate (const core::Configuration_t& config, core::ValidationReportPtr_t& validationReport);

      void setInitialReport(core::ValidationReportPtr_t initialReport);



    protected:
      DynamicValidation (bool rectangularContact, double sizeFootX, double sizeFootY, double mass, double mu);
    private:
      bool rectangularContact_;
      double sizeFootX_;
      double sizeFootY_;
      double mass_;
      double mu_;
      centroidal_dynamics::Equilibrium* sEq_;
      core::RbprmValidationReportPtr_t lastReport_;
      bool initContacts_;
      core::Configuration_t lastAcc_;
      centroidal_dynamics::Matrix63 H_;
      centroidal_dynamics::Vector6 h_;

    }; // class dynamicValidation
    /// \}
  } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_DYNAMIC_VALIDATION_HH
