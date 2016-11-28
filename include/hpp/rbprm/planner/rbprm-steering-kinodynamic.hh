// Copyright (c) 2016, LAAS-CNRS
// Authors: Pierre Fernbach (pierre.fernbach@laas.fr)
//
// This file is part of hpp-core
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

#ifndef HPP_RBPRM_STEERING_METHOD_KINODYNAMIC_HH
#define HPP_RBPRM_STEERING_METHOD_KINODYNAMIC_HH

#include <hpp/rbprm/config.hh>
#include <hpp/core/steering-method/steering-kinodynamic.hh>

namespace hpp {
  namespace rbprm {

    using core::Problem;
    using core::ConfigurationIn_t;
    using core::Path;

    HPP_PREDEF_CLASS (SteeringMethodKinodynamic);
    typedef boost::shared_ptr <SteeringMethodKinodynamic> SteeringMethodKinodynamicPtr_t;

    class HPP_RBPRM_DLLAPI SteeringMethodKinodynamic : public core::steeringMethod::Kinodynamic{

    public:

      /// Create an instance
      static SteeringMethodKinodynamicPtr_t create (const core::ProblemPtr_t& problem)
      {
        SteeringMethodKinodynamic* ptr = new SteeringMethodKinodynamic (problem);
        SteeringMethodKinodynamicPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      /// Copy instance and return shared pointer
      static SteeringMethodKinodynamicPtr_t createCopy
      (const SteeringMethodKinodynamicPtr_t& other)
      {
        SteeringMethodKinodynamic* ptr = new SteeringMethodKinodynamic (*other);
        SteeringMethodKinodynamicPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      /// Copy instance and return shared pointer
      virtual core::SteeringMethodPtr_t copy () const
      {
        return createCopy (weak_.lock ());
      }

      /// create a path between two configurations
      virtual core::PathPtr_t impl_compute (core::ConfigurationIn_t q1,
                                      core::ConfigurationIn_t q2) const;


    protected:

      /// Constructor
      SteeringMethodKinodynamic (const core::ProblemPtr_t& problem);

      /// Copy constructor
      SteeringMethodKinodynamic (const SteeringMethodKinodynamic& other);

      /// Store weak pointer to itself
      void init (SteeringMethodKinodynamicWkPtr_t weak)
      {
        core::SteeringMethod::init (weak);
        weak_ = weak;
      }

    private:
      core::DeviceWkPtr_t device_;
      SteeringMethodKinodynamicWkPtr_t weak_;
    }; // class rbprm-kinodynamic
  } // namespace hpp
} // namespace rbprm


#endif // HPP_RBPRM_STEERING_METHOD_KINODYNAMIC_HH
