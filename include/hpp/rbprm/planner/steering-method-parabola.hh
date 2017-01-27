//
// Copyright (c) 2015 CNRS
// Authors: Mylene Campana
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

#ifndef HPP_RBPRM_STEERING_METHOD_PARABOLA_HH
# define HPP_RBPRM_STEERING_METHOD_PARABOLA_HH

# include <hpp/util/debug.hh>
# include <hpp/core/steering-method.hh>
# include <hpp/core/weighed-distance.hh>

namespace hpp {
  namespace rbprm {
    /// \addtogroup steering_method
    /// \{

    using core::value_type;
    using core::vector_t;

    // forward declaration of class
    HPP_PREDEF_CLASS (SteeringMethodParabola);
    // Planner objects are manipulated only via shared pointers
    typedef boost::shared_ptr <SteeringMethodParabola> SteeringMethodParabolaPtr_t;

    /// Steering method that creates StraightPath instances
    ///
    class HPP_CORE_DLLAPI SteeringMethodParabola : public core::SteeringMethod
    {
    public:
      /// Create instance and return shared pointer
      static SteeringMethodParabolaPtr_t create (const core::ProblemPtr_t& problem)
      {
        SteeringMethodParabola* ptr = new SteeringMethodParabola (problem);
        SteeringMethodParabolaPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }
      /// Copy instance and return shared pointer
      static SteeringMethodParabolaPtr_t createCopy
      (const SteeringMethodParabolaPtr_t& other)
      {
        SteeringMethodParabola* ptr = new SteeringMethodParabola (*other);
        SteeringMethodParabolaPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }
      /// Copy instance and return shared pointer
      virtual core::SteeringMethodPtr_t copy () const
      {
        return createCopy (weak_.lock ());
      }

      core::PathPtr_t operator() (core::ConfigurationIn_t q1,
                                  core::ConfigurationIn_t q2) const
      {
        return impl_compute (q1, q2);
      }

      /// create a path between two configurations
      virtual core::PathPtr_t impl_compute (core::ConfigurationIn_t q1,
                                            core::ConfigurationIn_t q2) const;

      /// Compute a random parabola in direction of q1->q2
      core::PathPtr_t compute_random_3D_path (core::ConfigurationIn_t q1,
                                              core::ConfigurationIn_t q2,
                                              value_type* alpha0,
                                              value_type* v0) const;

      /// Compute third constraint : landing in the friction cone
      /// return false if constraint can never be respected.
      /// fill alpha_imp_sup/inf angles limiting initial angle
      /// to respect the constraint.
      bool third_constraint (bool fail, const value_type& X,
                             const value_type& Y,
                             const value_type alpha_imp_min,
                             const value_type alpha_imp_max,
                             value_type *alpha_imp_sup,
                             value_type *alpha_imp_inf,
                             const value_type n2_angle) const;

      /// Compute fiveth constraint: compute intersection between coneS
      /// and plane_theta. If not empty, compute the two lines and the angle
      /// between them = 2*delta.
      /// Equations have been obtained using Matlab.
      bool fiveth_constraint (const core::ConfigurationIn_t q,
                              const value_type theta,
                              const int number,
                              value_type *delta) const;

      // return maximal final (or impact) velocity
      value_type getVImpMax() {return Vimpmax_;}

    protected:
      /// Constructor with problem
      /// Robot and weighed distance are created from problem
      SteeringMethodParabola (const core::ProblemPtr_t& problem);

      /// Copy constructor
      SteeringMethodParabola (const SteeringMethodParabola& other) :
        SteeringMethod (other),
        problem_ (other.problem_), device_ (other.device_),
        distance_ (other.distance_), weak_ (), g_(other.g_),
        V0max_ (other.V0max_), Vimpmax_ (other.Vimpmax_),mu_ (other.mu_),
        Dalpha_ (other.Dalpha_), nLimit_ (other.nLimit_), V0_ (other.V0_),
        Vimp_ (other.Vimp_)
      {
      }

      /// Store weak pointer to itself
      void init (SteeringMethodParabolaWkPtr_t weak)
      {
        SteeringMethod::init (weak);
        weak_ = weak;
      }

    private:
      /// 3D impl_compute
      core::PathPtr_t compute_3D_path (core::ConfigurationIn_t q1,
                                       core::ConfigurationIn_t q2) const;

      /// Compute second constraint: V0 <= V0max
      /// return false if constraint can never be respected.
      /// fill alpha_lim_plus/minus angles limiting initial angle
      /// to respect the constraint.
      bool second_constraint (const value_type& X, const value_type& Y,
                              value_type *alpha_lim_plus,
                              value_type *alpha_lim_minus) const;

      /// Compute sixth constraint: V_imp <= V_imp_max
      /// return false if constraint can never be respected.
      /// fill alpha_imp_plus/minus angles limiting initial angle
      /// to respect the constraint.
      bool sixth_constraint (const value_type& X, const value_type& Y,
                             value_type *alpha_imp_plus,
                             value_type *alpha_imp_minus) const;

      /// Get the length of the path by numerical integration (Simpson method)
      /// Length is computed only when the path is created
      virtual value_type computeLength (const core::ConfigurationIn_t q1,
                                        const core::ConfigurationIn_t q2,
                                        const vector_t coefs) const;

      /// Function equivalent to sqrt( 1 + f'(x)^2 ) in 2D
      /// Function equivalent to sqrt( 1 + y0_dot/x0_dot + fz'(x)^2 ) in 3D
      value_type lengthFunction (const value_type x, const vector_t coefs) const;

      /// Compute parabola coefficients from takeoff angle and other parameters
      vector_t computeCoefficients (const value_type alpha,
                                    const value_type theta,
                                    const value_type X_theta,
                                    const value_type Z,
                                    const value_type x_theta_0,
                                    const value_type z_0) const;

      /// Return true if the maximal height of the parabola does not exceed the
      /// freeflyer translation bounds, false otherwise.
      bool parabMaxHeightRespected (const vector_t coefs,
                                    const value_type x_theta_0,
                                    const value_type x_theta_imp) const;

      /// Process Dichotomy at rank n in interval ]a_inf, a_plus[
      value_type dichotomy (value_type a_inf, value_type a_plus,
                            std::size_t n) const;

      /// Loop on collision ROMs and fill names in ParabolaPath
      void fillROMnames (core::ConfigurationIn_t q,
                         std::vector <std::string> * ROMnames) const;

      core::ProblemPtr_t problem_;
      core::DeviceWkPtr_t device_;
      core::WeighedDistancePtr_t distance_;
      SteeringMethodParabolaWkPtr_t weak_;
      value_type g_; // gravity constant
      mutable value_type V0max_; // maximal initial velocity
      mutable value_type Vimpmax_; // maximal landing velocity
      mutable value_type mu_; // friction coefficient
      value_type Dalpha_; // alpha increment
      mutable std::size_t nLimit_; // number of Dichotomies applied
      mutable bool initialConstraint_; // true if the constraint at the initial point are respected (5° for first cone, 1° and 2° constraints)
      mutable value_type alpha_1_plus_;
      mutable value_type alpha_1_minus_;
      mutable value_type alpha_0_max_;
      mutable value_type alpha_0_min_;
      mutable core::vector_t V0_;
      mutable core::vector_t Vimp_;
      mutable std::vector <std::string> initialROMnames_; // active ROMs
      mutable std::vector <std::string> endROMnames_;

      // Reminder for parabola-results = nb of fails from the following causes:
      // [0] collision or out of configs-bounds
      // [1] one 3D cone is not intersecting the vertical plane
      // [2] takeoff/landing in cone not OK
      // [3] takeoff/landing velocity bound not OK

    }; // SteeringMethodParabola
    /// \}
  } // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_STEERING_METHOD_PARABOLA_HH
