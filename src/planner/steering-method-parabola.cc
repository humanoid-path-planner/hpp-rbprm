//
// Copyright (c) 2015-2016 CNRS
// Authors: Mylene Campana, Pierre Fernbach
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

# include <hpp/util/debug.hh>
# include <hpp/core/config-validations.hh>
# include <hpp/core/path-validation.hh>
# include <hpp/core/path-vector.hh>
# include <hpp/core/problem.hh>
# include <hpp/rbprm/planner/timed-parabola-path.hh>
# include <hpp/rbprm/planner/steering-method-parabola.hh>
# include <hpp/rbprm/rbprm-device.hh>
# include <hpp/rbprm/rbprm-path-validation.hh>
# include <hpp/rbprm/rbprm-validation-report.hh>
# include <hpp/pinocchio/configuration.hh>

namespace hpp {
  namespace rbprm {
    using core::value_type;
    using core::vector_t;
    using core::interval_t;
    using pinocchio::size_type;

    SteeringMethodParabola::SteeringMethodParabola
    (const core::Problem& problem):
      SteeringMethod (problem),
      device_ (problem.robot ()),
      distance_ (core::WeighedDistance::create (problem.robot())), weak_ (),
      g_(9.81), V0max_ (1.),
      Vimpmax_ (1.),
      mu_ (0.5), Dalpha_ (0.001), nLimit_ (6),initialConstraint_(true),
      V0_ (vector_t(3)), Vimp_ (vector_t(3))
    {
      hppDout(notice,"Constructor steering-method-parabola");
      try {
        V0max_ = (double)problem.getParameter ("vMax").floatValue();
        Vimpmax_ =V0max_;
      } catch (const std::exception& e) {
        std::cout<<"Warning : no velocity bounds set in problem, use 1.0 as default"<<std::endl;
      }
    }

    core::PathPtr_t SteeringMethodParabola::impl_compute
    (core::ConfigurationIn_t q1, core::ConfigurationIn_t q2)
    const {
      hppDout (info, "q_init: " << hpp::pinocchio::displayConfig (q1));
      hppDout (info, "q_goal: " << hpp::pinocchio::displayConfig (q2));
      hppDout (info, "g_: " << g_ << " , mu_: " << mu_ << " , V0max: " <<
               V0max_ << " , Vimpmax: " << Vimpmax_);

      core::PathPtr_t pp = compute_3D_path (q1, q2);
      return pp;
    }

    core::PathPtr_t
    SteeringMethodParabola::compute_3D_path (core::ConfigurationIn_t q1,
                                             core::ConfigurationIn_t q2)
    const {
      std::vector<std::string> filter;
      core::PathPtr_t validPart;
      const core::PathValidationPtr_t pathValidation
          (problem_.pathValidation ());
      RbPrmPathValidationPtr_t rbPathValidation = boost::dynamic_pointer_cast<RbPrmPathValidation>(pathValidation);
      pinocchio::RbPrmDevicePtr_t rbDevice =
          boost::dynamic_pointer_cast<pinocchio::RbPrmDevice> (device_.lock ());
      core::PathValidationReportPtr_t pathReport;
      if (!rbDevice)
        hppDout (error, "Device cannot be cast");
      if (!rbPathValidation)
        hppDout (error, "PathValidation cannot be cast");

      /* Define some constants */
      const size_type index = device_.lock ()->configSize()
          - device_.lock ()->extraConfigSpace ().dimension (); // ecs index
      const value_type x_0 = q1(0);
      const value_type y_0 = q1(1);
      const value_type z_0 = q1(2);
      const value_type x_imp = q2(0);
      const value_type y_imp = q2(1);
      const value_type z_imp = q2(2);
      const value_type X = x_imp - x_0;
      const value_type Y = y_imp - y_0;
      const value_type Z = z_imp - z_0;
      const value_type theta = atan2 (Y, X);
      const value_type x_theta_0 = cos(theta) * x_0 +  sin(theta) * y_0;
      const value_type x_theta_imp = cos(theta) * x_imp +  sin(theta) * y_imp;
      const value_type X_theta = X*cos(theta) + Y*sin(theta);
      const value_type phi = atan (mu_);
      hppDout (info, "x_0: " << x_0);
      hppDout (info, "y_0: " << y_0);
      hppDout (info, "z_0: " << z_0);
      hppDout (info, "x_imp: " << x_imp);
      hppDout (info, "y_imp: " << y_imp);
      hppDout (info, "z_imp: " << z_imp);
      hppDout (info, "X: " << X);
      hppDout (info, "Y: " << Y);
      hppDout (info, "Z: " << Z);
      hppDout (info, "theta: " << theta);
      hppDout (info, "x_theta_0: " << x_theta_0);
      hppDout (info, "x_theta_imp: " << x_theta_imp);
      hppDout (info, "X_theta: " << X_theta);
      hppDout (info, "phi: " << phi);

      /* 5th constraint: first cone */

      /*   Remove friction check (testing)
      //value_type delta1,delta2;
      if (1000 * (q1 (index) * q1 (index) + q1 (index+1) * q1 (index+1))
          > q1 (index+2) * q1 (index+2)) { // cone 1 not vertical
        if (!fiveth_constraint (q1, theta, 1, &delta1)) {
          hppDout (info, "plane_theta not intersecting first cone");
          initialConstraint_ = false;
          //   problem.parabolaResults_ [1] ++;
          return core::PathPtr_t ();
        }
      }
      else { // cone 1 "very" vertical
        delta1 = phi;
      }
      hppDout (info, "delta1: " << delta1);

      // 5th constraint: second cone //
      if (1000 * (q2 (index) * q2 (index) + q2 (index+1) * q2 (index+1))
          > q2 (index+2) * q2 (index+2)) { // cone 1 not vertical
        if (!fiveth_constraint (q2, theta, 2, &delta2)) {
          hppDout (info, "plane_theta not intersecting second cone");
          // problem.parabolaResults_ [1] ++;
          return core::PathPtr_t ();
        }
      }
      else { // cone 2 "very" vertical
        delta2 = phi;
      }
      hppDout (info, "delta2: " << delta2);

      // Definition of gamma_theta angles //
      const value_type n1_angle = atan2(q1 (index+2), cos(theta)*q1 (index) +
                                        sin(theta)*q1 (index+1));
      const value_type n2_angle = atan2(q2 (index+2), cos(theta)*q2 (index) +
                                        sin(theta)*q2 (index+1));
      hppDout (info, "n1_angle: " << n1_angle);
      hppDout (info, "n2_angle: " << n2_angle);

      // Only for demo without friction :
      //delta1 = 100.;
      //delta2 = 100.;

      const value_type alpha_0_min = n1_angle - delta1;
      const value_type alpha_0_max = n1_angle + delta1;
      alpha_0_min_ = alpha_0_min; alpha_0_max_ = alpha_0_max;
      hppDout (info, "alpha_0_min: " << alpha_0_min);
      hppDout (info, "alpha_0_max: " << alpha_0_max);



      value_type alpha_imp_min = n2_angle - M_PI - delta2;
      value_type alpha_imp_max = n2_angle - M_PI + delta2;
      if (n2_angle < 0) {
        alpha_imp_min = n2_angle + M_PI - delta2;
        alpha_imp_max = n2_angle + M_PI + delta2;
      }

*/ // Commented in order to remove non friction test (testing)

      value_type alpha_inf4;
      alpha_inf4 = atan (Z/X_theta);
      hppDout (info, "alpha_inf4: " << alpha_inf4);


      // Ajout pour test sans friction :
      const value_type alpha_0_min = -2 * M_PI;
      const value_type alpha_0_max = 2 * M_PI;
      value_type alpha_imp_min = -2 * M_PI;
      value_type alpha_imp_max = 2 * M_PI;
      const value_type n2_angle = 1.5;
      // ########### ^  a enlever   ^  ############ //

      hppDout (info, "alpha_imp_min: " << alpha_imp_min);
      hppDout (info, "alpha_imp_max: " << alpha_imp_max);


      value_type alpha_lim_plus;
      value_type alpha_lim_minus;
      bool fail = second_constraint (X_theta, Z, &alpha_lim_plus,
                                     &alpha_lim_minus);
      if (fail) {
        hppDout (info, "failed to apply 2nd constraint");
        // problem.parabolaResults_ [3] ++;
        return core::PathPtr_t ();
      }

      hppDout (info, "alpha_lim_plus: " << alpha_lim_plus);
      hppDout (info, "alpha_lim_minus: " << alpha_lim_minus);

      value_type alpha_imp_plus;
      value_type alpha_imp_minus;
      bool fail6 = sixth_constraint (X_theta, Z, &alpha_imp_plus,
                                     &alpha_imp_minus);
      if (fail6) {
        hppDout (info, "failed to apply 6th constraint");
        // problem.parabolaResults_ [3] ++;
        return core::PathPtr_t ();
      }

      hppDout (info, "alpha_imp_plus: " << alpha_imp_plus);
      hppDout (info, "alpha_imp_minus: " << alpha_imp_minus);

      value_type alpha_imp_inf;
      value_type alpha_imp_sup;
      // commented (Pierre) : test without friction
      bool fail3 = third_constraint (fail, X_theta, Z, alpha_imp_min, alpha_imp_max, &alpha_imp_sup, &alpha_imp_inf, n2_angle);

      if (fail3) {
        hppDout (info, "failed to apply 3rd constraint");
        // problem.parabolaResults_ [2] ++;
        return core::PathPtr_t ();
      }

      hppDout (info, "alpha_imp_inf: " << alpha_imp_inf);
      hppDout (info, "alpha_imp_sup: " << alpha_imp_sup);

      value_type alpha_inf_bound = 0;
      value_type alpha_sup_bound = 0;

      /* Define alpha_0 interval satisfying constraints */
      if (n2_angle > 0) {
        alpha_lim_minus = std::max(alpha_lim_minus, alpha_imp_minus);
        alpha_inf_bound = std::max (std::max(alpha_imp_inf,alpha_lim_minus),
                                    std::max(alpha_0_min, alpha_inf4 +Dalpha_));

        if (alpha_imp_min < -M_PI/2) {
          alpha_lim_plus = std::min(alpha_lim_plus, alpha_imp_plus);
          alpha_sup_bound = std::min(alpha_0_max,
                                     std::min(alpha_lim_plus,M_PI/2));
        }
        else { // alpha_imp_sup is worth
          alpha_lim_plus = std::min(alpha_lim_plus, alpha_imp_plus);
          alpha_sup_bound = std::min(std::min(alpha_0_max, M_PI/2),
                                     std::min(alpha_lim_plus, alpha_imp_sup));
        }
      }
      else { // down-oriented cone
        if (alpha_imp_max < M_PI/2) {
          alpha_lim_minus = std::max(alpha_lim_minus, alpha_imp_minus);
          alpha_inf_bound = std::max (std::max(alpha_imp_inf, alpha_lim_minus),
                                      std::max(alpha_0_min, alpha_inf4 +
                                               Dalpha_));
        }
        else { // alpha_imp_max >= M_PI/2 so alpha_imp_inf inaccurate
          alpha_lim_minus = std::max(alpha_lim_minus, alpha_imp_minus);
          alpha_inf_bound = std::max (std::max(alpha_0_min, alpha_inf4 +
                                               Dalpha_) , alpha_lim_minus);
        }
        alpha_lim_plus = std::min(alpha_lim_plus, alpha_imp_plus);
        alpha_sup_bound = std::min(std::min(alpha_0_max, M_PI/2),
                                   std::min(alpha_lim_plus, alpha_imp_sup));
      }

      hppDout (info, "alpha_inf_bound: " << alpha_inf_bound);
      hppDout (info, "alpha_sup_bound: " << alpha_sup_bound);

      if (alpha_inf_bound > alpha_sup_bound) {
        hppDout (info, "Constraints intersection is empty");
        //  problem.parabolaResults_ [2] ++;
        return core::PathPtr_t ();
      }

      /* Select alpha_0 as middle of ]alpha_inf_bound,alpha_sup_bound[ */
      value_type alpha = 0.5*(alpha_inf_bound + alpha_sup_bound);
      // for demo only :
      alpha = alpha_inf_bound + 0.1*(alpha_sup_bound - alpha_inf_bound);
      /*if(alpha < 0)
          alpha = 0;*/

      hppDout (info, "alpha: " << alpha);

      /* Verify that maximal heigh of smaller parab is not out of the bounds */
      const vector_t coefsInf = computeCoefficients (alpha_inf_bound, theta,
                                                     X_theta, Z, x_theta_0,
                                                     z_0);
      bool maxHeightRespected = parabMaxHeightRespected (coefsInf, x_theta_0,
                                                         x_theta_imp);
      if (!maxHeightRespected) {
        hppDout (info, "Path always out of the bounds");
        // problem.parabolaResults_ [0] ++;
        return core::PathPtr_t ();
      }

      /* Compute Parabola coefficients */
      vector_t coefs = computeCoefficients (alpha, theta, X_theta, Z,
                                            x_theta_0, z_0);
      hppDout (info, "coefs: " << coefs.transpose ());



      maxHeightRespected = parabMaxHeightRespected (coefs, x_theta_0,
                                                    x_theta_imp);

      // fill ROM report, loop on ROM
      initialROMnames_.clear (); endROMnames_.clear ();
      fillROMnames (q1, &initialROMnames_);
      fillROMnames (q2, &endROMnames_);
      hppDout (info, "initialROMnames_ size= " << initialROMnames_.size ());
      hppDout (info, "endROMnames_ size= " << endROMnames_.size ());

      // parabola path with alpha_0 as the middle of alpha_0 bounds
      ParabolaPathPtr_t pp = ParabolaPath::create (device_.lock(), q1, q2,
                                                   computeLength (q1, q2,coefs),
                                                   coefs, V0_, Vimp_,
                                                   initialROMnames_,
                                                   endROMnames_);
      // checks
      hppDout (info, "pp->V0_= " << pp->V0_);
      hppDout (info, "pp->Vimp_= " << pp->Vimp_);
      hppDout (info, "pp->initialROMnames_ size= " << pp->initialROMnames_.size ());
      hppDout (info, "pp->endROMnames_ size= " << pp->endROMnames_.size ());

      bool hasCollisions = !rbPathValidation->validate (pp, false, validPart, pathReport, filter);
      std::size_t n = 0;
      if (hasCollisions || !maxHeightRespected) {
        // problem.parabolaResults_ [0] ++; // not increased during dichotomy
        hppDout (info, "parabola has collisions, start dichotomy");
        while ((hasCollisions || !maxHeightRespected) && n < nLimit_ ) {
          alpha = dichotomy (alpha_inf_bound, alpha_sup_bound, n);


          hppDout (info, "alpha= " << alpha);
          coefs = computeCoefficients (alpha, theta, X_theta, Z, x_theta_0,z_0);
          maxHeightRespected = parabMaxHeightRespected (coefs, x_theta_0,
                                                        x_theta_imp);
          pp = ParabolaPath::create (device_.lock (), q1, q2,
                                     computeLength (q1, q2, coefs), coefs, V0_,
                                     Vimp_, initialROMnames_, endROMnames_);
          hasCollisions = !rbPathValidation->validate (pp, false, validPart, pathReport, filter);
          hppDout (info, "Dichotomy iteration: " << n);
          n++;
        }//while
      }
      if (hasCollisions || !maxHeightRespected) return core::PathPtr_t ();
      core::Configuration_t init = pp->initial();
      core::Configuration_t end = pp->end();
      init.segment<3>(index) = pp->V0_;
      init[index+5] = -g_;
      end.segment<3>(index) = pp->Vimp_;
      return TimedParabolaPath::create(device_.lock(),init,end,pp);
    }

    // From Pierre
    core::PathPtr_t SteeringMethodParabola::compute_random_3D_path
    (core::ConfigurationIn_t q1, core::ConfigurationIn_t q2,
     value_type *alpha0, value_type *v0) const
    {
      const core::PathValidationPtr_t pathValidation
          (problem_.pathValidation ());
      RbPrmPathValidationPtr_t rbPathValidation = boost::dynamic_pointer_cast<RbPrmPathValidation>(pathValidation);
      std::vector<std::string> filter;
      core::PathValidationReportPtr_t report;
      core::PathPtr_t validPart;
      /* Define some constants */
      //const core::size_type index = device_.lock ()->configSize() - device_.lock ()->extraConfigSpace ().dimension (); // ecs index
      const value_type x_0 = q1(0);
      const value_type y_0 = q1(1);
      const value_type z_0 = q1(2);
      const value_type x_imp = q2(0);
      const value_type y_imp = q2(1);
      const value_type z_imp = q2(2);
      value_type X = x_imp - x_0;
      value_type Y = y_imp - y_0;
      value_type Z = z_imp - z_0;
      const value_type theta = atan2 (Y, X);
      const value_type x_theta_0 = cos(theta) * x_0 +  sin(theta) * y_0;
      const value_type x_theta_imp = cos(theta) * x_imp +  sin(theta) * y_imp;

      if(alpha_1_minus_ < 0 )
        alpha_1_minus_ = 0; //otherwise we go in the wrong direction
      value_type interval = (alpha_1_plus_-alpha_1_minus_)/2.;  // according to friction cone computed in compute_3d_path
      value_type alpha = (((value_type) rand()/RAND_MAX) * interval) + alpha_1_minus_;
      value_type v = (((value_type) rand()/RAND_MAX) * V0max_);
      *alpha0 = alpha;
      *v0 = v;
      hppDout(notice,"Compute random path :");
      hppDout(notice,"alpha_rand = "<<alpha);
      hppDout(notice,"v_rand = "<<v);

      value_type t = 3; //TODO : find better way to do it
      value_type x_theta_f = v*cos(alpha)*t + x_theta_0;
      value_type x_f = x_theta_f*cos(theta);
      value_type y_f = x_theta_f*sin(theta);
      value_type z_f = v*sin(alpha)*t - 0.5*g_*t*t + z_0;

      X = x_f - x_0;
      Y = y_f - y_0;
      Z = z_f - z_0;
      hppDout(notice,"x_f = "<<x_f);
      hppDout(notice,"y_f = "<<y_f);
      hppDout(notice,"z_f = "<<z_f);
      core::ConfigurationPtr_t qnew (new core::Configuration_t(q2));
      (*qnew)[0] = x_f;
      (*qnew)[1] = y_f;
      (*qnew)[2] = z_f;


      const value_type X_theta = X*cos(theta) + Y*sin(theta);

      const value_type x_theta_0_dot = sqrt((g_ * X_theta * X_theta)
                                            /(2 * (X_theta*tan(alpha) - Z)));
      const value_type inv_x_th_dot_0_sq = 1/(x_theta_0_dot*x_theta_0_dot);
      //const value_type v = sqrt((1 + tan(alpha)*tan(alpha))) * x_theta_0_dot;
      //hppDout (notice, "v: " << v);
      const value_type Vimp = sqrt(1 + (-g_*X*inv_x_th_dot_0_sq+tan(alpha)) *(-g_*X*inv_x_th_dot_0_sq+tan(alpha))) * x_theta_0_dot; // x_theta_0_dot > 0
      hppDout (notice, "Vimp (after 3 seconde) : " << Vimp);

      /* Compute Parabola coefficients */
      vector_t coefs = computeCoefficients (alpha, theta, X_theta, Z,
                                            x_theta_0, z_0);
      hppDout (info, "coefs: " << coefs.transpose ());

      // parabola path with alpha_0 as the middle of alpha_0 bounds
      ParabolaPathPtr_t pp = ParabolaPath::create (device_.lock (), q1, *qnew,
                                                   computeLength (q1, *qnew,
                                                                  coefs),coefs);
      bool hasCollisions = !rbPathValidation->validate (pp, false, validPart,
                                                        report, filter);
      bool maxHeightRespected = parabMaxHeightRespected (coefs, x_theta_0,
                                                         x_theta_imp);

      if (hasCollisions || !maxHeightRespected) return core::PathPtr_t ();
      hppDout (notice, "Create path between : init : " << hpp::pinocchio::displayConfig(q1));
      hppDout (notice, "Create path between : goal : " << hpp::pinocchio::displayConfig(*qnew));
      return pp;
    }

    bool SteeringMethodParabola::second_constraint (const value_type& X,
                                                    const value_type& Y,
                                                    value_type *alpha_lim_plus,
                                                    value_type *alpha_lim_minus)
    const {
      bool fail = 0;
      const value_type A = g_*X*X;
      const value_type B = -2*X*V0max_*V0max_;
      const value_type C = g_*X*X + 2*Y*V0max_*V0max_;
      const value_type delta = B*B -4*A*C;

      if (delta < 0)
        fail = 1;
      else {
        if (X > 0) {
          *alpha_lim_plus = atan(0.5*(-B + sqrt(delta))/A);
          *alpha_lim_minus = atan(0.5*(-B - sqrt(delta))/A);
        }
        else {
          *alpha_lim_plus = atan(0.5*(-B + sqrt(delta))/A) + M_PI;
          *alpha_lim_minus = atan(0.5*(-B - sqrt(delta))/A) + M_PI;
        }
      }
      return fail;
    }

    bool SteeringMethodParabola::third_constraint
    (bool fail, const value_type& X, const value_type& Y,
     const value_type alpha_imp_min, const value_type alpha_imp_max,
     value_type *alpha_imp_sup, value_type *alpha_imp_inf,
     const value_type n2_angle) const {
      if (fail)
        return fail;
      else {
        // Pierre : disable this test, always return true (for testing)
        fail = false;
        *alpha_imp_sup = alpha_imp_max;
        *alpha_imp_inf = alpha_imp_min;
        return false;
        // ########## ^ a enlever ^ ######## //
        if (X > 0) {
          if (n2_angle >= 0) {
            if (alpha_imp_max > -M_PI/2) {
              *alpha_imp_sup = atan(-tan(alpha_imp_min)+2*Y/X);
              *alpha_imp_inf = atan(-tan(alpha_imp_max)+2*Y/X);
            } else
              fail = 1;
          }
          else { // n2_angle < 0
            if (alpha_imp_min < M_PI/2) {
              *alpha_imp_sup = atan(-tan(alpha_imp_min)+2*Y/X);
              *alpha_imp_inf = atan(-tan(alpha_imp_max)+2*Y/X);
            } else
              fail = 1;
          }
        }
        else { // X < 0   // TODO: cases n2_angle > 0 or < 0 (2D only)
          if (alpha_imp_min < -M_PI/2) {
            *alpha_imp_sup = atan(-tan(alpha_imp_min)+2*Y/X) + M_PI;
            *alpha_imp_inf = atan(-tan(alpha_imp_max)+2*Y/X) + M_PI;
          }
          else
            fail =1;
        }
      }//ifNotfail
      return fail;
    }

    // at least one z value must be >= z_0
    bool SteeringMethodParabola::fiveth_constraint
    (const core::ConfigurationIn_t q, const value_type theta,
     const int /*number*/, value_type *delta) const {
      const size_type index = device_.lock ()->configSize()
          - device_.lock ()->extraConfigSpace ().dimension ();
      const value_type U = q (index); // n_x
      const value_type V = q (index+1); // n_y
      const value_type W = q (index+2); // n_z
      hppDout (info, "U= " << U << ", V= " << V << ", W= " << W);
      const value_type phi = atan (mu_);
      const value_type denomK = U*U + V*V - W*W*mu_*mu_;
      const bool tanThetaDef = theta != M_PI /2 && theta != -M_PI /2;
      const value_type psi = M_PI/2 - atan2 (W,U*cos(theta)+V*sin(theta));
      hppDout (info, "psi: " << psi);
      const bool nonVerticalCone = (psi < -phi && psi >= -M_PI/2)
          || (psi > phi && psi < M_PI - phi)
          || (psi > M_PI + phi && psi <= 3*M_PI/2);
      value_type epsilon = 1;
      if (!nonVerticalCone && denomK < 0)
        epsilon = -1;

      if (denomK > -1e-6 && denomK < 1e-6) { // denomK (or 'A') = 0
        hppDout (info, "denomK = 0 case");
        if (tanThetaDef) {
          const value_type tanTheta = tan(theta);
          if (U + V*tanTheta != 0) {
            const value_type numH = mu_*mu_*(U*U+V*V*tanTheta*tanTheta)-U*U*tanTheta*tanTheta-V*V+2*U*V*tanTheta*(1+mu_*mu_)-W*W*(1+tanTheta*tanTheta);
            const value_type H = -numH/(2*(1+mu_*mu_)*fabs(W)*fabs(U+V*tanTheta));
            const value_type cos2delta = H/sqrt(1+tanTheta*tanTheta+H*H);
            hppDout (info, "cos(2*delta): " << cos2delta);
            *delta = 0.5*acos (cos2delta);
            hppDout (info, "delta: " << *delta);
            assert (*delta <= phi + 1e-5);
            return true;
          } else { // U + V*tanTheta = 0
            *delta = M_PI/4;
            hppDout (info, "delta: " << *delta);
            return true;
          }
        } else { // theta = +-pi/2
          if (V != 0) {
            const value_type L = -(V*V*(1+mu_*mu_)-1)/(2*(1+mu_*mu_)*fabs(V)*fabs(W));
            const value_type cos2delta = L/sqrt(1+L*L);
            hppDout (info, "cos(2*delta): " << cos2delta);
            *delta = 0.5*acos (cos2delta);
            hppDout (info, "delta: " << *delta);
            assert (*delta <= phi + 1e-5);
            return true;
          } else { // V = 0
            hppDout (info, "cone-plane intersection is a line");
            return false;
          }
        }
      } // if denomK = 0

      if (tanThetaDef) {
        value_type x_plus, x_minus, z_x_plus, z_x_minus;
        const value_type tantheta = tan(theta);
        value_type discr = (U*U+W*W)*mu_*mu_ - V*V - U*U*tantheta*tantheta + (V*V + W*W)*mu_*mu_*tantheta*tantheta + 2*(1+mu_*mu_)*U*V*tantheta;
        hppDout (info, "discr: " << discr);
        if (discr < 0) {
          hppDout (info, "cone-plane intersection empty");
          return false;
        }
        if (discr < 5e-2) {
          hppDout (info, "cone-plane intersection too small");
          return false;
        }
        const value_type K1 = (sqrt(discr) + U*W + U*W*mu_*mu_ + V*W*tantheta + V*W*mu_*mu_*tantheta)/denomK;
        const value_type K2 = (-sqrt(discr) + U*W + U*W*mu_*mu_ + V*W*tantheta + V*W*mu_*mu_*tantheta)/denomK;
        hppDout (info, "denomK= " << denomK);

        if (nonVerticalCone) {
          // non-vertical up
          hppDout (info, "non-vertical up");
          if (U*cos(theta) + V*sin(theta) < 0)
            x_minus = -0.5;
          else
            x_minus = 0.5;
          x_plus = x_minus;
          z_x_minus = x_minus*K2;
          z_x_plus = x_plus*K1;

          if (psi > M_PI/2) {// down: invert z_plus and z_minus
            hppDout (info, "non-vertical down");
            z_x_plus = x_minus*K2;
            z_x_minus = x_plus*K1;
          }
        }
        else { // "vertical" cone
          if (- phi <= psi && psi <=  phi) { // up
            hppDout (info, "vertical up");
            x_minus = 0.5;
            if (denomK < 0) {
              x_minus = 0.5;
              x_plus = -x_minus;
            }
            else {
              x_minus = -0.5;
              x_plus = x_minus;
            }
            z_x_minus = x_minus*K2;
            z_x_plus = x_plus*K1;
          }
          else { // down
            hppDout (info, "vertical down");
            if (denomK < 0) {
              x_minus = -0.5;
              x_plus = -x_minus;
            }
            else {
              x_minus = 0.5;
              x_plus = x_minus;
            }
            z_x_minus = x_minus*K2;
            z_x_plus = x_plus*K1;
          }
        }

        // plot outputs
        hppDout (info, "q: " << hpp::pinocchio::displayConfig (q));
        hppDout (info, "x_plus: " << x_plus);
        hppDout (info, "x_minus: " << x_minus);
        hppDout (info, "z_x_plus: " << z_x_plus);
        hppDout (info, "z_x_minus: " << z_x_minus);

        value_type cos2delta = epsilon*(1+tantheta*tantheta+K1*K2)/sqrt((1+tantheta*tantheta+K1*K1)*(1+tantheta*tantheta+K2*K2));
        hppDout (info, "cos(2*delta): " << cos2delta);
        *delta = 0.5*acos (cos2delta);
        hppDout (info, "delta: " << *delta);
        assert (*delta <= phi + 1e-5);
        return true;
      }
      else { // theta = +-pi/2
        value_type discr =  -U*U+(V*V + W*W)*(mu_*mu_);
        hppDout (info, "discr: " << discr);
        if (discr < 0) {
          hppDout (info, "cone-plane intersection empty");
          return false;
        }
        if (discr < 5e-2) {
          hppDout (info, "cone-plane intersection too small");
          return false;
        }
        value_type G1 = ((1+mu_*mu_)*V*W + sqrt(discr))/(denomK);
        value_type G2 = ((1+mu_*mu_)*V*W - sqrt(discr))/(denomK);
        value_type y = 1;
        if (theta == -M_PI /2)
          y = -1;
        hppDout (info, "y: " << y);
        value_type z_y_plus = G1*y; //TODO: sign selection of y
        value_type z_y_minus = G2*y;
        hppDout (info, "z_y_plus: " << z_y_plus);
        hppDout (info, "z_y_minus: " << z_y_minus);

        value_type cos2delta = epsilon*(1+G1*G2)/sqrt((1+G1*G1)*(1+G2*G2));
        hppDout (info, "cos(2*delta): " << cos2delta);
        *delta = 0.5*acos (cos2delta);
        hppDout (info, "delta: " << *delta);
        assert (*delta <= phi + 1e-5);
        return true;
      }
    }

    bool SteeringMethodParabola::sixth_constraint (const value_type& X,
                                                   const value_type& Y,
                                                   value_type *alpha_imp_plus,
                                                   value_type *alpha_imp_minus)
    const {
      bool fail = 0;
      const value_type A = g_*X*X;
      const value_type B = -2*X*Vimpmax_*Vimpmax_ - 4*X*Y*g_;
      const value_type C = g_*X*X + 2*Y*Vimpmax_*Vimpmax_ + 4*g_*Y*Y;
      const value_type delta = B*B -4*A*C;

      if (delta < 0)
        fail = 1;
      else {
        if (X > 0) {
          *alpha_imp_plus = atan(0.5*(-B + sqrt(delta))/A);
          *alpha_imp_minus = atan(0.5*(-B - sqrt(delta))/A);
        }
        else {
          *alpha_imp_plus = atan(0.5*(-B + sqrt(delta))/A) + M_PI;
          *alpha_imp_minus = atan(0.5*(-B - sqrt(delta))/A) + M_PI;
        }
      }
      return fail;
    }

    // Function equivalent to sqrt( 1 + f'(x)^2 )
    value_type SteeringMethodParabola::lengthFunction (const value_type x,
                                                       const vector_t coefs)
    const {
      const value_type y = sqrt (1+(2*coefs (0)*x+coefs (1))
                                 * (2*coefs (0)*x+coefs (1)));
      return y;
    }

   /* value_type SteeringMethodParabola::computeLength
    (const core::ConfigurationIn_t q1, const core::ConfigurationIn_t q2,
     const vector_t coefs) const {
      const int N = 6; // number -1 of interval sub-divisions
      // for N = 4, computation error ~= 1e-5.
      // for N = 20, computation error ~= 1e-11.
      value_type length = 0;
      value_type x1 = q1 (0);
      value_type x2 = q2 (0);
      const value_type theta = coefs (3);
      x1 = cos(theta) * q1 (0)  + sin(theta) * q1 (1); // x_theta_0
      x2 = cos(theta) * q2 (0) + sin(theta) * q2 (1); // x_theta_imp

      // Define integration bounds
      if (x1 > x2) { // re-order integration bounds
        const value_type xtmp = x1;
        x1 = x2;
        x2 = xtmp;
      }

      const value_type dx = (x2 - x1) / N; // integration step size
      for (int i=0; i<N; i++) {
        length += dx*( 0.166666667*lengthFunction (x1 + i*dx, coefs)
                       + 0.666666667*lengthFunction (x1 + (i+0.5)*dx, coefs)
                       + 0.166666667*lengthFunction (x1 + (i+1)*dx, coefs));
        // apparently, 1/6 and 2/3 are not recognized as floats ...
      }
      hppDout (info, "length = " << length);
      return length;
    }*/

    // test (pierre) :
    value_type SteeringMethodParabola::computeLength
        (const core::ConfigurationIn_t q1, const core::ConfigurationIn_t q2,
         const vector_t coefs) const {
      const value_type theta = coefs (3);
      const value_type X = q2[0] - q1[0];
      const value_type Y = q2[1] - q1[1];;
      // theta = coef[3]
      const value_type X_theta = X*cos(theta) + Y*sin(theta);
      return X_theta;
    }

    vector_t SteeringMethodParabola::computeCoefficients
    (const value_type alpha, const value_type theta,
     const value_type X_theta, const value_type Z,
     const value_type x_theta_0, const value_type z_0) const {
      vector_t coefs (7);
      const value_type x_theta_0_dot = sqrt((g_ * X_theta * X_theta)
                                            /(2 * (X_theta*tan(alpha) - Z)));
      const value_type inv_x_th_dot_0_sq = 1/(x_theta_0_dot*x_theta_0_dot);
      coefs (0) = -0.5*g_*inv_x_th_dot_0_sq;
      coefs (1) = tan(alpha) + g_*x_theta_0*inv_x_th_dot_0_sq;
      coefs (2) = z_0 - tan(alpha)*x_theta_0 -
          0.5*g_*x_theta_0*x_theta_0*inv_x_th_dot_0_sq;
      coefs (3) = theta;
      coefs (4) = alpha;
      coefs (5) = x_theta_0_dot;
      coefs (6) = x_theta_0;
      // Also compute initial and final velocities
      const value_type V0 = sqrt((1 + tan(alpha)*tan(alpha))) * x_theta_0_dot;
      const value_type Vimp = sqrt(1 + (-g_*X_theta*inv_x_th_dot_0_sq+tan(alpha)) *(-g_*X_theta*inv_x_th_dot_0_sq+tan(alpha))) * x_theta_0_dot;
      hppDout (info, "V0: " << V0);
      hppDout (info, "Vimp: " << Vimp);
      V0_ [0] = x_theta_0_dot*cos(theta);
      V0_ [1] = x_theta_0_dot*sin(theta);
      V0_ [2] = V0*sin(alpha);
      Vimp_ [0] = x_theta_0_dot*cos(theta); // x_theta_imp_dot = x_theta_0_dot
      Vimp_ [1] = x_theta_0_dot*sin(theta);
      Vimp_ [2] = -g_*X_theta/x_theta_0_dot + x_theta_0_dot*tan(alpha);
      return coefs;
    }

    bool SteeringMethodParabola::parabMaxHeightRespected
    (const vector_t coefs, const value_type x_theta_0,
     const value_type x_theta_imp) const {
      const value_type x_theta_max = - 0.5 * coefs (1) / coefs (0);
      const value_type z_x_theta_max = coefs (0)*x_theta_max*x_theta_max +
          coefs (1)*x_theta_max + coefs (2);
      if (x_theta_0 <= x_theta_max && x_theta_max <= x_theta_imp) {
        if (z_x_theta_max > device_.lock ()->rootJoint()->upperBound (2)) {
          //hppDout (info, "z_x_theta_max: " << z_x_theta_max);
          return false;
        }
      }
      return true;
    }

    value_type SteeringMethodParabola::dichotomy (value_type a_inf,
                                                  value_type a_plus,
                                                  std::size_t n) const {
      value_type alpha, e; // e in ]0,1[
      switch (n) { // NLimit_ <= 6, otherwise fill missing values for n>5
        case 0: e = 0.25; break;
        case 1: e = 0.75; break;
        case 2: e = 0.125; break;
        case 3: e = 0.375; break;
        case 4: e = 0.625; break;
        case 5: e = 0.875; break;
        default: e = 0.5; break; // not supposed to happen
      }
      alpha = e*a_plus + (1-e)*a_inf;
      return alpha;
    }

    void SteeringMethodParabola::fillROMnames
    (core::ConfigurationIn_t q, std::vector <std::string> * ROMnames) const {
      core::ValidationReportPtr_t report;
      const core::Configuration_t config = q;
      problem_.configValidations()->validate(config, report);
      core::RbprmValidationReportPtr_t rbReport =
          boost::dynamic_pointer_cast<core::RbprmValidationReport> (report);
      if(rbReport){
        hppDout (info, "nbROM= " << rbReport->ROMReports.size());
        for (std::map<std::string,core::CollisionValidationReportPtr_t>::const_iterator it = rbReport->ROMReports.begin(); it != rbReport->ROMReports.end(); it++) {
          std::string ROMname = it->first;
          hppDout (info, "ROMname= " << ROMname);
          (*ROMnames).push_back (ROMname);
        }

      }else{
        hppDout(error,"Validation Report cannot be cast");

      }
    }

  } // namespace rbprm
} // namespace hpp
