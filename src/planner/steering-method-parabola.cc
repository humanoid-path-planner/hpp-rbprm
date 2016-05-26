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

# include <hpp/util/debug.hh>
# include <hpp/core/path-vector.hh>
# include <hpp/rbprm/planner/parabola-path.hh>
# include <hpp/rbprm/planner/steering-method-parabola.hh>
# include <hpp/model/collision-object.hh>
# include <hpp/model/joint.hh>
# include <hpp/model/joint-configuration.hh>
# include <hpp/model/configuration.hh>
# include <hpp/core/problem.hh>

namespace hpp {
  namespace rbprm {
    using model::displayConfig;
    using core::value_type;
    using core::vector_t;
    using core::interval_t;
    using model::size_type;
    
    SteeringMethodParabola::SteeringMethodParabola (const core::ProblemPtr_t& problem) :
      SteeringMethod (problem), device_ (problem->robot()),
      distance_ (core::WeighedDistance::create (problem->robot())), weak_ (),
      g_(9.81), V0max_ (5), Vimpmax_ (10), mu_ (0.5), Dalpha_ (0.001),
      workspaceDim_ (false),initialConstraint_(true)
    {
      hppDout(notice,"Constructor steering-method-parabola");
    }
    
    SteeringMethodParabola::SteeringMethodParabola
    (const core::ProblemPtr_t &problem, const core::WeighedDistancePtr_t& distance) :
      SteeringMethod (problem), device_ (problem->robot()), distance_ (distance),
      weak_ (), g_(9.81), V0max_ (7), Vimpmax_ (10),  mu_ (0.5),
      Dalpha_ (0.001), workspaceDim_ (false),initialConstraint_(true)
    {
      hppDout(notice,"Constructor steering-method-parabola");
    }
    
    core::PathPtr_t SteeringMethodParabola::impl_compute (core::ConfigurationIn_t q1,
                                                          core::ConfigurationIn_t q2)
    const
    {
      hppDout(notice,"begin impl_compute");
      hppDout(info, "extra conf size : "<<device_.lock()->extraConfigSpace().dimension());
      hppDout (notice, "q_init: " << displayConfig (q1));
      hppDout (notice, "q_goal: " << displayConfig (q2));
      hppDout (info, "g_: " << g_ << " , mu_: " << mu_ << " , V0max: " <<
               V0max_ << " , Vimpmax: " << Vimpmax_);
      
      /* Define dimension: 2D or 3D */
      std::string name = device_.lock ()->getJointVector () [0]->name ();
      if (name == "base_joint_xyz") // 3D (2D by default)
        workspaceDim_ = true;
      
      core::PathPtr_t pp;
      if (!workspaceDim_)
        pp = compute_2D_path (q1, q2);
      else
        pp = compute_3D_path (q1, q2);
      
      /*if(!pp)
        pp = compute_random_3D_path(q1,q2);*/
      return pp;
    }
    /*
    core::PathPtr_t SteeringMethodParabola::impl_compute (core::ConfigurationIn_t q1,
                                                          core::ConfigurationIn_t q2, value_type& alpha0Min, value_type& alpha0Max) const{
      hppDout(notice,"begin impl_compute with bounds");
      hppDout(info, "extra conf size : "<<device_.lock()->extraConfigSpace().dimension());
      hppDout (notice, "q_init: " << displayConfig (q1));
      hppDout (notice, "q_goal: " << displayConfig (q2));
      hppDout (info, "g_: " << g_ << " , mu_: " << mu_ << " , V0max: " <<
               V0max_ << " , Vimpmax: " << Vimpmax_);
      
      // Define dimension: 2D or 3D 
      std::string name = device_.lock ()->getJointVector () [0]->name ();
      if (name == "base_joint_xyz") // 3D (2D by default)
        workspaceDim_ = true;
      
      core::PathPtr_t pp;
      if (!workspaceDim_)
        pp = compute_2D_path (q1, q2);
      else
        pp = compute_3D_path (q1, q2);
      
      //if(!pp)
      //  pp = compute_random_3D_path(q1,q2);
      alpha0Min = alpha_0_min_;
      alpha0Max = alpha_0_max_;
      return pp;
    }
    
    core::PathPtr_t SteeringMethodParabola::impl_compute (core::ConfigurationIn_t q1,
                                                          core::ConfigurationIn_t q2, value_type& alpha0Min, value_type& alpha0Max, value_type& Z, value_type& X_theta) const{
      hppDout(notice,"begin impl_compute with bounds");
      hppDout(info, "extra conf size : "<<device_.lock()->extraConfigSpace().dimension());
      hppDout (notice, "q_init: " << displayConfig (q1));
      hppDout (notice, "q_goal: " << displayConfig (q2));
      hppDout (info, "g_: " << g_ << " , mu_: " << mu_ << " , V0max: " <<
               V0max_ << " , Vimpmax: " << Vimpmax_);
      
      // Define dimension: 2D or 3D 
      std::string name = device_.lock ()->getJointVector () [0]->name ();
      if (name == "base_joint_xyz") // 3D (2D by default)
        workspaceDim_ = true;
      
      core::PathPtr_t pp;
      if (!workspaceDim_)
        pp = compute_2D_path (q1, q2);
      else
        pp = compute_3D_path (q1, q2);
      
      //if(!pp)
      //  pp = compute_random_3D_path(q1,q2);
      alpha0Min = alpha_0_min_;
      alpha0Max = alpha_0_max_;
      Z = Z_;
      X_theta= X_theta_;
      return pp;
    }
    */
    core::PathPtr_t SteeringMethodParabola::compute_2D_path (core::ConfigurationIn_t q1,
                                                             core::ConfigurationIn_t q2)
    const {
      /* Define some constants */
      const value_type x_0 = q1(0);
      const value_type y_0 = q1(1);
      const value_type x_imp = q2(0);
      const value_type y_imp = q2(1);
      const value_type X = x_imp - x_0;
      const value_type Y = y_imp - y_0;
      const value_type phi = atan (mu_);
      hppDout (info, "x_0: " << x_0);
      hppDout (info, "y_0: " << y_0);
      hppDout (info, "x_imp: " << x_imp);
      hppDout (info, "y_imp: " << y_imp);
      hppDout (info, "X: " << X);
      hppDout (info, "Y: " << Y);
      hppDout (info, "phi: " << phi);
      
      value_type gamma_0 = atan2(q1 (3), q1 (2)) - M_PI/2;
      value_type gamma_imp = atan2(q2 (3), q2 (2)) - M_PI/2;
      
      // keep gamma in [-pi/2 ; pi/2]
      if (gamma_0 > M_PI/2)
        gamma_0 = gamma_0 - M_PI;
      if (gamma_0 < -M_PI/2)
        gamma_0 = gamma_0 + M_PI;
      if (gamma_imp > M_PI/2)
        gamma_imp = gamma_imp - M_PI;
      if (gamma_imp < -M_PI/2)
        gamma_imp = gamma_imp + M_PI;
      
      hppDout (info, "corrected gamma_imp: " << gamma_imp);
      
      const value_type alpha_0_min = M_PI/2 + gamma_0 - phi;
      const value_type alpha_0_max = M_PI/2 + gamma_0 + phi;
      hppDout (info, "alpha_0_min: " << alpha_0_min);
      hppDout (info, "alpha_0_max: " << alpha_0_max);
      
      value_type alpha_inf4;
      if (X > 0)
        alpha_inf4 = atan (Y/X);
      else
        alpha_inf4 = atan (Y/X) + M_PI;
      hppDout (info, "alpha_inf4: " << alpha_inf4);
      
      const value_type alpha_imp_min = -M_PI/2 + gamma_imp - phi;
      const value_type alpha_imp_max = -M_PI/2 + gamma_imp + phi;
      hppDout (info, "alpha_imp_min: " << alpha_imp_min);
      hppDout (info, "alpha_imp_max: " << alpha_imp_max);
      
      value_type alpha_lim_plus;
      value_type alpha_lim_minus;
      
      bool fail = second_constraint (X, Y, &alpha_lim_plus, &alpha_lim_minus);
      hppDout (info, "alpha_lim_plus: " << alpha_lim_plus);
      hppDout (info, "alpha_lim_minus: " << alpha_lim_minus);
      
      if (fail) {
        hppDout (notice, "failed to apply 2nd constraint");
        return core::PathPtr_t ();
      }
      
      value_type alpha_imp_inf;
      value_type alpha_imp_sup;
      
      // n2_angle = pi/2 - gamma_imp
      fail = third_constraint (fail, X, Y, alpha_imp_min, alpha_imp_max,
                               &alpha_imp_sup, &alpha_imp_inf,
                               M_PI/2 - gamma_imp);
      hppDout (info, "alpha_imp_inf: " << alpha_imp_inf);
      hppDout (info, "alpha_imp_sup: " << alpha_imp_sup);
      
      if (fail) {
        hppDout (notice, "failed to apply 3rd constraint");
        return core::PathPtr_t ();
      }
      
      value_type alpha_inf_bound = 0;
      value_type alpha_sup_bound = 0;
      
      /* Define alpha_0 satisfying constraints and minimizing parab length */
      if (X > 0) {
        alpha_inf_bound = std::max (std::max(alpha_imp_inf, alpha_lim_minus),
                                    std::max(alpha_0_min,
                                             alpha_inf4 + Dalpha_));
        if (alpha_imp_min < -M_PI/2)
          alpha_sup_bound = std::min(alpha_0_max, alpha_lim_plus);
        else
          alpha_sup_bound = std::min(alpha_0_max,
                                     std::min(alpha_lim_plus, alpha_imp_sup));
      }
      else {
        alpha_sup_bound = std::min (std::min(alpha_imp_sup,alpha_lim_plus),
                                    std::min(alpha_0_max,
                                             alpha_inf4 - Dalpha_));
        if (alpha_imp_max > -M_PI/2)
          alpha_inf_bound = std::max(alpha_0_min, alpha_lim_minus);
        else
          alpha_inf_bound = std::max(alpha_0_min,
                                     std::max(alpha_lim_minus, alpha_imp_inf));
      }
      
      hppDout (info, "alpha_inf_bound: " << alpha_inf_bound);
      hppDout (info, "alpha_sup_bound: " << alpha_sup_bound);
      
      if (alpha_inf_bound > alpha_sup_bound) {
        hppDout (notice, "Constraints intersection is empty");
        return core::PathPtr_t ();
      }
      
      /* Select alpha_0 */
      value_type alpha = alpha_inf_bound;
      if (X < 0) alpha = alpha_sup_bound;
      
      /* Compute Parabola */
      const int signX = (X > 0) - (X < 0);
      
      const value_type x0_dot = signX*sqrt((g_*X*X)/(2*(X*tan(alpha) - Y)));
      const value_type y0_dot = tan(alpha)*x0_dot;
      const value_type V0 = sqrt(x0_dot*x0_dot + y0_dot*y0_dot);
      hppDout (info, "V0: " << V0);
      
      vector_t coefs (3);
      coefs (0) = -0.5*g_/(x0_dot*x0_dot);
      coefs (1) = tan(alpha) + g_*x_0/(x0_dot*x0_dot);
      coefs (2) = y_0 - tan(alpha)*x_0 -
          0.5*g_*x_0*x_0/(x0_dot*x0_dot);
      hppDout (notice, "coefs: " << coefs.transpose ());
      
      ParabolaPathPtr_t pp = ParabolaPath::create (device_.lock (), q1, q2,
                                                   computeLength (q1, q2,coefs),
                                                   coefs);
      hppDout (notice, "path: " << *pp);
      return pp;
    }
    
    
    core::PathPtr_t SteeringMethodParabola::compute_random_3D_path (core::ConfigurationIn_t q1,
                                                                    core::ConfigurationIn_t q2, value_type *alpha0, value_type *v0) const
    {
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
      vector_t coefs (5);
      coefs.resize (5);
      coefs (0) = -0.5*g_*inv_x_th_dot_0_sq;
      coefs (1) = tan(alpha) + g_*x_theta_0*inv_x_th_dot_0_sq;
      coefs (2) = z_0 - tan(alpha)*x_theta_0 -
          0.5*g_*x_theta_0*x_theta_0*inv_x_th_dot_0_sq;
      coefs (3) = theta; // NOT tan(theta) !
      coefs (4) = -tan(theta)*x_0 + y_0;
      hppDout (info, "coefs: " << coefs.transpose ());
      
      /* Verify that maximal height is not out of the bounds */
      const value_type x_theta_max = - 0.5 * coefs (1) / coefs (0);
      const value_type z_x_theta_max = coefs (0)*x_theta_max*x_theta_max +
          coefs (1)*x_theta_max + coefs (2);
      if (x_theta_0 <= x_theta_max && x_theta_max <= x_theta_f) {
        if (z_x_theta_max > device_.lock ()->rootJoint()->upperBound (2)) {
          hppDout (notice, "z_x_theta_max: " << z_x_theta_max);
          hppDout (notice, "Path is out of the bounds");
          return core::PathPtr_t ();
        }
      }
      ParabolaPathPtr_t pp = ParabolaPath::create (device_.lock (), q1, *qnew,
                                                   computeLength (q1, *qnew,coefs),
                                                   coefs);
      hppDout (notice, "Create path between : init : " << displayConfig(q1));
      hppDout (notice, "Create path between : goal : " << displayConfig(*qnew));
      
      return pp;
    }
    
    
    core::PathPtr_t SteeringMethodParabola::compute_3D_path (core::ConfigurationIn_t q1,
                                                             core::ConfigurationIn_t q2)
    const {
      /* Define some constants */
      const core::size_type index = device_.lock ()->configSize()
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
      hppDout (info, "index: " << index);
      hppDout (info, "ecs size: " << device_.lock ()->extraConfigSpace ().dimension ());
      
      
      /* 5th constraint: first cone */
      value_type delta1;
      if (1000 * (q1 (index) * q1 (index) + q1 (index+1) * q1 (index+1))
          > q1 (index+2) * q1 (index+2)) { // cone 1 not vertical
        if (!fiveth_constraint (q1, theta, 1, &delta1)) {
          hppDout (notice, "plane_theta not intersecting first cone");
          initialConstraint_=false;
          return core::PathPtr_t ();
        }
      }
      else { // cone 1 "very" vertical
        delta1 = phi;
      }
      hppDout (info, "delta1: " << delta1);
      
      /* 5th constraint: second cone */
      value_type delta2;
      if (1000 * (q2 (index) * q2 (index) + q2 (index+1) * q2 (index+1))
          > q2 (index+2) * q2 (index+2)) { // cone 1 not vertical
        if (!fiveth_constraint (q2, theta, 2, &delta2)) {
          hppDout (notice, "plane_theta not intersecting second cone");
          return core::PathPtr_t ();
        }
      }
      else { // cone 2 "very" vertical
        delta2 = phi;
      }
      hppDout (info, "delta2: " << delta2);
      
      /* Definition of gamma_theta angles */
      const value_type n1_angle = atan2(q1 (index+2), cos(theta)*q1 (index) +
                                        sin(theta)*q1 (index+1));
      const value_type n2_angle = atan2(q2 (index+2), cos(theta)*q2 (index) +
                                        sin(theta)*q2 (index+1));
      hppDout (notice, "n1_angle: " << n1_angle);
      hppDout (notice, "n2_angle: " << n2_angle);
      
      const value_type alpha_0_min = n1_angle - delta1;
      const value_type alpha_0_max = n1_angle + delta1;
      alpha_1_plus_=alpha_0_max;
      alpha_1_minus_=alpha_0_min;
      hppDout (notice, "alpha_0_min: " << alpha_0_min);
      hppDout (notice, "alpha_0_max: " << alpha_0_max);
      
      value_type alpha_inf4;
      alpha_inf4 = atan (Z/X_theta);
      hppDout (info, "alpha_inf4: " << alpha_inf4);
      
      value_type alpha_imp_min = n2_angle - M_PI - delta2;
      value_type alpha_imp_max = n2_angle - M_PI + delta2;
      if (n2_angle < 0) {
        alpha_imp_min = n2_angle + M_PI - delta2;
        alpha_imp_max = n2_angle + M_PI + delta2;
      }
      hppDout (info, "alpha_imp_min: " << alpha_imp_min);
      hppDout (info, "alpha_imp_max: " << alpha_imp_max);
      
      value_type alpha_lim_plus;
      value_type alpha_lim_minus;
      bool fail = second_constraint (X_theta, Z, &alpha_lim_plus,
                                     &alpha_lim_minus);
      hppDout (info, "alpha_lim_plus: " << alpha_lim_plus);
      hppDout (info, "alpha_lim_minus: " << alpha_lim_minus);
      
      if (fail) {
        hppDout (notice, "failed to apply 2nd constraint");
        return core::PathPtr_t ();
      }
      
      value_type alpha_imp_plus;
      value_type alpha_imp_minus;
      bool fail6 = sixth_constraint (X_theta, Z, &alpha_imp_plus,
                                     &alpha_imp_minus);
      hppDout (info, "alpha_imp_plus: " << alpha_imp_plus);
      hppDout (info, "alpha_imp_minus: " << alpha_imp_minus);
      
      if (fail6) {
        hppDout (notice, "failed to apply 6th constraint");
        return core::PathPtr_t ();
      }
      
      value_type alpha_imp_inf;
      value_type alpha_imp_sup;
      bool fail3 = third_constraint (fail, X_theta, Z, alpha_imp_min,
                                     alpha_imp_max, &alpha_imp_sup,
                                     &alpha_imp_inf, n2_angle);
      if (fail3) {
        hppDout (notice, "failed to apply 3rd constraint");
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
        hppDout (notice, "Constraints intersection is empty");
        return core::PathPtr_t ();
      }
      
      /* Select alpha_0 */
      //value_type alpha = alpha_inf_bound; //debug
      value_type alpha = 0.5*(alpha_inf_bound + alpha_sup_bound); // better
      hppDout (notice, "alpha: " << alpha);
      alpha_sup_bound;
      alpha_inf_bound;
      
      
      // TODO (Pierre) : Here, choose all couple alpha / V which satisfy constraints
      
      /* Compute Parabola initial and final velocities */
      const value_type x_theta_0_dot = sqrt((g_ * X_theta * X_theta)
                                            /(2 * (X_theta*tan(alpha) - Z)));
      const value_type inv_x_th_dot_0_sq = 1/(x_theta_0_dot*x_theta_0_dot);
      const value_type V0 = sqrt((1 + tan(alpha)*tan(alpha))) * x_theta_0_dot;
      hppDout (notice, "V0: " << V0);
      const value_type Vimp = sqrt(1 + (-g_*X*inv_x_th_dot_0_sq+tan(alpha)) *(-g_*X*inv_x_th_dot_0_sq+tan(alpha))) * x_theta_0_dot; // x_theta_0_dot > 0
      hppDout (notice, "Vimp: " << Vimp);
      
      /* Compute Parabola coefficients */
      vector_t coefs (5);
      coefs.resize (5);
      coefs (0) = -0.5*g_*inv_x_th_dot_0_sq;
      coefs (1) = tan(alpha) + g_*x_theta_0*inv_x_th_dot_0_sq;
      coefs (2) = z_0 - tan(alpha)*x_theta_0 -
          0.5*g_*x_theta_0*x_theta_0*inv_x_th_dot_0_sq;
      coefs (3) = theta; // NOT tan(theta) !
      coefs (4) = -tan(theta)*x_0 + y_0;
      hppDout (info, "coefs: " << coefs.transpose ());
      
      /* Verify that maximal height is not out of the bounds */
      const value_type x_theta_max = - 0.5 * coefs (1) / coefs (0);
      const value_type z_x_theta_max = coefs (0)*x_theta_max*x_theta_max +
          coefs (1)*x_theta_max + coefs (2);
      if (x_theta_0 <= x_theta_max && x_theta_max <= x_theta_imp) {
        if (z_x_theta_max > device_.lock ()->rootJoint()->upperBound (2)) {
          hppDout (notice, "z_x_theta_max: " << z_x_theta_max);
          hppDout (notice, "Path is out of the bounds");
          return core::PathPtr_t ();
        }
      }
      ParabolaPathPtr_t pp = ParabolaPath::create (device_.lock (), q1, q2,
                                                   computeLength (q1, q2,coefs),
                                                   coefs);
      pp->alphaMax_=alpha_sup_bound;
      pp->alphaMin_ = alpha_inf_bound;
      pp->Z_=Z;
      pp->Xtheta_ = X_theta;
      pp->alpha_ = alpha;
     // pp->setAlpha(alpha,alpha_inf_bound,alpha_sup_bound);
      hppDout (notice, "path: " << *pp);
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
        if (X > 0) {
          if (n2_angle > 0) {
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
    bool SteeringMethodParabola::fiveth_constraint (const core::ConfigurationIn_t q,
                                                    const value_type theta,
                                                    const int/* number*/,
                                                    value_type *delta) const {
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
        hppDout (info, "q: " << displayConfig (q));
        hppDout (info, "x_plus: " << x_plus);
        hppDout (info, "x_minus: " << x_minus);
        hppDout (info, "z_x_plus: " << z_x_plus);
        hppDout (info, "z_x_minus: " << z_x_minus);
        
        value_type cos2delta = epsilon*(1+tantheta*tantheta+K1*K2)/sqrt((1+tantheta*tantheta+K1*K1)*(1+tantheta*tantheta+K2*K2));
        hppDout (info, "cos(2*delta): " << cos2delta);
        *delta = 0.5*acos (cos2delta);
        hppDout (info, "delta: " << *delta);
        if (*delta <= phi + 1e-5)
          return true;
        else
          return false;
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
        if (*delta <= phi + 1e-5)
          return true;
        else
          return false;
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
    
    value_type SteeringMethodParabola::computeLength
    (const core::ConfigurationIn_t q1, const core::ConfigurationIn_t q2,
     const vector_t coefs) const {
      const int N = 6; // number -1 of interval sub-divisions
      // for N = 4, computation error ~= 1e-5.
      // for N = 20, computation error ~= 1e-11.
      value_type length = 0;
      value_type x1 = q1 (0);
      value_type x2 = q2 (0);
      
      if (workspaceDim_) { // 3D
        const value_type theta = coefs (3);
        x1 = cos(theta) * q1 (0)  + sin(theta) * q1 (1); // x_theta_0
        x2 = cos(theta) * q2 (0) + sin(theta) * q2 (1); // x_theta_imp
      }
      
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
      hppDout (notice, "length = " << length);
      return length;
    }
    
  } // namespace core
} // namespace hpp
