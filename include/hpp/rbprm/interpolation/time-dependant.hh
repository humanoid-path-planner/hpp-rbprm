/// Copyright (c) 2015 CNRS
/// Authors: Joseph Mirabel
///
///
// This file is part of hpp-wholebody-step.
// hpp-wholebody-step-planner is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-wholebody-step-planner is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-wholebody-step-planner. If not, see
// <http://www.gnu.org/licenses/>.

#ifndef PP_RBPRM_TIME_DEPENDANT_HH
#define PP_RBPRM_TIME_DEPENDANT_HH

#include <hpp/constraints/implicit.hh>
#include <hpp/rbprm/config.hh>
#include <vector>

namespace hpp {
namespace rbprm {
namespace interpolation {

/// Time varying right hand side of constraint
struct RightHandSideFunctor {
  /// Compute and set right hand side of constraint
  /// \param eq Implicit constraint,
  /// \param input real valued parameter between 0 and 1.
  virtual void operator()(constraints::ImplicitPtr_t eq,
                          const constraints::value_type& input,
                          pinocchio::ConfigurationOut_t conf) const = 0;
};
typedef std::shared_ptr<const RightHandSideFunctor> RightHandSideFunctorPtr_t;

/// Set time varying right hand side of a constraint (constraints::Implicit)
///
/// This class stores
///  \li an instance of implicit constraint and
///  \li an instance of time varying right hand side (RightHandSideFunctor)
///
/// Call to operator () set the right hand side of the constraint to the
/// value at a given time.
/// \note Parameterization may be normalized between 0 and 1
/// (see interpolation::funEvaluator for an example).
struct TimeDependant {
  /// Set time varying right hand side
  /// \param s time value in interval [0,1],
  void operator()(const constraints::value_type s,
                  pinocchio::ConfigurationOut_t conf) const {
    (*rhsFunc_)(eq_, s, conf);
  }

  /// Constructor
  /// \param eq implicit constraint,
  /// \param rhs time-varying right hand side.
  TimeDependant(const constraints::ImplicitPtr_t& eq,
                const RightHandSideFunctorPtr_t rhs)
      : eq_(eq), rhsFunc_(rhs) {}

  TimeDependant(const TimeDependant& other)
      : eq_(other.eq_), rhsFunc_(other.rhsFunc_) {}

  constraints::ImplicitPtr_t eq_;
  RightHandSideFunctorPtr_t rhsFunc_;
};  // class TimeDependant

typedef std::vector<TimeDependant> T_TimeDependant;
typedef T_TimeDependant::const_iterator CIT_TimeDependant;
}  // namespace interpolation
}  // namespace rbprm
}  // namespace hpp
#endif  // PP_RBPRM_TIME_DEPENDANT_HH
