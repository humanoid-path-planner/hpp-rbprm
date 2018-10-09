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
# define PP_RBPRM_TIME_DEPENDANT_HH

# include <hpp/rbprm/config.hh>
# include <hpp/core/equation.hh>
# include <vector>

namespace hpp {
namespace rbprm {
namespace interpolation {
    struct RightHandSideFunctor {
        virtual void operator() (constraints::vectorOut_t output, const constraints::value_type& input, pinocchio::ConfigurationOut_t conf)
        const = 0;
    };
    typedef boost::shared_ptr <const RightHandSideFunctor> RightHandSideFunctorPtr_t;

    struct TimeDependant
    {
        void operator() (const constraints::value_type s, pinocchio::ConfigurationOut_t conf) const
        {
            (*rhsFunc_) (eq_->nonConstRightHandSide(), s, conf);
        }

        TimeDependant (const core::EquationPtr_t& eq,
          const RightHandSideFunctorPtr_t rhs):
        eq_ (eq), rhsFunc_ (rhs)
        {}

        TimeDependant (const TimeDependant& other) :
        eq_ (other.eq_), rhsFunc_ (other.rhsFunc_)
        {}

        core::EquationPtr_t eq_;
        RightHandSideFunctorPtr_t rhsFunc_;
    }; // class TimeDependant

    typedef std::vector <TimeDependant> T_TimeDependant;
    typedef T_TimeDependant::const_iterator CIT_TimeDependant;
    } // namespace interpolation
  } // namespace rbprm
} // namespace hpp
#endif // PP_RBPRM_TIME_DEPENDANT_HH
