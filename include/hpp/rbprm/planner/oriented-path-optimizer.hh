//
// Copyright (c) 2017 CNRS
// Authors: Pierre Fernbach
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


#ifndef HPP_RBPRM_ORIENTED_PATH_OPTIMIZER_HH
#define HPP_RBPRM_ORIENTED_PATH_OPTIMIZER_HH

#include <hpp/core/path-optimizer.hh>
#include <hpp/rbprm/planner/rbprm-steering-kinodynamic.hh>
#include <hpp/rbprm/rbprm-path-validation.hh>


namespace hpp {
  namespace rbprm {
    /// \addtogroup path_optimization
    /// \{

    /// Oriented Path Optimizer
    ///
    /// Optimizer that try to use OrientedKinodynamicPath along the computed path
    /// Only use oriented path if it's collision free and if the previous/next path can be adjusted
    ///  to the new orientation
    ///
    /// \note The optimizer assumes that the input path is a vector of optimal
    ///       paths for the distance function.



    // forward declaration
    HPP_PREDEF_CLASS (OrientedPathOptimizer);
    // Planner objects are manipulated only via shared pointers
    typedef boost::shared_ptr <OrientedPathOptimizer> OrientedPathOptimizerPtr_t;


    class OrientedPathOptimizer : public core::PathOptimizer
    {
    public:
      /// Return shared pointer to new object.
      static OrientedPathOptimizerPtr_t create (const core::Problem& problem);

      /// Optimize path
      virtual core::PathVectorPtr_t optimize (const core::PathVectorPtr_t& path);
    protected:
      OrientedPathOptimizer (const core::Problem& problem);

    private:
      const RbPrmPathValidationPtr_t rbprmPathValidation_;



    };//class oriented-path-optimizer
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_RBPRM_ORIENTED_PATH_OPTIMIZER_HH
