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

#include <hpp/rbprm/planner/oriented-path-optimizer.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/kinodynamic-oriented-path.hh>
#include <limits>
#include <deque>
#include <cstdlib>
#include <hpp/util/assertion.hh>
#include <hpp/util/debug.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/path-projector.hh>


namespace hpp{
  namespace rbprm{
    using core::PathVector;
    using core::PathVectorPtr_t;
    using core::PathPtr_t;
    using core::Problem;
    using core::ConfigurationIn_t;
    using core::ConfigurationPtr_t;
    using core::Configuration_t;
    using model::value_type;
    using core::DistancePtr_t;
    using core::KinodynamicPathPtr_t;
    using core::KinodynamicPath;
    using core::KinodynamicOrientedPathPtr_t;
    using core::KinodynamicOrientedPath;


    OrientedPathOptimizerPtr_t
    OrientedPathOptimizer::create (const Problem& problem)
    {
      OrientedPathOptimizer* ptr = new OrientedPathOptimizer (problem);
      return OrientedPathOptimizerPtr_t (ptr);
    }

    OrientedPathOptimizer::OrientedPathOptimizer (const Problem& problem) :
      PathOptimizer (problem),
      rbprmPathValidation_(boost::dynamic_pointer_cast<RbPrmPathValidation>(problem.pathValidation()))
    {
      assert(rbprmPathValidation_ && "Path validation should be a RbPrmPathValidation class for this solver");

    }


    PathVectorPtr_t OrientedPathOptimizer::optimize (const PathVectorPtr_t& path)
    {
      hppDout(notice,"!! Start optimize()");
      using std::numeric_limits;
      using std::make_pair;
      PathVectorPtr_t result = PathVector::create (path->outputSize (),path->outputDerivativeSize ());

      // TODO : iterate over all elements of path (all kinodynamicPath), convert them to orientedPath and test collision
      // TODO : then test if previous/next path can be adjusted to the new orientation
      return result;
    }


  }//rbprm
}//hpp
