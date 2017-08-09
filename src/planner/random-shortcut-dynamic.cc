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


#include <hpp/rbprm/planner/random-shortcut-dynamic.hh>
#include <limits>
#include <deque>
#include <cstdlib>
#include <hpp/util/assertion.hh>
#include <hpp/util/debug.hh>
#include <hpp/core/distance.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/kinodynamic-oriented-path.hh>
#include <hpp/rbprm/planner/rbprm-node.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/config-validations.hh>

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


    RandomShortcutDynamicPtr_t
    RandomShortcutDynamic::create (const Problem& problem)
    {
      RandomShortcutDynamic* ptr = new RandomShortcutDynamic (problem);
      return RandomShortcutDynamicPtr_t (ptr);
    }

    RandomShortcutDynamic::RandomShortcutDynamic (const Problem& problem) :
      RandomShortcut (problem),
      sm_(boost::dynamic_pointer_cast<SteeringMethodKinodynamic>(problem.steeringMethod())),
      rbprmPathValidation_(boost::dynamic_pointer_cast<RbPrmPathValidation>(problem.pathValidation()))
    {
      assert(sm_ && "Random-shortcut-dynamic must use a kinodynamic-steering-method");
      assert(rbprmPathValidation_ && "Path validation should be a RbPrmPathValidation class for this solver");

      // retrieve parameters from problem :
      try {
        boost::any value_x = problem.get<boost::any> (std::string("sizeFootX"));
        boost::any value_y = problem.get<boost::any> (std::string("sizeFootY"));
        sizeFootX_ = boost::any_cast<double>(value_x)/2.;
        sizeFootY_ = boost::any_cast<double>(value_y)/2.;
        rectangularContact_ = 1;
      } catch (const std::exception& e) {
        hppDout(warning,"Warning : size of foot not definied, use 0 (contact point)");
        sizeFootX_ =0;
        sizeFootY_ =0;
        rectangularContact_ = 0;
      }
      try {
        boost::any value = problem.get<boost::any> (std::string("tryJump"));
        tryJump_ = boost::any_cast<bool>(value);
      } catch (const std::exception& e) {
        tryJump_=false;
      }
      hppDout(notice,"tryJump in random shortcut = "<<tryJump_);

      try {
        boost::any value = problem.get<boost::any> (std::string("friction"));
        mu_ = boost::any_cast<double>(value);
        hppDout(notice,"mu define in python : "<<mu_);
      } catch (const std::exception& e) {
        mu_= 0.5;
        hppDout(notice,"mu not defined, take : "<<mu_<<" as default.");
      }
    }


    // Compute the length of a vector of paths assuming that each element
    // is optimal for the given distance.
    template <bool reEstimateLength = false> struct PathLength {
      static inline value_type run (const PathVectorPtr_t& path,
                                    const DistancePtr_t& /*distance*/)
      {
        if (reEstimateLength) return path->length ();
        else {
          value_type result = 0;
          for (std::size_t i=0; i<path->numberPaths (); ++i) {
            const PathPtr_t& element (path->pathAtRank (i));
            result += element->length();
          }
          return result;
        }
      }
    };




    PathVectorPtr_t RandomShortcutDynamic::optimize (const PathVectorPtr_t& path)
    {
      hppDout(notice,"!! Start optimize()");
      using std::numeric_limits;
      using std::make_pair;
      bool finished = false;
      value_type t[4];
      Configuration_t q[4];
      q[0] = path->initial ();
      q[3] = path->end ();
      PathVectorPtr_t tmpPath = path;

      // Maximal number of iterations without improvements
      std::size_t n = problem().getParameter<std::size_t>("PathOptimizersNumberOfLoops", 5);
      n = 100;
      std::cout<<" number of loops : "<<n<<std::endl;
      std::size_t projectionError = n;
      std::deque <value_type> length (n-1,
                                      numeric_limits <value_type>::infinity ());
      length.push_back (PathLength<>::run (tmpPath, problem ().distance ()));
      PathVectorPtr_t result;
      Configuration_t q1 (path->outputSize ()),
          q2 (path->outputSize ());

      q[1] = q1;
      q[2] = q2;
      while (!finished && projectionError != 0) {
        t[0] = tmpPath->timeRange ().first;
        t[3] = tmpPath->timeRange ().second;
        value_type u2 = t[0] + (t[3] -t[0]) * rand ()/RAND_MAX;
        value_type u1 = t[0] + (t[3] -t[0]) * rand ()/RAND_MAX;
        if (u1 < u2) {t[1] = u1; t[2] = u2;} else {t[1] = u2; t[2] = u1;}
        if (!(*tmpPath) (q[1], t[1])) {
          hppDout (error, "Configuration at param " << t[1] << " could not be "
                                                               "projected");
          projectionError--;
          continue;
        }
        if (!(*tmpPath) (q[2], t[2])) {
          hppDout (error, "Configuration at param " << t[2] << " could not be "
                                                               "projected");
          projectionError--;
          continue;
        }
        // Validate sub parts
        bool valid [3];
        PathPtr_t straight [3];
        core::PathValidationReportPtr_t report;
        PathPtr_t validPart;

        for (unsigned i=0; i<3; ++i) {
          straight [i] = steer (q[i], q[i+1]);
          if (!straight [i]) valid[i] = false;
          else { // with kinodynamic path, we are not assured that a 'straight line' is shorter than the previously found path
            valid[i] = (straight[i]->length() < PathLength<true>::run (tmpPath->extract(make_pair <value_type, value_type> (t[i], t[i+1]))->as <PathVector> (), problem ().distance ()));
            if(valid[i])
              valid[i] = problem ().pathValidation ()->validate(straight [i], false, validPart, report);
          }
        }
        hppDout(notice,"t0 = "<<t[0]<<" ; t1 = "<<t[1]<<" ; t2 = "<<t[2]<<" ; t3 = "<<t[3]);
        hppDout(notice,"first segment : valid : "<<valid[0]);
        hppDout(notice,"mid segment   : valid : "<<valid[1]);
        hppDout(notice,"last segment  : valid : "<<valid[2]);

        // Replace valid parts
        result = PathVector::create (path->outputSize (),
                                     path->outputDerivativeSize ());

        for (unsigned i=0; i<3; ++i) {
          try {
            if (valid [i])
              result->appendPath (straight[i]);
            else
              result->concatenate (tmpPath->extract(make_pair <value_type, value_type> (t[i], t[i+1]))->as <PathVector> ());
          } catch (const core::projection_error& e) {
            hppDout (error, "Caught exception at with time " << t[i] << " and " <<
                                                                        t[i+1] << ": " << e.what ());
            projectionError--;
            result = tmpPath;
            continue;
          }
        }

        length.push_back (PathLength<>::run (result, problem ().distance ()));
        length.pop_front ();
        finished = (length [0] - length [n-1]) <= 1e-4 * length[n-1];
        hppDout (info, "length = " << length [n-1]);
        tmpPath = result;
      }
      for (std::size_t i = 0; i < result->numberPaths (); ++i) {
        if (result->pathAtRank(i)->constraints())
          hppDout (info, "At rank " << i << ", constraints are " <<
                   *result->pathAtRank(i)->constraints());
        else
          hppDout (info, "At rank " << i << ", no constraints");
      }
      return result;
    } // optimize


    PathPtr_t RandomShortcutDynamic::steer (ConfigurationIn_t q1,
                                            ConfigurationIn_t q2) const
    {
      // according to optimize() method : the path is always in the direction q1 -> q2
      // first : create a node and fill all informations about contacts for the initial state (q1):
      core::RbprmNodePtr_t x1(new core::RbprmNode (ConfigurationPtr_t (new Configuration_t(q1))));
      core::ValidationReportPtr_t report;
      rbprmPathValidation_->getValidator()->computeAllContacts(true);
      problem().configValidations()->validate(q1,report);
      rbprmPathValidation_->getValidator()->computeAllContacts(false);
      hppDout(notice,"Random shortucut, fillNodeMatrices : ");
      x1->fillNodeMatrices(report,rectangularContact_,sizeFootX_,sizeFootY_,problem().robot()->mass(),mu_);

      // call steering method kinodynamic with the newly created node
      hppDout(notice,"Random shortucut, steering method  : ");
      PathPtr_t dp = (*sm_)(x1,q2);
      if (dp) {
        hppDout(notice,"Random shortucut, path exist ");
        if((dp->initial() != q1)  || (dp->end() != q2)){
          hppDout(notice,"Path doesn't link the targets");
          return PathPtr_t ();
        }
        if(dp->length() <= 0.001){
          hppDout(notice,"Path length < epsilon");
          return PathPtr_t ();
        }
        if (!problem().pathProjector()) return dp;
        PathPtr_t pp;
        if (problem().pathProjector()->apply (dp, pp)){
          hppDout(notice,"Path projector exist in random-shortcut");
          return pp;
        }
      }
      return PathPtr_t ();
    }










  } // rbprm
}//hpp

