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


    OrientedPathOptimizerPtr_t
    OrientedPathOptimizer::create (const Problem& problem)
    {
      OrientedPathOptimizer* ptr = new OrientedPathOptimizer (problem);
      return OrientedPathOptimizerPtr_t (ptr);
    }

    OrientedPathOptimizer::OrientedPathOptimizer (const Problem& problem) :
      PathOptimizer (problem),
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


    PathVectorPtr_t OrientedPathOptimizer::optimize (const PathVectorPtr_t& path)
    {
      hppDout(notice,"!! Start optimize()");
      using std::numeric_limits;
      using std::make_pair;
      PathPtr_t unusedValidPart;
      core::PathValidationReportPtr_t unusedReport;
      PathVectorPtr_t result = PathVector::create (path->outputSize (),path->outputDerivativeSize ());
      const size_t numPaths = path->numberPaths ();
      bool orientedValid[numPaths];
      bool replaceValid[numPaths];
      PathPtr_t orientedPaths[numPaths];
      PathPtr_t resultPaths[numPaths];
      KinodynamicPathPtr_t castedPath;
      // iterate over all elements of path (all kinodynamicPath), convert them to orientedPath and test collision
      for (std::size_t i=0; i<numPaths; ++i) {
        orientedValid[i] = false;
        const PathPtr_t& element (path->pathAtRank (i));
        castedPath = boost::dynamic_pointer_cast<KinodynamicPath>(element);
        if(castedPath){
          orientedPaths[i] = core::KinodynamicOrientedPath::createCopy(castedPath);
          if(orientedPaths[i]){
            orientedValid[i] = rbprmPathValidation_->validate(orientedPaths[i], false, unusedValidPart, unusedReport);
          }
        }
        resultPaths[i] = element;
      }
      // then test if previous/next path can be adjusted to the new orientation
      for (size_t i=0;i<numPaths;i++){
        replaceValid[i]=false;
      }
      hppDout(notice,"Number of paths : "<<numPaths);
      std::ostringstream oss;
      for(size_t i = 0 ; i < numPaths ; i++)
        oss<<orientedValid[i]<<" ; ";
      hppDout(notice,"orientedValid : "<<oss.str());

      for (size_t i=0;i<numPaths;i++){
        if(!replaceValid[i])
          replaceValid[i] = checkReplaceOrientation(i,numPaths,replaceValid,orientedValid,orientedPaths,resultPaths);
      }

      std::ostringstream oss2;
      for(size_t i = 0 ; i < numPaths ; i++)
        oss2<<replaceValid[i]<<" ; ";
      hppDout(notice,"replaceValid : "<<oss2.str());
      // concatenate resultPath in a pathVector :
      //TODO
      for(size_t i=0;i<numPaths;i++){
        result->appendPath(resultPaths[i]);
      }
      return result;
    }


    /**
     * @brief OrientedPathOptimizer::checkReplaceOrientation recursively check if an element of the path vector can be changed to an
     * oriented path : the final state of the previous element and the initial state of the next element must be changed
     * and this changes must be valid
     * @param index
     * @param replaceValid
     * @param orientedValid
     * @param orientedPaths
     * @param resultPaths
     * @return
     */
    bool OrientedPathOptimizer::checkReplaceOrientation(const size_t index,const size_t lastIndex, bool* replaceValid, bool* orientedValid, PathPtr_t* orientedPaths, PathPtr_t* resultPaths){
      PathPtr_t previousPath,nextPath;
      bool previousValid(false);
      PathPtr_t unusedValidPart;
      core::PathValidationReportPtr_t unusedReport;

      if(!orientedValid[index])
        return false;

      if(index>0){          // test if previous element can be adjusted:
        if(orientedValid[index-1]&&replaceValid[index-1]){
          previousValid=true;
        }else{
          previousPath = steer(resultPaths[index-1]->initial(),orientedPaths[index]->initial());
          if(previousPath)
            previousValid=rbprmPathValidation_->validate(previousPath, false, unusedValidPart, unusedReport);
        }
      }else{ // if first element of the pathvector : always valid
        previousValid = true;
      }

      if(!previousValid)
        return false;

      if(index<lastIndex-1){ // test if next element can be adjusted
        if(orientedValid[index+1]){ // make a recursive test for the next elements
          replaceValid[index] = checkReplaceOrientation(index+1,lastIndex,replaceValid,orientedValid,orientedPaths,resultPaths);
          nextPath = orientedPaths[index+1];
        }else{
          nextPath = steer(orientedPaths[index]->initial(),resultPaths[index+1]->end());
          if(nextPath)
            replaceValid[index]=rbprmPathValidation_->validate(nextPath, false, unusedValidPart, unusedReport);
        }
      }else{ // if last element of the pathVector : always valid
        replaceValid[index] = true;
      }


      if(replaceValid[index]){ // replace values in resultPaths
        resultPaths[index]=orientedPaths[index];
        resultPaths[index-1]=previousPath;
        resultPaths[index+1]=nextPath;
      }


      return replaceValid[index];
    }

    core::PathPtr_t OrientedPathOptimizer::steer (ConfigurationIn_t q1,
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
        if (problem().pathProjector()->apply (dp, pp))
          return pp;
      }
      return PathPtr_t ();
    }



  }//rbprm
}//hpp
