//
// Copyright (c) 2017 CNRS
// Authors: Fernbach Pierre
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

#include <hpp/rbprm/dynamic/dynamic-path-validation.hh>
#include <hpp/core/path.hh>
#include <hpp/rbprm/rbprm-validation.hh>
#include <hpp/core/config-validation.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/validation-report.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/util/debug.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/util/timer.hh>

namespace hpp {
  namespace rbprm {
    using core::value_type;
    using core::Configuration_t;




    DynamicPathValidationPtr_t DynamicPathValidation::create (const core::DevicePtr_t& robot, const core::value_type& stepSize)
    {
      DynamicPathValidation* ptr (new DynamicPathValidation(robot, stepSize));
      DynamicPathValidationPtr_t shPtr (ptr);
      return shPtr;
    }


    DynamicPathValidation::DynamicPathValidation(const core::DevicePtr_t &robot, const core::value_type &stepSize) :
      RbPrmPathValidation(robot,stepSize)
    {}



    /// validate with custom filter for the rom validation
    bool DynamicPathValidation::validate (const core::PathPtr_t& path, bool reverse, core::PathPtr_t& validPart, core::PathValidationReportPtr_t& validationReport,const std::vector<std::string>& filter){
      hppDout(notice,"dynamic path validation called with filters");
      hppStartBenchmark(PATH_VALIDATION);
      core::ValidationReportPtr_t configReport;
      Configuration_t q;
      if(reverse)
        (*path)(q,path->timeRange ().second);
      else
        (*path)(q,path->timeRange ().first);

      rbprmValidation_->validate(q,configReport);
      dynamicValidation_->setInitialReport(configReport);
      hppDout(notice,"dynamic validation set initial report OK");

      assert (path);
      bool valid = true;
      if (reverse) {
        value_type tmin = path->timeRange ().first;
        value_type tmax = path->timeRange ().second;
        value_type lastValidTime = tmax;
        value_type t = tmax;
        unsigned finished = 0;
        Configuration_t q (path->outputSize());
        while (finished < 2 && valid) {
          bool success = (*path) (q, t);
          if (!success || !rbprmValidation_->validate (q, configReport,filter) || !dynamicValidation_->validate(q,configReport)) {
            validationReport = core::CollisionPathValidationReportPtr_t
                (new core::CollisionPathValidationReport (t, configReport));
            valid = false;
          } else {
            lastValidTime = t;
            t -= stepSize_;
          }
          if (t < tmin) {
            t = tmin;
            finished++;
          }
        }
        if (valid) {
          validPart = path;
          hppStopBenchmark(PATH_VALIDATION);
          hppDisplayBenchmark(PATH_VALIDATION);
          return true;
        } else {
          validPart = path->extract (std::make_pair (lastValidTime, tmax));
          hppStopBenchmark(PATH_VALIDATION);
          hppDisplayBenchmark(PATH_VALIDATION);
          return false;
        }
      } else {
        value_type tmin = path->timeRange ().first;
        value_type tmax = path->timeRange ().second;
        value_type lastValidTime = tmin;
        value_type t = tmin;
        unsigned finished = 0;
        Configuration_t q (path->outputSize());
        while (finished < 2 && valid) {
          bool success = (*path) (q, t);
          if (!success || !rbprmValidation_->validate (q, configReport,filter) || !dynamicValidation_->validate(q,configReport)) {
            validationReport = core::CollisionPathValidationReportPtr_t
                (new core::CollisionPathValidationReport (t, configReport));
            valid = false;
          } else {
            lastValidTime = t;
            t += stepSize_;
          }
          if (t > tmax) {
            t = tmax;
            finished ++;
          }
        }
        if (valid) {
          validPart = path;
          hppStopBenchmark(PATH_VALIDATION);
          hppDisplayBenchmark(PATH_VALIDATION);
          return true;
        } else {
          validPart = path->extract (std::make_pair (tmin, lastValidTime));
          hppStopBenchmark(PATH_VALIDATION);
          hppDisplayBenchmark(PATH_VALIDATION);
          return false;
        }
      }
      hppStopBenchmark(PATH_VALIDATION);
      hppDisplayBenchmark(PATH_VALIDATION);
    }

    bool DynamicPathValidation::validate (const core::PathPtr_t& path, bool reverse,  core::PathPtr_t& validPart,  core::PathValidationReportPtr_t& validationReport){
      hppDout(info,"dynamic path validation called");
      hppDout(info,"path begin : "<<path->timeRange ().first);
      hppDout(info,"path end : "<<path->timeRange ().second);
      hppStartBenchmark(PATH_VALIDATION);
      core::ValidationReportPtr_t configReport;
      Configuration_t q (path->outputSize());
      if(reverse)
        (*path)(q,path->timeRange ().second);
      else
        (*path)(q,path->timeRange ().first);

      hppDout(info,"q = "<<pinocchio::displayConfig(q));
      rbprmValidation_->validate(q,configReport);


      hppDout(info,"rbprmValidation called" );
      dynamicValidation_->setInitialReport(configReport);
      hppDout(info,"dynamic validation set initial report OK");
      bool valid = core::pathValidation::Discretized::validate(path,reverse,validPart,validationReport);
      hppStopBenchmark(PATH_VALIDATION);
      hppDisplayBenchmark(PATH_VALIDATION);
      return valid;
    }




  } // namespace rbprm
} // namespace hpp
