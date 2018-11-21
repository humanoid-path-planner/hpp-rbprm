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

# include <hpp/rbprm/dynamic/dynamic-validation.hh>
# include <hpp/util/debug.hh>
# include <hpp/rbprm/planner/rbprm-node.hh>
#include <hpp/util/timer.hh>
#include <hpp/pinocchio/configuration.hh>

namespace hpp {
  namespace rbprm {

    DynamicValidationPtr_t DynamicValidation::create
    (bool rectangularContact, double sizeFootX, double sizeFootY, double mass, double mu)
    {
      DynamicValidation* ptr = new DynamicValidation (rectangularContact,sizeFootX,sizeFootY,mass,mu);
      return DynamicValidationPtr_t (ptr);
    }



    bool DynamicValidation::validate (const core::Configuration_t& config, core::ValidationReportPtr_t& validationReport){
      hppDout(notice,"Begin dynamic validation");
     // hppStartBenchmark(DYNAMIC_VALIDATION);
      // test if the same number of ROM are in collision :
      core::RbprmValidationReportPtr_t rbReport = boost::dynamic_pointer_cast<core::RbprmValidationReport> (validationReport);
      if(!rbReport){
        hppDout(error,"error while casting the report");
       // hppStopBenchmark(DYNAMIC_VALIDATION);
        //hppDisplayBenchmark(DYNAMIC_VALIDATION);
        return false;
      }
      if(lastReport_->ROMReports.size() != rbReport->ROMReports.size()){
        //hppDout(notice,"dynamic validation : rom report not the same size");
       // hppStopBenchmark(DYNAMIC_VALIDATION);
       // hppDisplayBenchmark(DYNAMIC_VALIDATION);
        return false;
      }else{
       // hppDout(notice,"dynamic validation : rom report have the same size");
      }
      bool sameContacts(true);

      for(std::map<std::string,core::CollisionValidationReportPtr_t>::const_iterator it = rbReport->ROMReports.begin() ; it != rbReport->ROMReports.end() ; ++it){
        if(lastReport_->ROMReports.find(it->first) != lastReport_->ROMReports.end()){ // test if the same rom was in collision in init report
         // hppDout(notice,"rom "<<it->first<<" is in both reports");
          if(lastReport_->ROMReports.at(it->first)->object2 != it->second->object2){
            //hppDout(notice,"detect contact change for rom : "<<it->first);
            sameContacts=false;
            break;
          }else{
           // hppDout(notice,"rom : "<<it->first<< " have the same contacts in both report");
          }
        }
      }
       // if !sameContact, compute new contacts infos and test acceleration
      if(sameContacts && initContacts_){
       hppDout(notice,"initial contacts still active");
       // hppStopBenchmark(DYNAMIC_VALIDATION);
       // hppDisplayBenchmark(DYNAMIC_VALIDATION);
        return true;
      }
      size_t configSize = config.size();

      if(sameContacts){ // new contacts already computed
        if(config.segment<3>(configSize-3) == lastAcc_){
            hppDout(notice,"this acceleration is already verified");
        //  hppStopBenchmark(DYNAMIC_VALIDATION);
        //  hppDisplayBenchmark(DYNAMIC_VALIDATION);
          return true;
        }else{ // new acceleration, check if valid
          lastAcc_ = config.segment<3>(configSize-3);
          bool aValid = sEq_->checkAdmissibleAcceleration(H_,h_,lastAcc_);
            hppDout(notice,"new acceleration : "<<lastAcc_.transpose()<<", valid = "<<aValid);
        //  hppStopBenchmark(DYNAMIC_VALIDATION);
        //  hppDisplayBenchmark(DYNAMIC_VALIDATION);
          return aValid;
        }
      }else{ // changes in contacts, recompute the matrices and check the acceleration :
        initContacts_ = false;
        hppDout(notice,"new contacts ! for config = "<<pinocchio::displayConfig(config));
        lastAcc_ = config.segment<3>(configSize-3);
        lastReport_=rbReport;
        core::ConfigurationPtr_t q = core::ConfigurationPtr_t (new core::Configuration_t(config));
        core::RbprmNode node(q);
        node.fillNodeMatrices(rbReport,rectangularContact_,sizeFootX_,sizeFootY_,mass_,mu_);
        sEq_->setG(node.getG());
        h_=node.geth();
        H_=node.getH();
       // hppDout(notice,"fill matrices OK, test acceleration : ");

        // test the acceleration
        bool aValid = sEq_->checkAdmissibleAcceleration(H_,h_,lastAcc_);
          hppDout(notice,"new acceleration : "<<lastAcc_.transpose()<<", valid = "<<aValid);
       // hppStopBenchmark(DYNAMIC_VALIDATION);
      //  hppDisplayBenchmark(DYNAMIC_VALIDATION);
        return aValid;
      }

    }



    void DynamicValidation::setInitialReport(core::ValidationReportPtr_t initialReport){
      core::RbprmValidationReportPtr_t rbReport = boost::dynamic_pointer_cast<core::RbprmValidationReport> (initialReport);
      if(rbReport){
        lastReport_ = rbReport;
        initContacts_=true;
      }
      else
        hppDout(error,"Error while casting rbprmReport");
    }




    DynamicValidation::DynamicValidation (bool rectangularContact, double sizeFootX, double sizeFootY, double mass, double mu) :
      rectangularContact_(rectangularContact),sizeFootX_(sizeFootX),sizeFootY_(sizeFootY),mass_(mass),mu_(mu),
      sEq_(new centroidal_dynamics::Equilibrium("dynamic_val", mass,4,centroidal_dynamics::SOLVER_LP_QPOASES,true,10,false))
    {
      hppDout(info,"Dynamic validation created with attribut : rectangular contact = "<<rectangularContact<<" size foot : "<<sizeFootX);
      hppDout(info,"mass = "<<mass<<"  mu = "<<mu);
    }


  }//rbprm
}//hpp
