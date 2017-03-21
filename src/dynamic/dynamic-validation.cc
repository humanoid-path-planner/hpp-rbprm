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

#include <hpp/rbprm/dynamic/dynamic-validation.hh>
#include <hpp/util/debug.hh>

namespace hpp {
  namespace rbprm {

    DynamicValidationPtr_t DynamicValidation::create
    (bool rectangularContact, double sizeFootX, double sizeFootY, double mass, double mu)
    {
      DynamicValidation* ptr = new DynamicValidation (rectangularContact,sizeFootX,sizeFootY,mass,mu);
      return DynamicValidationPtr_t (ptr);
    }

    bool DynamicValidation::validate (const core::Configuration_t& /*config*/, core::ValidationReportPtr_t& /*report*/){
      hppDout(warning,"Dynamic validation called without rbprm reports");
      return true;
    }


    bool DynamicValidation::validate (const core::Configuration_t& config, core::ValidationReportPtr_t& inputReport, core::ValidationReportPtr_t& outputReport){
      hppDout(notice,"Begin dynamic validation");
      // test if the same number of ROM are in collision :
      core::RbprmValidationReportPtr_t rbReport = boost::dynamic_pointer_cast<core::RbprmValidationReport> (inputReport);
      if(initialReport_->ROMReports.size() != rbReport->ROMReports.size()){
        hppDout(notice,"dynamic validation : rom report not the same size");
        return false;
      }else{
        hppDout(notice,"dynamic validation : rom report have the same size");
      }
      bool sameContacts(true);
      for(std::map<std::string,core::CollisionValidationReportPtr_t>::const_iterator it = rbReport->ROMReports.begin() ; it != rbReport->ROMReports.end() ; ++it){
        if(initialReport_->ROMReports.find(it->first) != initialReport_->ROMReports.end()){ // test if the same rom was in collision in init report
          hppDout(notice,"rom "<<it->first<<" is in both reports");
          if(initialReport_->ROMReports.at(it->first)->object2 != it->second->object2){
            hppDout(notice,"detect contact change for rom : "<<it->first);
            sameContacts=false;
            break;
          }else{
            hppDout(notice,"rom : "<<it->first<< " have the same contacts in both report");
          }
        }
      }


      return sameContacts;
      //TODO : if !sameContact, compute new contacts infos and test acceleration
    }



    void DynamicValidation::setInitialReport(core::ValidationReportPtr_t initialReport){
      core::RbprmValidationReportPtr_t rbReport = boost::dynamic_pointer_cast<core::RbprmValidationReport> (initialReport);
      if(rbReport)
        initialReport_ = rbReport;
      else
        hppDout(error,"Error while casting rbprmReport");
    }




    DynamicValidation::DynamicValidation (bool rectangularContact, double sizeFootX, double sizeFootY, double mass, double mu) :
      rectangularContact_(rectangularContact),sizeFootX_(sizeFootX),sizeFootY_(sizeFootY),mass_(mass),mu_(mu),
      sEq_(new robust_equilibrium::StaticEquilibrium("dynamic_val", mass,4,robust_equilibrium::SOLVER_LP_QPOASES,true,10,false))
    {
    }


  }//rbprm
}//hpp
