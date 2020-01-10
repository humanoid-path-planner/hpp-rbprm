// Copyright (c) 2014, LAAS-CNRS
// Authors: Steve Tonneau (steve.tonneau@laas.fr)
//
// This file is part of hpp-rbprm.
// hpp-rbprm is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-rbprm is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-rbprm. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/rbprm/rbprm-validation.hh>
#include <hpp/core/collision-validation.hh>
#include <hpp/core/joint-bound-validation.hh>
#include <hpp/rbprm/rbprm-validation-report.hh>


namespace
{
  hpp::core::CollisionValidationPtr_t tuneFclValidation(const hpp::pinocchio::RbPrmDevicePtr_t& robot)
  {
    typedef hpp::core::ObstacleUser::CollisionRequests_t CollisionRequests_t;
    hpp::core::CollisionValidationPtr_t validation = hpp::core::CollisionValidation::create(robot);
    // enable contact for all collision pairs
    CollisionRequests_t& req (validation->requests ());
    for (CollisionRequests_t::iterator it (req.begin ()); it != req.end ();
         ++it) {
      it->enable_contact = true;
    }
    validation->defaultRequest().enable_contact = true;
    return validation;
  }

  hpp::rbprm::T_RomValidation createRomValidations(const hpp::pinocchio::RbPrmDevicePtr_t& robot,
                                                   const std::map<std::string, std::vector<std::string> >& affFilters)
  {
    hpp::rbprm::T_RomValidation result;
    hppDout(notice,"Number of ROM : "<<robot->robotRoms_.size());
    for(hpp::pinocchio::T_Rom::const_iterator cit = robot->robotRoms_.begin(); cit != robot->robotRoms_.end(); ++cit)
    {
      hppDout(notice,"name = "<<cit->first);
      std::map<std::string, std::vector<std::string> >::const_iterator cfit = affFilters.find(cit->first);
      if(cfit != affFilters.end())
      {
        result.insert(std::make_pair(cit->first, hpp::rbprm::RbPrmRomValidation::create(cit->second, cfit->second)));
      }
      else
      { // no affordance filter defined for this ROM, assume it can collide with all the objects
        result.insert(std::make_pair(cit->first, hpp::rbprm::RbPrmRomValidation::create(cit->second)));
      }
    }
    return result;
  }
}

namespace hpp {
  using namespace core;
  namespace rbprm {

    RbPrmValidationPtr_t RbPrmValidation::create
    (const pinocchio::RbPrmDevicePtr_t& robot, const std::vector<std::string>& filter,
     const std::map<std::string, std::vector<std::string> >& affFilters,
     const hpp::rbprm::affMap_t &affordances,
     const core::ObjectStdVector_t& geometries)
    {
      RbPrmValidation* ptr = new RbPrmValidation (robot, filter, affFilters,
                                                  affordances, geometries);
      return RbPrmValidationPtr_t (ptr);
    }

    RbPrmValidation::RbPrmValidation (const pinocchio::RbPrmDevicePtr_t& robot
                                      , const std::vector<std::string>& filter,
                                      const std::map<std::string,
                                      std::vector<std::string> >& affFilters,
                                      const hpp::rbprm::affMap_t &affordances,
                                      const hpp::core::ObjectStdVector_t &geometries)
      : CollisionValidation(robot)
      , trunkValidation_(tuneFclValidation(robot))
      , boundValidation_(core::JointBoundValidation::create(robot))
      , romValidations_(createRomValidations(robot, affFilters))
      , unusedReport_(new CollisionValidationReport)
    {
      for(std::vector<std::string>::const_iterator cit = filter.begin();
          cit != filter.end(); ++cit)
      {
        if(romValidations_.find(*cit) == romValidations_.end())
        {
          std::cout << "warning: default filter impossible to match in rbprmshooter" << std::endl;
        }
      }

      for(hpp::core::ObjectStdVector_t::const_iterator cit = geometries.begin();
          cit != geometries.end(); ++cit)
      {
        addObstacle(*cit);
      }
      // Add obstacles corresponding to affordances of given rom
      hppDout(notice,"add obstacle, filter size = "<<filter.size());
      for(hpp::pinocchio::T_Rom::const_iterator cit = robot->robotRoms_.begin(); cit != robot->robotRoms_.end(); ++cit)
      {
        std::map<std::string, std::vector<std::string> >::const_iterator affFilterIt = affFilters.find(cit->first);
        // Check if all rom filters have been given an affordance filter.
        // If not, an error is thrown.
        if (affFilterIt == affFilters.end ()) {
          hppDout(notice,"Filter " + cit->first + " has no affordance filter settings! Please add them to continue.");
        }else{
          defaultFilter_.push_back(cit->first);

          T_RomValidation::const_iterator romIt = romValidations_.find (affFilterIt->first);
          if (romIt == romValidations_.end ()) {
            // TODO: Throw error?
            std::runtime_error ("No romValidator object found for filter " + affFilterIt->first + "!");
          }
          for (unsigned int fIdx = 0; fIdx < affFilterIt->second.size (); fIdx++)
          {
              if (!affordances.has(std::string (affFilterIt->second[fIdx])))
              {
                  std::cout << "Filter " << affFilterIt->first << " has invalid affordance filter setting "
                            << affFilterIt->second[fIdx] << ". Ignoring such filter setting." << std::endl;
              }
              else
              {
                  const hpp::core::AffordanceObjects_t& affObjs = affordances.get(affFilterIt->second[fIdx]);
                  for (std::size_t affIdx = 0; affIdx < affObjs.size (); affIdx++)
                  {
                      romIt->second->addObstacle(affObjs[affIdx].second);
                  }
              }
          }
          if(std::find(filter.begin(), filter.end(), romIt->first) == filter.end()){
            romIt->second->setOptional(true);
          }
        }
      }

    }


    bool RbPrmValidation::validateRoms(const core::Configuration_t& config,
                                       const std::vector<std::string>& filter, RbprmValidationReportPtr_t& rbprmReport)

    {
      ValidationReportPtr_t rbprmReportCast =  rbprmReport;
      unsigned int filterMatch(0);
      for(T_RomValidation::const_iterator cit = romValidations_.begin();
          cit != romValidations_.end() && (filterMatch < 1 || filterMatch < filter.size()); ++cit)
      {
        if((filter.empty() || std::find(filter.begin(), filter.end(), cit->first) != filter.end())
           && cit->second->validate(config, rbprmReportCast))
        {
          ++filterMatch;
        }
      }

      rbprmReport->romsValid = filterMatch >= filter.size();
      return rbprmReport->romsValid;
    }

    bool RbPrmValidation::validateRoms(const core::Configuration_t& config, hpp::core::RbprmValidationReportPtr_t &validationReport)
    {
      return validateRoms(config,defaultFilter_,validationReport);
    }

    bool RbPrmValidation::validateRoms(const core::Configuration_t& config,const std::vector<std::string>& filter)
    {
      RbprmValidationReportPtr_t unusedReport;
      return validateRoms(config,filter,unusedReport);
    }

    bool RbPrmValidation::validateRoms(const core::Configuration_t& config)
    {
      RbprmValidationReportPtr_t unusedReport;
      return validateRoms(config,defaultFilter_,unusedReport);
    }


    bool RbPrmValidation::validate (const Configuration_t& config)
    {
      return validate(config,unusedReport_, defaultFilter_);
    }

    bool RbPrmValidation::validate (const Configuration_t& config,
                                    ValidationReportPtr_t& validationReport)
    {
        return validate(config, validationReport, defaultFilter_);
    }



    bool RbPrmValidation::validate (const Configuration_t& config,
                                    const std::vector<std::string> &filter)
    {
      return validate(config, unusedReport_, filter);
    }

    bool RbPrmValidation::validate (const Configuration_t& config,
                                    hpp::core::ValidationReportPtr_t &validationReport,
                                    const std::vector<std::string>& filter)
    {
        CollisionValidationReportPtr_t colReport = boost::dynamic_pointer_cast<CollisionValidationReport>(validationReport);
        if (colReport)
            colReport->result.clear();

        RbprmValidationReportPtr_t rbprmReport(new RbprmValidationReport);

        bool success = trunkValidation_->validate(config, validationReport);
        if(success){
            rbprmReport->trunkInCollision=false;
        }else{
            colReport = boost::dynamic_pointer_cast<CollisionValidationReport>(validationReport);
            rbprmReport->object1 = colReport->object1;
            rbprmReport->object2 = colReport->object2;
            rbprmReport->result = colReport->result;
            rbprmReport->trunkInCollision = true;
        }
      // always compute full report
      success = validateRoms(config, filter,rbprmReport) && success && boundValidation_->validate(config,validationReport);
      validationReport = rbprmReport;
      return success;
    }

    bool RbPrmValidation::validateTrunk(const Configuration_t& config,
                                           hpp::core::ValidationReportPtr_t &validationReport)
    {
      return trunkValidation_->validate(config, validationReport);// && boundValidation_->validate(config,validationReport);
    }


    void RbPrmValidation::addObstacle (const CollisionObjectConstPtr_t& object)
    {
      trunkValidation_->addObstacle(object);
    }

    void RbPrmValidation::removeObstacleFromJoint
    (const JointPtr_t& joint, const CollisionObjectPtr_t& obstacle)
    {
      trunkValidation_->removeObstacleFromJoint(joint, obstacle);
      for(T_RomValidation::const_iterator cit = romValidations_.begin();
          cit != romValidations_.end(); ++cit)
      {
        cit->second->removeObstacleFromJoint(joint, obstacle);
      }
    }

    void RbPrmValidation::randomnizeCollisionPairs(){
      for(T_RomValidation::const_iterator cit = romValidations_.begin();
          cit != romValidations_.end(); ++cit)
      {
        cit->second->randomnizeCollisionPairs();
      }
    }

    void RbPrmValidation::computeAllContacts(bool computeAllContacts){
      for(T_RomValidation::const_iterator cit = romValidations_.begin();
          cit != romValidations_.end(); ++cit)
      {
        cit->second->computeAllContacts(computeAllContacts);
      }
    }

  }// namespace rbprm
}// namespace hpp
