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

#include <hpp/rbprm/interpolation/rbprm-path-interpolation.hh>
#include <hpp/rbprm/contact_generation/algorithm.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/rbprm/projection/projection.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/rbprm/interpolation/interpolation-constraints.hh>
#include <hpp/rbprm/sampling/heuristic-tools.hh>
#include <hpp/rbprm/contact_generation/reachability.hh>
#ifdef PROFILE
    #include "hpp/rbprm/rbprm-profiler.hh"
#endif

namespace hpp {
  namespace rbprm {
    namespace interpolation {

    RbPrmInterpolationPtr_t RbPrmInterpolation::create (const hpp::rbprm::RbPrmFullBodyPtr_t robot,
                                                        const hpp::rbprm::State &start, const hpp::rbprm::State &end,
                                                        const core::PathVectorConstPtr_t path, const bool testReachability, const bool quasiStatic)
    {
        RbPrmInterpolation* rbprmDevice = new RbPrmInterpolation(path, robot, start, end, testReachability,quasiStatic);
        RbPrmInterpolationPtr_t res (rbprmDevice);
        res->init (res);
        return res;
    }

    RbPrmInterpolation::~RbPrmInterpolation()
    {
        // NOTHING
    }

    // ========================================================================


    core::Configuration_t RbPrmInterpolation::configPosition(core::ConfigurationIn_t previous, const core::PathVectorConstPtr_t path, double i)
    {
        core::Configuration_t configuration = previous;
        size_t pathConfigSize = path->outputSize() - robot_->device_->extraConfigSpace().dimension();
        core::Configuration_t configPosition(path->outputSize());
        (*path)(configPosition,std::min(i, path->timeRange().second));
        configuration.head(pathConfigSize) = configPosition.head(pathConfigSize);
        configuration.tail(robot_->device_->extraConfigSpace().dimension()) = configPosition.tail(robot_->device_->extraConfigSpace().dimension()) ;
       // configuration[2] = configuration[2] + 0.05; //walk static
        //configuration[2] = configuration[2] + 0.02; //stairs
        return configuration;
    }


    rbprm::T_StateFrame RbPrmInterpolation::Interpolate(const affMap_t& affordances,
            const std::map<std::string, std::vector<std::string> >& affFilters, const double timeStep, const double robustnessTreshold, const bool filterStates)
    {
        if(!path_) throw std::runtime_error ("Cannot interpolate; no path given to interpolator ");
        T_Configuration configs;
        const core::interval_t& range = path_->timeRange();
        configs.push_back(start_.configuration_);
        hppDout(notice,"config start = "<<pinocchio::displayConfig(start_.configuration_));
        int j = 0;
        //for(double i = range.first + timeStep; i< range.second; i+= timeStep)
        for(double i = range.first+timeStep; i< range.second; i+= timeStep, ++j)
        {
            configs.push_back(configPosition(configs.back(),path_,i));
            //hppDout(notice,"config added = "<<pinocchio::displayConfig(configs.back()));
        }
        configs.push_back(configPosition(configs.back(),path_,range.second));
        return Interpolate(affordances, affFilters, configs, robustnessTreshold, timeStep, range.first, filterStates);
    }

    ///
    /// \brief replaceLimbContactForState Try to create a state with all the contacts of stateCurrent, except for LimbId which have the contact of stateGoal
    /// \param robot
    /// \param stateCurrent
    /// \param stateGoal
    /// \param limbId
    /// \return a projection report
    ///
    projection::ProjectionReport replaceLimbContactForState(RbPrmFullBodyPtr_t robot, State stateCurrent, State stateGoal,std::string limbId){
      stateCurrent.RemoveContact(limbId);
      hppDout(notice,"Try to replace contact for limb : "<<limbId);
      pinocchio::Configuration_t configuration = stateCurrent.configuration_;
      core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(robot->device_,"proj", 1e-4, 1000);
      interpolation::addContactConstraints(robot, robot->device_,proj, stateCurrent, stateCurrent.fixedContacts(stateCurrent));
      std::vector<bool> rotationMaskGoal;
      for(std::size_t i =0; i <3; ++i)
      {
          rotationMaskGoal.push_back(true);
      }
      return projection::projectEffector(proj,robot,limbId,robot->GetLimb(limbId),robot->GetCollisionValidation(),configuration,stateGoal.contactRotation_.at(limbId),rotationMaskGoal,stateGoal.contactPositions_.at(limbId),stateGoal.contactNormals_.at(limbId),stateCurrent);
    }

    // greedy algorithm to try the combination of order to move the legs until a successfull one is found
    rbprm::T_StateFrame RbPrmInterpolation::addGoalConfigRec(const rbprm::T_StateFrame& states, const std::vector<std::string> variations){
        hppDout(notice,"addGoalConfigRec, variations size : "<<variations.size());
        if(variations.size() == 1){
          return states;
        }
        State midStateGoal(states.back().second);
        projection::ProjectionReport projReport;
        for(size_t id = 0 ; id < variations.size() ; ++id){
            std::string limbId(variations[id]);
            hppDout(notice,"addGoalConfigRec, try to move limb : "<<limbId);
            projReport = replaceLimbContactForState(robot_,midStateGoal,end_,limbId);
            // If projection was successful, we add the new intermediate state to the results
            if(projReport.success_){
                hppDout(notice,"projection of contact successful");
                hppDout(notice,"New intermediate state : "<<pinocchio::displayConfig(projReport.result_.configuration_));
                rbprm::T_StateFrame results(states.begin(),states.end()); // copy input states
                results.push_back(std::make_pair(states.back().first,projReport.result_)); // there will be 2 states at the same time index ...
                std::vector<std::string> remainingVariations(variations);
                remainingVariations.erase(remainingVariations.begin()+id);
                return addGoalConfigRec(results,remainingVariations);
            }else{
              hppDout(notice,"addGoalConfigRec projection failed for limb "<<limbId);
            }
        } // all projections failed, return original list
        return states;
    }

    // Try to add the desired final configuration at the end of the current contact sequence, while guaranteing that there is always only one contact variation between states
    rbprm::T_StateFrame RbPrmInterpolation::addGoalConfig(const rbprm::T_StateFrame& states){
        hppDout(notice,"AddGoalConfig, size of state list : "<<states.size());
        rbprm::T_StateFrame results(states.begin(),states.end()); // copy input states
        State lastState = states.back().second;


        std::vector<std::string> variationsGoal(lastState.contactVariations(end_)); // all limb that must be moved to reach the goal configuration
        if(variationsGoal.size() ==0){
          hppDout(notice,"no contact variation, return input list");
          return results;
        }else if(variationsGoal.size() == 1){
          hppDout(notice,"specified goal and last state are already adjacent, add the goal state to the list");
          results.push_back(std::make_pair(states.back().first,end_)); // there will be 2 states at the same time index ...
          return results;
        }
        // Last state and end are not adjacent, we keep lastState in the list and produce intermediate states for each contact transition
        hppDout(notice," Last state and end are not adjacent, try to add intermediate state");
        // for each different contact, try to replace to it's position in end_ and add an intermediate state
        results = addGoalConfigRec(states,variationsGoal);
        if((results.back().second.contactVariations(end_)).size() == 1){
          hppDout(notice,"LastState is adjacent to goal, add it to the list and return");
          results.push_back(std::make_pair(path_->timeRange().second,end_));
        }else{
          hppDout(notice,"LastState is still not adjacent to goal, contact sequence do not end with the goal state");
        }
        return results;
    }

    /**
     * @brief loadPreviousConfiguration take the root and extra dof config from config, and the whole body from previous
     * @param previous take the whole body configuration of the configuration
     * @param config take the root and extra dof configuration of this configuration
     * @return
     */
    core::Configuration_t loadPreviousConfiguration(const DevicePtr_t& device,const core::Configuration_t& previous,const core::Configuration_t& config){
        core::Configuration_t res(previous);
        res.head<7>() = config.head<7>();
        res.tail(device->extraConfigSpace().dimension()) = config.tail(device->extraConfigSpace().dimension()) ;
        return res;
    }

    rbprm::T_StateFrame RbPrmInterpolation::Interpolate(const affMap_t& affordances,
                                                        const std::map<std::string, std::vector<std::string> >& affFilters,
                                                        const hpp::rbprm::T_Configuration &configs, const double robustnessTreshold,
                                                        const pinocchio::value_type timeStep, const pinocchio::value_type initValue, const bool filterStates)
    {
        hppDout(notice,"Begin interpolate in path-interpolation, number of configs to test : "<<configs.size());
        int nbFailures = 0;
        size_t accIndex = robot_->device_->configSize() - robot_->device_->extraConfigSpace().dimension () + 3 ; // index of the start of the acceleration vector (of size 3), in the configuration vector
        hppDout(notice,"acceleration index : "<<accIndex);
        pinocchio::value_type currentVal(initValue);
        rbprm::T_StateFrame states;
        states.push_back(std::make_pair(currentVal, this->start_));
        Configuration_t lastConfig(this->start_.configuration_);
        CIT_Configuration lastIterator = configs.begin();
        std::size_t nbRecontacts = 0;
        std::size_t repos = 0;
        bool allowFailure = true;
        Eigen::Vector3d dir,acc; acc = Eigen::Vector3d::Zero();
        const PathConstPtr_t comPath = boost::dynamic_pointer_cast<const core::Path>(path_);
#ifdef PROFILE
    RbPrmProfiler& watch = getRbPrmProfiler();
    watch.reset_all();
    watch.start("complete generation");
#endif
        for(CIT_Configuration cit = configs.begin()+1; cit != configs.end(); ++cit, currentVal+= timeStep)
        {
            const State& previous = states.back().second;
            core::Configuration_t configuration = loadPreviousConfiguration(robot_->device_,lastConfig,*cit);
            if( accIndex < (std::size_t)configuration.size())
            {
                acc = configuration.segment<3>(accIndex);
                dir = configuration.segment<3>(accIndex-3);
            }
            else
                dir = configuration.head<3>() - previous.configuration_.head<3>();
            fcl::Vec3f direction(dir[0], dir[1], dir[2]);
            bool nonZero(false);
            direction.normalize();
            if(!nonZero) direction = fcl::Vec3f(0,0,1.);
            // TODO Direction 6d
            hppDout(notice,"#call ComputeContact, looking for state "<<states.size()-1);
            hpp::rbprm::contact::ContactReport rep = contact::ComputeContacts(previous, robot_,configuration, affordances,affFilters,direction,robustnessTreshold,acc,comPath,currentVal,testReachability_,quasiStatic_);
            State& newState = rep.result_;


            const bool  sameAsPrevious = rep.success_ && rep.contactMaintained_ && !rep.contactCreated_;
            const bool& multipleBreaks = rep.multipleBreaks_;
            const bool& respositioned = rep.repositionedInPlace_;
            hppDout(notice,"success : "<<rep.success_<<" ; contacts maintained  : "<<rep.contactMaintained_<<" ; contact created : "<<rep.contactCreated_<<" ; sameAsPrevious : "<<sameAsPrevious<<" ; multipleBreak : "<<multipleBreaks<<" ; repositionned : "<<respositioned<<" ; allowFailure : "<<allowFailure<<" ; status :"<<rep.status_);

            if(allowFailure && (!rep.success_ || rep.multipleBreaks_))
            {
                ++ nbFailures;
                if(cit != configs.end() && (cit+1)!= configs.end())
                {
                    ++cit;
                }
                currentVal+= timeStep;
if (nbFailures > 1)
{
    /*
    std::ofstream fout;
    fout.open("/local/fernbac/bench_iros18/success/log_success.log",std::fstream::app);
    fout<<"failed."<<std::endl;
    fout.close();
    */
    std::cout << "failed " << std::endl;
#ifdef PROFILE
    watch.stop("complete generation");
    watch.add_to_count("planner failed", 1);
    std::ofstream fout;
    fout.open("log.txt", std::fstream::out | std::fstream::app);
    std::ostream* fp = &fout;
    watch.report_count(*fp);
    fout.close();
#endif
    hppDout(notice,"Abort interpolate, too much fails");
    return FilterStates(states, filterStates);
    //return states;
}
            }
            if(multipleBreaks && !allowFailure)
            {
                ++nbRecontacts;
                cit--;
                currentVal-= timeStep;
            }
            else if(!multipleBreaks && respositioned)
            {
                ++nbRecontacts;
                cit--;
                currentVal-= timeStep;
            }
            else
            {
                nbRecontacts = 0;
            }
            if(respositioned)
            {
                ++repos;
                if (repos > 20)
                {
                /*std::ofstream fout;
                fout.open("/local/fernbac/bench_iros18/success/log_success.log",std::fstream::app);
                fout<<"failed, too much repositionning"<<std::endl;
                fout.close();
                */
				std::cout<<"failed, too much repositionning"<<std::endl;
				#ifdef PROFILE
					watch.stop("complete generation");
					watch.add_to_count("planner failed", 1);
					std::ofstream fout;
					fout.open("log.txt", std::fstream::out | std::fstream::app);
					std::ostream* fp = &fout;
					watch.report_count(*fp);
					fout.close();
				#endif
                    hppDout(notice,"Abort interpolate, too much repositionning");
                    return FilterStates(states, filterStates);
                }
                cit = lastIterator;
                currentVal = states.back().first;
            }

            newState.nbContacts = newState.contactNormals_.size();
            /*
            // code to add the last valid config for each state :
            if(sameAsPrevious){
                states.pop_back();
            }
            else{
              hppDout(notice,"new state added at index "<<states.size()-1<<" conf = r(["<<pinocchio::displayConfig(states.back().second.configuration_)<<"])");
               hppDout(notice,"First configuration for state "<<states.size()<<" : r(["<<pinocchio::displayConfig(newState.configuration_)<<"])");
            }
            if(states.empty()){ // initial state, we keep the initial configuration but update the timing
                states.push_back(std::make_pair(currentVal, this->start_));
            }else{
                states.push_back(std::make_pair(currentVal, newState));
            }
            */

            // code to add the first valid config of each states :
            if(!sameAsPrevious){
                states.push_back(std::make_pair(currentVal, newState));
                hppDout(notice,"new state added at index "<<states.size()-1<<" conf = r(["<<pinocchio::displayConfig(states.back().second.configuration_)<<"])");
                lastIterator = cit;
            }else{
                hppDout(notice,"Same as previous, new config = r(["<<pinocchio::displayConfig(newState.configuration_)<<"])");
            }
            //allowFailure = nbRecontacts < robot_->GetLimbs().size();
            allowFailure = nbRecontacts < 2;
            lastConfig = newState.configuration_;
        }
        //states.push_back(std::make_pair(this->path_->timeRange().second, this->end_));
#ifdef PROFILE
        watch.add_to_count("planner succeeded", 1);
        watch.stop("complete generation");
        /*std::ofstream fout;
        fout.open("log.txt", std::fstream::out | std::fstream::app);
        std::ostream* fp = &fout;
        watch.report_all_and_count(2,*fp);
        fout.close();*/
#endif
        /*
        std::ofstream fout;
        fout.open("/local/fernbac/bench_iros18/success/log_success.log",std::fstream::app);
        fout<<"Planner succeeded"<<std::endl;
        fout.close();
        */
        hppDout(notice,"Interpolate finished, filter and add goal : ");
        if(states.size() > 1){
          states = addGoalConfig(states);
          states = FilterStates(states, filterStates);
        }
        return states;
    }

    void RbPrmInterpolation::init(const RbPrmInterpolationWkPtr_t& weakPtr)
    {
        weakPtr_ = weakPtr;
    }

    RbPrmInterpolation::RbPrmInterpolation (const core::PathVectorConstPtr_t path, const hpp::rbprm::RbPrmFullBodyPtr_t robot, const hpp::rbprm::State &start, const hpp::rbprm::State &end, const bool testReachability, const bool quasiStatic)
        : path_(path)
        , start_(start)
        , end_(end)
        , testReachability_(testReachability)
        , quasiStatic_(quasiStatic)
        , robot_(robot)
    {
        // TODO
    }

    bool EqStringVec(const std::vector<std::string>& v1, const std::vector<std::string>& v2)
    {
        return (v1.size() == v2.size()) && std::equal ( v1.begin(), v1.end(), v2.begin() );
    }

    StateFrame RbPrmInterpolation::findBestRepositionState(T_StateFrame candidates,std::vector<std::string> limbsNames){
      StateFrame bestState = candidates.back();
      double bestScore = -std::numeric_limits<double>::infinity();
      double currentScore;
      fcl::Vec3f direction = candidates.back().second.configuration_.head<3>() - candidates.front().second.configuration_.head<3>();
      // TODO : iterate over candidates and call the heuristic for limbsName. Then return the state with the best score (average of scores if more than one limbs)
      for(CIT_StateFrame sit=candidates.begin() ; sit != candidates.end() ; ++sit){
        currentScore=0;
        for(std::vector<std::string>::const_iterator lit = limbsNames.begin() ; lit != limbsNames.end() ; ++lit){
          RbPrmLimbPtr_t limb = robot_->GetLimbs().at(*lit);
          sampling::Sample sample(limb->limb_, limb->effector_, sit->second.configuration_,  limb->offset_,limb->limbOffset_, 0);
          currentScore += limb->evaluate_(sample,direction, sit->second.contactNormals_.at(*lit),sampling::HeuristicParam());
        }
        if(currentScore>bestScore){
          bestScore=currentScore;
          bestState = *sit;
          hppDout(notice,"here, config = "<<pinocchio::displayConfig(bestState.second.configuration_));
        }
      }
      hppDout(notice,"Filtering, looking for best candidate : best score = "<<bestScore);

      return bestState;
    }

    bool RbPrmInterpolation::testReachability(const State& s0, const State& s1){
        if(testReachability_){
            State state0(s0);
            State state1(s1);
            reachability::Result resReachability;
            if(quasiStatic_){
                resReachability = reachability::isReachable(robot_,state0,state1);
            }else{
                resReachability = reachability::isReachableDynamic(robot_,state0,state1);
            }
            return resReachability.success();
        }else{
            return true;
        }
    }

    void RbPrmInterpolation::FilterRepositioning(const CIT_StateFrame& from, const CIT_StateFrame to, T_StateFrame& res)
    {
        if(from == to) return;
        State current    = (from)->second;
        State current_m1 = (from-1)->second;
        State current_p1 = (from+1)->second;
        if(EqStringVec(current.contactBreaks(current_m1),
                       current_p1.contactBreaks(current_m1)) &&
           EqStringVec(current.contactCreations(current_m1),
                       current_p1.contactCreations(current))
                && testReachability(current_m1,current_p1))
        {
            if(from+1 == to) return;
            // Check if there is others state with the same contacts, and only add the one with the best score for the heuristic :
            /*bool reposition(true);
            size_t id = 2;
            T_StateFrame repositionnedStates;
            repositionnedStates.push_back(std::make_pair((from)->first, (from)->second));
            repositionnedStates.push_back(std::make_pair((from+1)->first, (from+1)->second));
            while(reposition && (from+id != to)){
              current_m1=(from+id-1)->second;
              current=(from+id)->second;
              current_p1 = (from+id+1)->second;
              if(EqStringVec(current.contactBreaks(current_m1),
                             current_p1.contactBreaks(current_m1)) &&
                 EqStringVec(current.contactCreations(current_m1),
                             current_p1.contactCreations(current))){
                repositionnedStates.push_back(std::make_pair((from+id)->first, (from+id)->second));
                repositionnedStates.push_back(std::make_pair((from+id+1)->first, (from+id+1)->second));
                id+=2;
              }
              else
                reposition = false;

            }
            hppDout(notice,"repositionned contacts found : number of states = "<<repositionnedStates.size());
            // iterate over respoitionnedStates and find the one with the best score with the heuristic (for the limb that move)

            T_StateFrame repositionnedStates;
            repositionnedStates.push_back(std::make_pair((from)->first, (from)->second));
            repositionnedStates.push_back(std::make_pair((from+1)->first, (from+1)->second));
            std::vector<std::string> limbsNames = current.contactCreations(current_m1);
            StateFrame bestState = findBestRepositionState(repositionnedStates,limbsNames);
            */            
            res.push_back(std::make_pair((from+1)->first, (from+1)->second));
            FilterRepositioning(from+2, to, res);
        }
        else
        {
            res.push_back(std::make_pair(from->first, from->second));
            FilterRepositioning(from+1, to, res);
        }
    }

    void RbPrmInterpolation::FilterBreakCreate(const CIT_StateFrame& from, const CIT_StateFrame to, T_StateFrame& res)
    {
        if(from == to) return;
        const State& current    = (from)->second;
        const State& current_m1 = (from-1)->second;
        const State& current_p1 = (from+1)->second;
        if(current.contactCreations(current_m1).empty()  &&
           current_p1.contactBreaks(current).empty() &&
           EqStringVec(current_p1.contactCreations(current),
                       current.contactBreaks(current_m1))
                &&  testReachability(current_m1,current_p1))
        {
            if(from+1 == to) return;
            res.push_back(std::make_pair((from+1)->first, (from+1)->second));
            FilterBreakCreate(from+2, to, res);
        }
        else
        {
            res.push_back(std::make_pair(from->first, from->second));
            FilterBreakCreate(from+1, to, res);
        }
    }

    T_StateFrame RbPrmInterpolation::FilterRepositioning(const T_StateFrame& originStates)
    {
        if(originStates.size() < 3) return originStates;
        T_StateFrame res;
        res.push_back(originStates.front());
        FilterRepositioning(originStates.begin()+1, originStates.end()-1, res);
        res.push_back(originStates.back());
        return res;
    }

    T_StateFrame RbPrmInterpolation::FilterBreakCreate(const T_StateFrame& originStates)
    {
        if(originStates.size() < 3) return originStates;
        T_StateFrame res;
        res.push_back(originStates.front());
        FilterBreakCreate(originStates.begin()+1, originStates.end()-1, res);
        res.push_back(originStates.back());
        return res;
    }

    T_StateFrame FilterObsolete(const T_StateFrame& originStates)
    {
        if(originStates.size() < 3) return originStates;
        T_StateFrame res;
        res.push_back(originStates.front());
        CIT_StateFrame cit = originStates.begin();
        for(CIT_StateFrame cit2 = originStates.begin()+1;
            cit2 != originStates.end()-1; ++cit, ++cit2)
        {
            const State& current    = (cit2)->second;
            const State& current_m1 = (cit)->second;
            if((current.configuration_ - current_m1.configuration_).norm() > std::numeric_limits<double>::epsilon()
                    && !(current.contactBreaks(current_m1).empty() && current.contactCreations(current_m1).empty()))
            {
                res.push_back(std::make_pair(cit2->first, cit2->second));
            }
        }
        res.push_back(originStates.back());

        cit = res.begin();
        std::size_t idx = 0;
        for(T_StateFrame::const_iterator cit2 = res.begin()+1; cit2 != res.end()-1; ++cit, ++cit2, ++idx)
        {
            const State prev = cit->second;
            const State next = cit2->second;
            std::vector<std::string> breaks = next.contactBreaks(prev);
            std::vector<std::string> creations = next.contactCreations(prev);
            if(breaks.size() > 1 || creations.size() > 1)
            {
                std::cout << "AFTER FILTER " << std::endl;
                std::cout << "\t REMOVING CONTACT " << breaks.size() << std::endl;
                for(std::vector<std::string>::const_iterator tf = breaks.begin(); tf != breaks.end(); ++tf)
                    std::cout << "\t \t " << *tf << std::endl;
                std::cout << "\t CREATING CONTACT " << creations.size() << std::endl;
                for(std::vector<std::string>::const_iterator tf = creations.begin(); tf != creations.end(); ++tf)
                    std::cout << "\t \t " << *tf << std::endl;

                std::cout << "END AFTERAFTER FILTER  " << breaks.size() << std::endl;
            }

        }
        return res;
    }

    void RbPrmInterpolation::tryReplaceStates(const CIT_StateFrame& from, const CIT_StateFrame to, T_StateFrame& res){
        if(from == to){
            res.push_back(std::make_pair(from->first, from->second));
            res.push_back(std::make_pair((from+1)->first, (from+1)->second));
            return;
        }
        const State& ci0 = (from)->second;
        const State& ci1 = (from+1)->second;
        const State& ci2 = (from+2)->second;
        const State& ci3 = (from+3)->second;

        hppDout(notice,"Try Replace State : ");
        if(ci3.contactCreations(ci2).size() == 1
        && EqStringVec(ci1.contactBreaks(ci0), ci3.contactBreaks(ci2))
        && EqStringVec(ci1.contactBreaks(ci0), ci1.contactCreations(ci0))
        && EqStringVec(ci1.contactBreaks(ci0), ci3.contactCreations(ci2))
        && !EqStringVec(ci1.contactBreaks(ci0), ci2.contactBreaks(ci1))
        && !EqStringVec(ci1.contactCreations(ci0), ci2.contactCreations(ci1))){
            hppDout(notice,"condition on contact OK");
            // try to create a state s1_bis : s1 with the new contact in the same position as in s3
            //robot_->device_->currentConfiguration(ci3.configuration_);
            //robot_->device_->computeForwardKinematics();
            State s1_bis(ci1);
            s1_bis.configuration_=ci3.configuration_;
            // get contact information from state 3 :
            std::string contactCreate = ci3.contactCreations(ci2)[0];
            hppDout(notice,"contact to change : "<<contactCreate);
            fcl::Vec3f n = ci3.contactNormals_.at(contactCreate);
            fcl::Vec3f p = ci3.contactPositions_.at(contactCreate) + robot_->GetLimb(contactCreate)->offset_;
            fcl::Vec3f p1 = ci1.contactPositions_.at(contactCreate) + robot_->GetLimb(contactCreate)->offset_;
            hppDout(notice,"position : "<<p);
            hppDout(notice,"normal   : "<<n);
            hppDout(notice,"difference with previous position : "<<(p1-p).norm());
           // fcl::Matrix3f r = ci3.contactRotation_.at(contactCreate);
            projection::ProjectionReport rep = projection::projectStateToObstacle(robot_,contactCreate,robot_->GetLimb(contactCreate),s1_bis,n,p);
            hppDout(notice,"projection success : "<<rep.success_);
            if(rep.success_){
                rep = projection::projectToRootConfiguration(robot_,ci1.configuration_,rep.result_);
            }
            ValidationReportPtr_t rport (ValidationReportPtr_t(new CollisionValidationReport));
            if((p1-p).norm() < 0.2 && rep.success_ && robot_->GetCollisionValidation()->validate(rep.result_.configuration_,rport)
                   &&  testReachability(rep.result_,ci3) ){
                hppDout(notice,"projection is collision free !");
                // success ! add s1_bis instead of s1, and skip s2 :
                res.push_back(std::make_pair(from->first, from->second));
                res.push_back(std::make_pair((from+1)->first, rep.result_));
                if(from+1 == to) return;
                if(from+2 == to){
                    res.push_back(std::make_pair((from+3)->first, (from+3)->second));
                    return;
                }
                tryReplaceStates(from+3, to, res);
            }else
            {
                res.push_back(std::make_pair(from->first, from->second));
                tryReplaceStates(from+1, to, res);
            }
        }
        else
        {
            res.push_back(std::make_pair(from->first, from->second));
            tryReplaceStates(from+1, to, res);
        }
    }


    T_StateFrame RbPrmInterpolation::tryReplaceStates( const T_StateFrame& originStates){
        hppDout(notice,"Begin tryReplaceStates, size of list : "<<originStates.size());
        if(originStates.size() < 4) return originStates;
        T_StateFrame res;
        tryReplaceStates(originStates.begin(), originStates.end()-3, res);
        res.push_back(originStates.back());
        return res;
    }

    void RbPrmInterpolation::trySkipStates(const CIT_StateFrame& from, const CIT_StateFrame to, T_StateFrame& res){
        if(from == to){
            res.push_back(std::make_pair(from->first, from->second));
            res.push_back(std::make_pair((from+1)->first, (from+1)->second));
            return;
        }
        const State& ci0 = (from)->second;
        const State& ci1 = (from+1)->second;
        const State& ci2 = (from+2)->second;
        const State& ci3 = (from+3)->second;

        hppDout(notice,"Try Skip State : ");
        if(ci2.contactCreations(ci1).size() == 1
        && EqStringVec(ci1.contactBreaks(ci0), ci3.contactBreaks(ci2))
        && EqStringVec(ci1.contactBreaks(ci0), ci1.contactCreations(ci0))
        && EqStringVec(ci1.contactBreaks(ci0), ci3.contactCreations(ci2))
        && !EqStringVec(ci1.contactBreaks(ci0), ci2.contactBreaks(ci1))
        && !EqStringVec(ci1.contactCreations(ci0), ci2.contactCreations(ci1))){
            hppDout(notice,"condition on contact OK");
            // try to create a state s2_bis : s2 with the previous contact in the same position as in s0
            State s2_bis(ci2);
            s2_bis.configuration_ = ci0.configuration_;
            // get contact information from state 0 :
            std::string contactCreate = ci1.contactCreations(ci0)[0];
            hppDout(notice,"contact to change : "<<contactCreate);
            fcl::Vec3f n = ci0.contactNormals_.at(contactCreate);
            fcl::Vec3f p = ci0.contactPositions_.at(contactCreate) + robot_->GetLimb(contactCreate)->offset_;
            p -= n*10e-3 ; // FIXME see 'epsilon' in projection::computeProjectionMatrix, why is it added ?
            fcl::Vec3f p1 = ci1.contactPositions_.at(contactCreate) + robot_->GetLimb(contactCreate)->offset_;
            p1 -= ci1.contactNormals_.at(contactCreate)*10e-3 ;
            hppDout(notice,"position : "<<p);
            hppDout(notice,"normal   : "<<n);
            hppDout(notice,"difference with previous position : "<<(p1-p).norm());
           // fcl::Matrix3f r = ci3.contactRotation_.at(contactCreate);
            projection::ProjectionReport rep = projection::projectStateToObstacle(robot_,contactCreate,robot_->GetLimb(contactCreate),s2_bis,n,p);
            hppDout(notice,"projection success : "<<rep.success_);
            if(rep.success_){
                rep = projection::projectToRootConfiguration(robot_,ci2.configuration_,rep.result_);
            }
            ValidationReportPtr_t rport (ValidationReportPtr_t(new CollisionValidationReport));
            if((p1-p).norm() < 0.1 && rep.success_ && robot_->GetCollisionValidation()->validate(rep.result_.configuration_,rport)
                   &&  testReachability(ci0,rep.result_) && testReachability(rep.result_,ci3) ){
                hppDout(notice,"projection is collision free !");
                // success ! add s2_bis instead of s2, and skip s1 :
                res.push_back(std::make_pair(from->first, from->second));
                res.push_back(std::make_pair((from+2)->first, rep.result_));
                if(from+1 == to) return;
                if(from+2 == to){
                    res.push_back(std::make_pair((from+3)->first, (from+3)->second));
                    return;
                }
                trySkipStates(from+3, to, res);
            }else
            {
                res.push_back(std::make_pair(from->first, from->second));
                trySkipStates(from+1, to, res);
            }
        }
        else
        {
            res.push_back(std::make_pair(from->first, from->second));
            trySkipStates(from+1, to, res);
        }
    }

    T_StateFrame RbPrmInterpolation::trySkipStates( const T_StateFrame& originStates){
        hppDout(notice,"Begin trySkipStates, size of list : "<<originStates.size());
        if(originStates.size() < 4) return originStates;
        T_StateFrame res;
        trySkipStates(originStates.begin(), originStates.end()-3, res);
        res.push_back(originStates.back());
        return res;
    }




    T_StateFrame RbPrmInterpolation::FilterStatesRec( const T_StateFrame& originStates)
    {
        hppDout(notice,"FilterStatesRec");
        return trySkipStates(tryReplaceStates(FilterObsolete(FilterBreakCreate(FilterRepositioning(originStates)))));
    }

    T_StateFrame RbPrmInterpolation::FilterStates( const T_StateFrame& originStates, const bool deep)
    {
        //make sure they re ok
        hppDout(notice,"Begin filter states !");
        hppDout(notice,"Number of state before filtering : "<<originStates.size());
        if(originStates.size() <= 2 ){
          hppDout(notice,"Return original state list");
          return originStates;
        }
        T_StateFrame::const_iterator cit = originStates.begin();
        std::size_t idx = 0;
        for(T_StateFrame::const_iterator cit2 = originStates.begin()+1; cit2 != originStates.end()-1; ++cit, ++cit2, ++idx)
        {
            const State prev = cit->second;
            const State next = cit2->second;
            std::vector<std::string> breaks = next.contactBreaks(prev);
            std::vector<std::string> creations = next.contactCreations(prev);
            if(breaks.size() > 1 || creations.size() > 1)
            {
                hppDout(notice,"too many contact changes between the two states.");
                std::cout << "BEFORE FILTER " << std::endl;
                std::cout << "\t REMOVING CONTACT " << breaks.size() << std::endl;
                for(std::vector<std::string>::const_iterator tf = breaks.begin(); tf != breaks.end(); ++tf)
                    std::cout << "\t \t " << *tf << std::endl;
                std::cout << "\t CREATING CONTACT " << creations.size() << std::endl;
                for(std::vector<std::string>::const_iterator tf = creations.begin(); tf != creations.end(); ++tf)
                    std::cout << "\t \t " << *tf<< std::endl;

                std::cout << "END BEFORE FILTER  " << breaks.size() << std::endl;
            }

        }
        hppDout(notice,"End of checks for too many contact changes, filter redunbdant states, size of list :  "<<originStates.size());
        T_StateFrame res = originStates;
        if(deep)
        {
            std::size_t previousSize;
            do
            {
                hppDout(notice,"Begin iteration of filtering, size of res : "<<res.size());
                previousSize = res.size();
                res = FilterStatesRec(res);
                hppDout(notice,"End iteration of filtering, size of res : "<<res.size());
            }
            while(res.size() != previousSize);
            hppDout(notice,"End of filtering, final size : "<<res.size());
            return res;
        }
        else
        {
            return FilterObsolete(originStates);
        }
    }
    } // interpolation
  } // rbprm
} //hpp
