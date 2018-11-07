//
// Copyright (c) 2017 CNRS
// Authors: Steve Tonneau
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

#include <hpp/rbprm/contact_generation/contact_generation.hh>
#include <hpp/rbprm/stability/stability.hh>
#include <hpp/rbprm/contact_generation/reachability.hh>
#include <hpp/rbprm/tools.hh>
#include <hpp/pinocchio/configuration.hh>
#include <pinocchio/spatial/se3.hpp>
#include <hpp/rbprm/sampling/heuristic-tools.hh>

#ifdef PROFILE
    #include "hpp/rbprm/rbprm-profiler.hh"
#endif


namespace hpp {
namespace rbprm {
namespace contact{


ContactGenHelper::ContactGenHelper(RbPrmFullBodyPtr_t fb, const State& ps, pinocchio::ConfigurationIn_t configuration,
                                    const hpp::rbprm::affMap_t &affordances, const std::map<std::string, std::vector<std::string> > &affFilters,
                                    const double robustnessTreshold,
                                    const std::size_t maxContactBreaks, const std::size_t maxContactCreations,
                                    const bool checkStabilityMaintain, const bool checkStabilityGenerate,
                                    const fcl::Vec3f& direction,
                                    const fcl::Vec3f& acceleration,
                                    const bool contactIfFails,
                                    const bool stableForOneContact, const core::PathConstPtr_t &comPath, const double currentPathId)
: fullBody_(fb)
, previousState_(ps)
, checkStabilityMaintain_(checkStabilityMaintain)
, contactIfFails_(contactIfFails)
, stableForOneContact_(stableForOneContact)
, acceleration_(acceleration)
, direction_(direction)
, robustnessTreshold_(robustnessTreshold)
, maxContactBreaks_(maxContactBreaks)
, maxContactCreations_(maxContactCreations)
, affordances_(affordances)
, affFilters_(affFilters)
, workingState_(previousState_)
, checkStabilityGenerate_(checkStabilityGenerate)
, comPath_(comPath)
, currentPathId_(currentPathId)
, quasiStatic_(false)
, testReachability_(true)
, maximiseContacts_(true)
, accept_unreachable_(false)
, tryQuasiStatic_(false)
, reachabilityPointPerPhases_(0)
{
    workingState_.configuration_ = configuration;
    workingState_.stable = false;
}

typedef std::vector<T_State > T_DepthState;

bool push_if_new(T_State& states, const State currentState)
{
    for(CIT_State cit = states.begin(); cit != states.end(); ++cit)
    {
        if(currentState.contactOrder_== cit->contactOrder_)
            return false;
    }
    states.push_back(currentState);
    return true;
}

void reverse(std::queue<std::string> &queue){
    assert(!queue.empty());
    std::string temp(queue.front());
    queue.pop();
    if(!queue.empty())
        reverse(queue);
    queue.push(temp);
}

void maintain_contacts_combinatorial_rec(const hpp::rbprm::State& currentState, const std::size_t  depth,
                                         const std::size_t maxBrokenContacts, T_DepthState& res)
{
    hppDout(notice,"maintain contact combinatorial recursif : ");
    if (!push_if_new(res[depth], currentState) || depth>=maxBrokenContacts) return;
    std::queue<std::string> contactOrder = currentState.contactOrder_;
    // TEST CODE : reverse order of the queue :
    hppDout(notice,"Size of contact queue : "<<contactOrder.size());
    hppDout(notice,"Reverse :");
    reverse(contactOrder);
    hppDout(notice,"Reverse done.");
    int size = contactOrder.size(); int i = 0;
    while(!contactOrder.empty() && size != i)
    {
        hpp::rbprm::State copyState = currentState;
        std::vector<std::string> fixed = currentState.fixedContacts(currentState);
        const std::string contactRemoved = contactOrder.front();
        //if(!
        //((std::find(fixed.begin(), fixed.end(),std::string("hrp2_rleg_rom")) == fixed.end() && contactRemoved == std::string("hrp2_lleg_rom")) ||
        //(std::find(fixed.begin(), fixed.end(),std::string("hrp2_lleg_rom")) == fixed.end() && contactRemoved == std::string("hrp2_rleg_rom"))))
        {
            hppDout(notice,"Try to remove contact "<<contactRemoved);
            copyState.RemoveContact(contactRemoved);
            hppDout(notice,"Done.");
            maintain_contacts_combinatorial_rec(copyState, depth+1, maxBrokenContacts, res);
        }
        /*else
{
 std::cout << "avoided both leg removed"    << std::endl;
 contactOrder.push(contactRemoved);
}*/
        ++i;
        contactOrder.pop();
    }
}

Q_State flatten(const T_DepthState& depthstates)
{
    Q_State res;
    for(T_DepthState::const_iterator cit = depthstates.begin(); cit != depthstates.end(); ++cit)
    {
        for(CIT_State ccit = cit->begin(); ccit != cit->end(); ++ccit)
            res.push(*ccit);
    }
    return res;
}

Q_State maintain_contacts_combinatorial(const hpp::rbprm::State& currentState, const std::size_t maxBrokenContacts)
{
    T_DepthState res(maxBrokenContacts+1);
    hppDout(notice,"maintain contact combinatorial : ");
    maintain_contacts_combinatorial_rec(currentState, 0, maxBrokenContacts,res);
    return flatten(res);
}

using namespace projection;

bool maintain_contacts_stability_rec(hpp::rbprm::RbPrmFullBodyPtr_t fullBody,
                        pinocchio::ConfigurationIn_t targetRootConfiguration,
                        Q_State& candidates,const std::size_t contactLength,
                        const fcl::Vec3f& acceleration, const double robustness,
                        ProjectionReport& currentRep)
{
    if(stability::IsStable(fullBody,currentRep.result_, acceleration) > robustness)
    {
        currentRep.result_.stable = true;
        return true;
    }
    currentRep.result_.stable = false;
    if(!candidates.empty())
    {
        State cState = candidates.front();
        candidates.pop();
         // removed more contacts, cannot be stable if previous state was not
        if(cState.contactOrder_.size() < contactLength) return false;
        ProjectionReport rep = projectToRootConfiguration(fullBody,targetRootConfiguration,cState);
        Q_State copy_candidates = candidates;
        if(maintain_contacts_stability_rec(fullBody,targetRootConfiguration,copy_candidates,contactLength,acceleration, robustness, rep))
        {
            currentRep = rep;
            candidates = copy_candidates;
            return true;
        }
    }
    return false;
}

pinocchio::CollisionObjectPtr_t second( const std::pair<std::string, pinocchio::CollisionObjectPtr_t> &p ) {
    return p.second;
}

std::vector<pinocchio::CollisionObjectPtr_t> getAffObjectsForLimb(const std::string& limb,
    const affMap_t& affordances, const std::map<std::string, std::vector<std::string> >& affFilters)
{
    std::vector<pinocchio::CollisionObjectPtr_t> affs;
    std::vector<std::string> affTypes;
    bool settingFound = false;
    for (std::map<std::string, std::vector<std::string> >::const_iterator fIt =
        affFilters.begin (); fIt != affFilters.end (); ++fIt)
    {
        std::size_t found = fIt->first.find(limb);
        if (found != std::string::npos)
        {
            affTypes = fIt->second;
            settingFound = true;
            break;
        }
    }
    if (!settingFound)
    {
        // TODO: Keep warning or delete it?
        std::cout << "No affordance filter setting found for limb " << limb
            << ". Has such filter been set?" << std::endl;
        // Use all AFF OBJECTS as default if no filter setting exists
        for (std::map<std::string, std::vector< std::pair<std::string, pinocchio::CollisionObjectPtr_t> > >::const_iterator affordanceIt = affordances.map.begin ();
            affordanceIt != affordances.map.end (); ++affordanceIt)
        {
            std::transform (affordanceIt->second.begin (), affordanceIt->second.end (), std::back_inserter (affs), second);
        }
    }
    else
    {
        for (std::vector<std::string>::const_iterator affTypeIt = affTypes.begin ();
            affTypeIt != affTypes.end (); ++affTypeIt)
        {
            affMap_t::const_iterator affIt = affordances.map.find(*affTypeIt);
            std::transform (affIt->second.begin (), affIt->second.end (), std::back_inserter (affs), second);
        }
    }
    if (affs.empty())
        throw std::runtime_error ("No aff objects found for limb " + limb);
    return affs;
}

ProjectionReport maintain_contacts_stability(ContactGenHelper &contactGenHelper, ProjectionReport& currentRep)
{
    const std::size_t contactLength(currentRep.result_.contactOrder_.size());
//contactGenHelper.candidates_.pop(); // TODO REMOVE (TEST)
    maintain_contacts_stability_rec(contactGenHelper.fullBody_,
                                    contactGenHelper.workingState_.configuration_,
                                    contactGenHelper.candidates_,
                                    contactLength, contactGenHelper.acceleration_,
                                    contactGenHelper.robustnessTreshold_, currentRep);
    hppDout(notice,"check stability maintain contact : "<<currentRep.result_.stable);
    return currentRep;
}


std::vector<std::string> extractEffectorsName(const rbprm::T_Limb& limbs)
{
    std::vector<std::string> res;
    for(rbprm::T_Limb::const_iterator cit = limbs.begin(); cit != limbs.end(); ++cit)
        res.push_back(cit->first);
    return res;
}

std::vector<std::string> sortLimbs(const State& currentState, const std::vector<std::string>& freeLimbs)
{
    std::vector<std::string> res1;
    std::vector<std::string> res2;
    // first add unused limbs
    // then undraw from contact order
    std::queue<std::string> order = currentState.contactOrder_;
    while(!order.empty())
    {
        const std::string l = order.front(); order.pop();
        if(std::find(freeLimbs.begin(), freeLimbs.end(), l) != freeLimbs.end())
            tools::insertIfNew(res1, l);
    }
    for(std::vector<std::string>::const_iterator cit = freeLimbs.begin(); cit!= freeLimbs.end(); ++cit)
    {
        if(std::find(res1.begin(), res1.end(), *cit) == res1.end())
        {
            res2.push_back(*cit);
        }
    }
    res2.insert(res2.end(), res1.begin(), res1.end());
    //res2.insert(res2.begin(), res1.begin(), res1.end());
    return res2;
}

ProjectionReport genColFree(ContactGenHelper &contactGenHelper, ProjectionReport& currentRep)
{
    ProjectionReport res = currentRep;
    // identify broken limbs and find collision free configurations for each one of them.
    std::vector<std::string> effNames(extractEffectorsName(contactGenHelper.fullBody_->GetLimbs()));
    std::vector<std::string> freeLimbs = rbprm::freeEffectors(currentRep.result_,effNames.begin(), effNames.end() );
    freeLimbs = sortLimbs(contactGenHelper.workingState_, freeLimbs);
    for(std::vector<std::string>::const_iterator cit = freeLimbs.begin(); cit != freeLimbs.end() && res.success_; ++cit){
        res = projection::setCollisionFree(contactGenHelper.fullBody_,contactGenHelper.fullBody_->GetLimbCollisionValidation().at(*cit),*cit,res.result_);
        hppDout(notice,"free limb in maintain contact : "<<*cit);
    }
    // gen collision free configuration for the limbs not used for contact in last :
    hppDout(notice,"size of val map = "<<contactGenHelper.fullBody_->GetLimbCollisionValidation().size());
    std::vector<std::string> effNotContactingNames(extractEffectorsName(contactGenHelper.fullBody_->GetNonContactingLimbs()));
    for(std::vector<std::string>::const_iterator cit = effNotContactingNames.begin(); cit != effNotContactingNames.end() && res.success_; ++cit){
        hppDout(notice,"for limb name = "<<*cit);
        res = projection::setCollisionFree(contactGenHelper.fullBody_,contactGenHelper.fullBody_->GetLimbCollisionValidation().at(*cit),*cit,res.result_);
        hppDout(notice,"free limb not used for contact in maintain contact : "<<*cit);
    }
    return res;
}

void stringCombinatorialRec(std::vector<std::vector<std::string> >& res, const std::vector<std::string>& candidates, const std::size_t depth)
{
    if(depth == 0) return;
    std::vector<std::vector<std::string> > newStates;
    for(std::vector<std::vector<std::string> >::iterator it = res.begin(); it != res.end(); ++it)
    {
        for(std::vector<std::string>::const_iterator canditates_it = candidates.begin(); canditates_it != candidates.end(); ++canditates_it)
        {
            std::vector<std::string> contacts = *it;
            if(tools::insertIfNew(contacts,*canditates_it))
            {
                newStates.push_back(contacts);
            }
        }
    }
    stringCombinatorialRec(newStates, candidates, depth-1);
    res.insert(res.end(),newStates.begin(),newStates.end());
}


std::vector<std::vector<std::string> > stringCombinatorial(const std::vector<std::string>& candidates, const std::size_t maxDepth,const bool maximiseContacts = false)
{
    std::vector<std::vector<std::string> > res;
    std::vector<std::string> tmp;
    res.push_back(tmp);
    stringCombinatorialRec(res, candidates, maxDepth);
    if(maximiseContacts){
        // put first element (no contact creation) at the end
        std::rotate( res.begin(), res.begin() + 1, res.end() );
    }
    return res;
}

void gen_contacts_combinatorial_rec(const std::vector<std::string>& freeEffectors, const State& previous, T_ContactState& res, const std::size_t maxCreatedContacts,const bool maximiseContact = false)
{
    std::vector<std::vector<std::string> > allNewStates = stringCombinatorial(freeEffectors, maxCreatedContacts,maximiseContact);
    for(std::vector<std::vector<std::string> >::const_iterator cit = allNewStates.begin(); cit!=allNewStates.end();++cit)
    {
        ContactState contactState; contactState.first = previous; contactState.second = *cit;
        res.push(contactState);
    }
}

T_ContactState gen_contacts_combinatorial(const std::vector<std::string>& freeEffectors, const State& previous, const std::size_t maxCreatedContacts,const bool maximiseContacts)
{
    T_ContactState res;;
    gen_contacts_combinatorial_rec(freeEffectors, previous, res, maxCreatedContacts,maximiseContacts);
    return res;
}

T_ContactState gen_contacts_combinatorial(ContactGenHelper& contactGenHelper)
{
    State& cState = contactGenHelper.workingState_;
    std::vector<std::string> effNames(extractEffectorsName(contactGenHelper.fullBody_->GetLimbs()));
    const std::vector<std::string> freeLimbs = rbprm::freeEffectors(cState,effNames.begin(), effNames.end() );
    hppDout(notice,"in gen contact, number of free limbs : "<<freeLimbs.size());
    return gen_contacts_combinatorial(freeLimbs, cState, contactGenHelper.maxContactCreations_,contactGenHelper.maximiseContacts_);
}


ProjectionReport maintain_contacts(ContactGenHelper &contactGenHelper)
{
    hppDout(notice,"Begin maintain contact");
    ProjectionReport rep;
    Q_State& candidates = contactGenHelper.candidates_;
    hppDout(notice,"Get candidates");
    if(candidates.empty() && !contactGenHelper.workingState_.contacts_.empty()){
        hppDout(notice,"candidate list empty, gen combinatorial : ");
        candidates = maintain_contacts_combinatorial(contactGenHelper.workingState_,contactGenHelper.maxContactBreaks_);
    }
    else{
        hppDout(notice,"candidate list already generated, take the next one.");
        candidates.pop(); // first candidate already treated.
        }
    hppDout(notice,"candidates OK");
    while(!candidates.empty() && !rep.success_)
    {
        //retrieve latest state
        State cState = candidates.front();
        candidates.pop();
        hppDout(notice,"Project toRootConfiguration : ");
        rep = projectToRootConfiguration(contactGenHelper.fullBody_,contactGenHelper.workingState_.configuration_,cState);
        hppDout(notice,"maintain contacts, projection success : "<<rep.success_<<" for contacts : ");
        for(std::map<std::string,bool>::const_iterator cit = cState.contacts_.begin();cit!=cState.contacts_.end(); ++ cit)
        {
          hppDout(notice,"limb : "<<cit->first<<", contact = "<<cit->second);
        }
        if(rep.success_)
            rep = genColFree(contactGenHelper, rep);
        if(rep.success_)
        {
            //collision validation
            hpp::core::ValidationReportPtr_t valRep (new hpp::core::CollisionValidationReport);
            rep.success_ = contactGenHelper.fullBody_->GetCollisionValidation()->validate(rep.result_.configuration_, valRep);
            hppDout(notice,"maintain contact collision for config : r(["<<pinocchio::displayConfig(rep.result_.configuration_)<<"])");
            hppDout(notice,"valide  : "<<rep.success_);
            if(!rep.success_)
            {
              valRep->print( std::cout) ;
              std::cout << std::endl;
              hppDout(notice,"report = "<<*valRep);
            }
        }
        if(rep.success_){
            if(contactGenHelper.quasiStatic_ && contactGenHelper.testReachability_){
                reachability::Result resReachability = reachability::isReachable(contactGenHelper.fullBody_,contactGenHelper.workingState_,rep.result_);
                rep.success_ = resReachability.success();
                hppDout(notice,"Reachability test for maintain contact : succes = "<<rep.success_);
                rep.status_ = rep.success_ ? REACHABLE_CONTACT : STABLE_CONTACT;
                if (!rep.success_)
                    hppDout(notice,"NOT REACHABLE in maintain contact");
            }
        }
    }
    hppDout(notice,"maintain contact, check stability maintain : "<<contactGenHelper.checkStabilityMaintain_);
    if(rep.success_ && contactGenHelper.checkStabilityMaintain_)
        return maintain_contacts_stability(contactGenHelper, rep);
    return rep;
}


sampling::T_OctreeReport CollideOctree(const ContactGenHelper &contactGenHelper, const std::string& limbName,
                                                    RbPrmLimbPtr_t limb, const sampling::heuristic evaluate, const sampling::HeuristicParam & params)
{
    pinocchio::Transform3f transformpinocchio = limb->octreeRoot(); // get root transform from configuration
    fcl::Transform3f transform(transformpinocchio.rotation(),transformpinocchio.translation());
    std::vector<pinocchio::CollisionObjectPtr_t> affordances = getAffObjectsForLimb (limbName,contactGenHelper.affordances_, contactGenHelper.affFilters_);

    //#pragma omp parallel for
    // request samples which collide with each of the collision objects
    sampling::heuristic eval =  evaluate == 0 ? limb->evaluate_ : evaluate;
    std::size_t i (0);
    if (affordances.empty ())
      throw std::runtime_error ("No aff objects found!!!");

    std::vector<sampling::T_OctreeReport> reports(affordances.size());
    for(std::vector<pinocchio::CollisionObjectPtr_t>::const_iterator oit = affordances.begin();
        oit != affordances.end(); ++oit, ++i)
    {
        if(eval)
            sampling::GetCandidates(limb->sampleContainer_, transform, *oit, contactGenHelper.direction_, reports[i], params, eval);
        else
            sampling::GetCandidates(limb->sampleContainer_, transform, *oit, contactGenHelper.direction_, reports[i], params);
    }
    sampling::T_OctreeReport finalSet;
    // order samples according to EFORT
    for(std::vector<sampling::T_OctreeReport>::const_iterator cit = reports.begin();
        cit != reports.end(); ++cit)
    {
        finalSet.insert(cit->begin(), cit->end());
    }
    return finalSet;
}

hpp::rbprm::State findValidCandidate(const ContactGenHelper &contactGenHelper, const std::string& limbId,
                        RbPrmLimbPtr_t limb, core::CollisionValidationPtr_t validation, bool& found_sample,bool& found_stable,
                                     bool& unstableContact, const sampling::HeuristicParam & params, const sampling::heuristic evaluate = 0)
{
    State current = contactGenHelper.workingState_;
    current.stable = false;
    State intermediateState(current); // state before new contact creation
    State previous(contactGenHelper.previousState_); // previous state, before contact break
    sampling::T_OctreeReport finalSet = CollideOctree(contactGenHelper, limbId, limb, evaluate, params);
    core::Configuration_t moreRobust, bestUnreachable,configuration;
    configuration = current.configuration_;
    double maxRob = -std::numeric_limits<double>::max();
    sampling::T_OctreeReport::const_iterator it = finalSet.begin();
    fcl::Vec3f position, normal;
    fcl::Matrix3f rotation;
    ProjectionReport rep ;
    double robustness;
    bool isReachable;
    int evaluatedCandidates = 0;
    hppDout(notice,"in findValidCandidate for limb : "<<limbId);
    hppDout(notice,"number of candidate : "<<finalSet.size());
    for(;!found_sample && it!=finalSet.end(); ++it)
    {
        evaluatedCandidates++;
        const sampling::OctreeReport& bestReport = *it;
        hppDout(notice,"heuristic value = "<<it->value_);
        core::Configuration_t conf_before(configuration);
        sampling::Load(*bestReport.sample_, conf_before);
        hppDout(notice,"config before projection : r(["<<pinocchio::displayConfig(conf_before)<<"])");
        /*ProjectionReport */rep = projectSampleToObstacle(contactGenHelper.fullBody_, limbId, limb, bestReport, validation, configuration, current);
        hppDout(notice,"config : r(["<<pinocchio::displayConfig(configuration)<<"])");
        hppDout(notice,"projection to obstacle success = "<<rep.success_);
        if(rep.success_)
        {
            if(  !contactGenHelper.checkStabilityGenerate_ || (rep.result_.nbContacts == 1 && !contactGenHelper.stableForOneContact_)){ // only check projection, success !
                hppDout(notice,"Don't check stability : one contact or not the last contact");
                position = rep.result_.contactPositions_.at(limbId);
                rotation = rep.result_.contactRotation_.at(limbId);
                normal = rep.result_.contactNormals_.at(limbId);
                found_sample = true;
            }else{ // check stability and reachability
                robustness = stability::IsStable(contactGenHelper.fullBody_,rep.result_, contactGenHelper.acceleration_);
                hppDout(notice,"stability rob = "<<robustness);
                if( robustness>=contactGenHelper.robustnessTreshold_)
                {
                    hppDout(notice,"stability OK, test reachability : ");
                    if(contactGenHelper.testReachability_){
                        reachability::Result resReachability;
                        if(contactGenHelper.quasiStatic_){
                            resReachability = reachability::isReachable(contactGenHelper.fullBody_,intermediateState,rep.result_);
                        }else{
                            resReachability = reachability::isReachableDynamic(contactGenHelper.fullBody_,previous,rep.result_,contactGenHelper.tryQuasiStatic_,std::vector<double>(),contactGenHelper.reachabilityPointPerPhases_);
                        }
                        isReachable = resReachability.success();
                    }else{
                        isReachable = true;
                    }
                    if(isReachable){// reachable
                        position = rep.result_.contactPositions_.at(limbId);
                        rotation = rep.result_.contactRotation_.at(limbId);
                        normal = rep.result_.contactNormals_.at(limbId);
                        found_sample = true;
                        //TODO : save path (if not quasistatic)??
                    }else{
                        hppDout(notice,"NOT REACHABLE");
                        if(!found_stable){
                            maxRob = std::max(robustness, maxRob);
    // if no reachable state are found, we keep the first stable configuration found (ie. the one with the best heuristic score)
                            bestUnreachable = configuration;
                            found_stable = true;
                            position = rep.result_.contactPositions_.at(limbId);
                            rotation = rep.result_.contactRotation_.at(limbId);
                            normal = rep.result_.contactNormals_.at(limbId);
                        }
                      /*   // DEBUGING PURPOSE : call again evaluate on the sample found
                        hppDout(notice,"found sample, evaluate : "); // remove
                        Eigen::Vector3d eDir(contactGenHelper.direction_[0], contactGenHelper.direction_[1], contactGenHelper.direction_[2]); // remove
                        eDir.normalize();       // remove
                        hppDout(notice,"sample = "<<(*(bestReport.sample_)).startRank_); // remove
                        sampling::heuristic eval =  evaluate == 0 ? limb->evaluate_ : evaluate;// remove
                        (*eval)(*(bestReport.sample_), eDir, normal, params); // TODO : comment when not debugging
                        core::Configuration_t confBefore= current.configuration_; // remove
                        sampling::Load(*(bestReport.sample_),confBefore); //remove
                        hppDout(notice,"config before projection : r(["<<pinocchio::displayConfig(confBefore)<<"])"); //remove
                       */
                    }
                }
                // if no stable candidate is found, select best contact
                // anyway
                else if((robustness > maxRob) && contactGenHelper.contactIfFails_)
                {
                    hppDout(notice,"unstable contact, but the most robust yet.");
                    moreRobust = configuration;
                    maxRob = robustness;
                    position = rep.result_.contactPositions_.at(limbId);
                    rotation = rep.result_.contactRotation_.at(limbId);
                    normal = rep.result_.contactNormals_.at(limbId);
                    unstableContact = true;
                }
            }
        }
    }
    if(found_sample || found_stable || unstableContact)
    {
        current.contacts_[limbId] = true;
        current.contactNormals_[limbId] = normal;
        current.contactPositions_[limbId] = position;
        current.contactRotation_[limbId] = rotation;
        current.contactOrder_.push(limbId);
        hppDout(notice,"In find valid candidate, for limb : "<<limbId);
        hppDout(notice,"position : "<<position);
        hppDout(notice,"normal   : "<<normal);
    }
    if(found_sample)
    {
        hppDout(notice,"Valid sample found, stable and reachable");
        current.configuration_ = configuration;
        current.stable = true;
    }else if(found_stable){
        hppDout(notice,"Valid sample found, stable BUT NOT REACHABLE");
        current.configuration_ = bestUnreachable;
        current.stable = true;
    }else if(unstableContact)
    {
        hppDout(notice,"No valid sample found, take an unstable one");
        current.configuration_ = moreRobust;
        current.stable = false;
    }
    return current;
}

ProjectionReport generate_contact(const ContactGenHelper &contactGenHelper, const std::string& limbName, sampling::HeuristicParam & params,
                                  const sampling::heuristic evaluate)
{
    ProjectionReport rep;
    hppDout(notice,"in generate_contact, check stability = "<<contactGenHelper.checkStabilityGenerate_);
    RbPrmLimbPtr_t limb = contactGenHelper.fullBody_->GetLimbs().at(limbName);
    core::CollisionValidationPtr_t validation = contactGenHelper.fullBody_->GetLimbCollisionValidation().at(limbName);
    limb->limb_->robot()->currentConfiguration(contactGenHelper.workingState_.configuration_);
    limb->limb_->robot()->computeForwardKinematics ();
    // pick first sample which is collision free
    bool found_sample(false);
    bool found_stable(false);
    bool unstableContact(false); //set to true in case no stable contact is found
    params.comPath_=contactGenHelper.comPath_;
    params.currentPathId_ = contactGenHelper.currentPathId_;
    params.limbReferenceOffset_ = limb->effectorReferencePosition_;
    rep.result_ = findValidCandidate(contactGenHelper,limbName,limb, validation, found_sample,found_stable,unstableContact, params, evaluate);

    if(found_sample){
        hppDout(notice,"found reachable sample : "<<pinocchio::displayConfig(rep.result_.configuration_));
        rep.status_ =  REACHABLE_CONTACT;
        rep.success_ = true;
        #ifdef PROFILE
          RbPrmProfiler& watch = getRbPrmProfiler();
          watch.add_to_count("reachable contact", 1);
        #endif
    }
    else if(found_stable)
    {
        hppDout(notice,"NOT REACHABLE in generate_contact");
        hppDout(notice,"found stable sample : "<<pinocchio::displayConfig(rep.result_.configuration_));
        rep.status_ = STABLE_CONTACT;
        if(contactGenHelper.accept_unreachable_)
            rep.success_ = true;
        else
            rep.success_ = false;
        #ifdef PROFILE
          RbPrmProfiler& watch = getRbPrmProfiler();
          watch.add_to_count("unreachable stable contact", 1);
        #endif
    }
    else if(unstableContact)
    {
        hppDout(notice,"unstable contact");
        rep.status_ = UNSTABLE_CONTACT;
        rep.success_ = !contactGenHelper.checkStabilityGenerate_;
        #ifdef PROFILE
          RbPrmProfiler& watch = getRbPrmProfiler();
          watch.add_to_count("unstable contact", 1);
        #endif
    }
    else
    {
        hppDout(notice,"set Collision Free");
        rep =  setCollisionFree(contactGenHelper.fullBody_,validation,limbName,rep.result_);
        rep.status_ = NO_CONTACT;
        rep.success_ = false;
#ifdef PROFILE
  RbPrmProfiler& watch = getRbPrmProfiler();
  watch.add_to_count("no contact", 1);
#endif
    }
    return rep;
}

ProjectionReport gen_contacts(ContactGenHelper &contactGenHelper)
{
    ProjectionReport rep;
    T_ContactState candidates = gen_contacts_combinatorial(contactGenHelper);
    hppDout(notice,"gen_contact candidates size : "<<candidates.size());
    hppDout(notice,"working state config : r(["<<pinocchio::displayConfig(contactGenHelper.workingState_.configuration_)<<"])");
    bool checkStability(contactGenHelper.checkStabilityGenerate_);
    while(!candidates.empty() && !rep.success_)
    {
        //retrieve latest state
        ContactState cState = candidates.front();
        candidates.pop();
        hppDout(notice,"generateContact, number of limbs to test   : "<<cState.second.size());
        if(cState.second.empty() && checkStability && (contactGenHelper.workingState_.nbContacts >= 2 || contactGenHelper.stableForOneContact_)){
          hppDout(notice,"List of free limbs empty in gen_contact, check stability for workingState with contact maintained");
          double robustness = stability::IsStable(contactGenHelper.fullBody_,contactGenHelper.workingState_, contactGenHelper.acceleration_);
          hppDout(notice,"stability rob = "<<robustness);
          if(robustness >= contactGenHelper.robustnessTreshold_){
            contactGenHelper.workingState_.stable=true;
          }
        }
        if(cState.second.empty() && (contactGenHelper.workingState_.stable || !checkStability))
        {
            if(contactGenHelper.workingState_.nbContacts >= 2)
            {
                hppDout(notice,"working state stable, contact maintained OK.");
                rep.result_ = contactGenHelper.workingState_;
                rep.status_ = STABLE_CONTACT;
                rep.success_ = true;
                return rep;
            }
        }
        contactGenHelper.checkStabilityGenerate_ = false; // stability not mandatory before last contact is created
        for(std::vector<std::string>::const_iterator cit = cState.second.begin();
            cit != cState.second.end(); ++cit)
        {
            hppDout(notice,"Try to generate contact for limb : "<<*cit);
            sampling::HeuristicParam params;
            params.contactPositions_ = cState.first.contactPositions_;
            contactGenHelper.fullBody_->device_->currentConfiguration(cState.first.configuration_);
            contactGenHelper.fullBody_->device_->computeForwardKinematics();
            params.comPosition_ = contactGenHelper.fullBody_->device_->positionCenterOfMass();
            int cfgSize(cState.first.configuration_.rows());
            params.comSpeed_ = fcl::Vec3f(cState.first.configuration_[cfgSize-6], cState.first.configuration_[cfgSize-5], cState.first.configuration_[cfgSize-4]);
            params.comAcceleration_ = contactGenHelper.acceleration_;
            params.sampleLimbName_ = *cit;
            params.tfWorldRoot_ = fcl::Transform3f();
            params.tfWorldRoot_.setTranslation(fcl::Vec3f(cState.first.configuration_[0],cState.first.configuration_[1],cState.first.configuration_[2]));
            params.tfWorldRoot_.setQuatRotation(fcl::Quaternion3f(cState.first.configuration_[3],cState.first.configuration_[4],cState.first.configuration_[5],cState.first.configuration_[6]));

            /*if(cit+1 == cState.second.end()) // DEBUG STABILITY
                contactGenHelper.checkStabilityGenerate_ = checkStability;*/
            rep = generate_contact(contactGenHelper,*cit, params);
            if(rep.success_)
            {
                contactGenHelper.workingState_ = rep.result_;
            }
            //else
            //    break;
        }
    }
    contactGenHelper.checkStabilityGenerate_=checkStability;
    return rep;
}

projection::ProjectionReport repositionContacts(ContactGenHelper& helper)
{
    ProjectionReport resultReport;
    State result = helper.workingState_;
    result.stable = false;
    State previous = result;
    // replace existing contacts
    // start with older contact created
    std::stack<std::string> poppedContacts;
    std::queue<std::string> oldOrder = result.contactOrder_;
    std::queue<std::string> newOrder;
    std::string nContactName ="";
    core::Configuration_t savedConfig = helper.previousState_.configuration_;
    core::Configuration_t config = savedConfig;
    while(!result.stable &&  !oldOrder.empty())
    {
        std::string previousContactName = oldOrder.front();
        std::string groupName = helper.fullBody_->GetLimbs().at(previousContactName)->limb_->name();
        const std::vector<std::string>& group = helper.fullBody_->GetGroups().at(groupName);
        oldOrder.pop();
        core::ConfigurationIn_t save = helper.fullBody_->device_->currentConfiguration();
        bool notFound(true);
        for(std::vector<std::string>::const_iterator cit = group.begin();
            notFound && cit != group.end(); ++cit)
        {
            result.RemoveContact(*cit);
            helper.workingState_ = result;

            sampling::HeuristicParam params;
            params.contactPositions_ = helper.workingState_.contactPositions_;
            helper.fullBody_->device_->currentConfiguration(result.configuration_);
            helper.fullBody_->device_->computeForwardKinematics();
            params.comPosition_ = helper.fullBody_->device_->positionCenterOfMass();
            int cfgSize(helper.workingState_.configuration_.rows());
            params.comSpeed_ = fcl::Vec3f(helper.workingState_.configuration_[cfgSize-6], helper.workingState_.configuration_[cfgSize-5], helper.workingState_.configuration_[cfgSize-4]);
            params.comAcceleration_ = helper.acceleration_;
            params.sampleLimbName_ = *cit;
            params.tfWorldRoot_ = fcl::Transform3f();
            params.tfWorldRoot_.setTranslation(fcl::Vec3f(helper.workingState_.configuration_[0],helper.workingState_.configuration_[1],helper.workingState_.configuration_[2]));
            params.tfWorldRoot_.setQuatRotation(fcl::Quaternion3f(helper.workingState_.configuration_[3],helper.workingState_.configuration_[4],helper.workingState_.configuration_[5],helper.workingState_.configuration_[6]));

            projection::ProjectionReport rep = contact::generate_contact(helper,*cit, params);
            if(rep.status_ == STABLE_CONTACT || REACHABLE_CONTACT)
            {
                nContactName = *cit;
                notFound = false;
                result = rep.result_;
            }
            else
            {
                result = previous;
                config = savedConfig;
            }
        }
        if(notFound)
        {
            config = savedConfig;
            result.configuration_ = savedConfig;
            poppedContacts.push(previousContactName);
            helper.fullBody_->device_->currentConfiguration(save);
        }
    }
    while(!poppedContacts.empty())
    {
        newOrder.push(poppedContacts.top());
        poppedContacts.pop();
    }
    while(!oldOrder.empty())
    {
        newOrder.push(oldOrder.front());
        oldOrder.pop();
    }
    if(result.stable)
    {
        newOrder.push(nContactName);
        resultReport.status_ = STABLE_CONTACT;
        resultReport.success_ = true;
    }
    result.contactOrder_ = newOrder;
    resultReport.result_ = result;
    return resultReport;
}

} // namespace projection
} // namespace rbprm
} // namespace hpp
