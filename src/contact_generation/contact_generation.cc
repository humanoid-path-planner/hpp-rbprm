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
#include <hpp/rbprm/tools.hh>



namespace hpp {
namespace rbprm {
namespace contact{


ContactGenHelper::ContactGenHelper(RbPrmFullBodyPtr_t fb, const State& ps, model::ConfigurationIn_t configuration,
                                    const double robustnessTreshold,
                                    const std::size_t maxContactBreaks,
                                    const bool checkStability,
                                    const fcl::Vec3f& acceleration)
: fullBody_(fb)
, previousState_(ps)
, checkStability_(checkStability)
, acceleration_(acceleration)
, robustnessTreshold_(robustnessTreshold)
, candidates_(maintain_contacts_combinatorial(ps, maxContactBreaks))
, targetRootConfiguration_(configuration)
{}

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

void maintain_contacts_combinatorial_rec(const hpp::rbprm::State& currentState, const std::size_t  depth,
                                         const std::size_t maxBrokenContacts, T_DepthState& res)
{
    if (!push_if_new(res[depth], currentState) || depth>=maxBrokenContacts) return;
    std::queue<std::string> contactOrder = currentState.contactOrder_;
    while(!contactOrder.empty())
    {
        hpp::rbprm::State copyState = currentState;
        const std::string contactRemoved = contactOrder.front();
        copyState.RemoveContact(contactRemoved);
        contactOrder.pop();
        maintain_contacts_combinatorial_rec(copyState, depth+1, maxBrokenContacts, res);
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
    maintain_contacts_combinatorial_rec(currentState, 0, maxBrokenContacts,res);
    return flatten(res);
}

using namespace projection;

bool maintain_contacts_stability_rec(hpp::rbprm::RbPrmFullBodyPtr_t fullBody,
                        model::ConfigurationIn_t targetRootConfiguration,
                        Q_State& candidates,const std::size_t contactLength,
                        const fcl::Vec3f& acceleration, const double robustness,
                        ProjectionReport& currentRep)
{
    if(stability::IsStable(fullBody,currentRep.result_) > robustness) return true;
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


void pState(const State& state)
{
    std::cout << "**** active contacts ****" << std::endl;
    std::queue<std::string> queue = state.contactOrder_;
    while(!queue.empty())
    {
        std::string c = queue.front();
        queue.pop();
        std::cout << c << std::endl;
    }
    std::cout << "**** end active contacts ****" << std::endl;
}
ProjectionReport maintain_contacts_stability(ContactGenHelper &contactGenHelper, ProjectionReport& currentRep)
{
    const std::size_t contactLength(currentRep.result_.contactOrder_.size());
    maintain_contacts_stability_rec(contactGenHelper.fullBody_,
                                    contactGenHelper.targetRootConfiguration_,
                                    contactGenHelper.candidates_,
                                    contactLength, contactGenHelper.acceleration_,
                                    contactGenHelper.robustnessTreshold_, currentRep);
    return currentRep;
}

ProjectionReport genColFree(ContactGenHelper &contactGenHelper, ProjectionReport& currentRep)
{
    ProjectionReport res = currentRep;
    // identify broken limbs and find collision free configurations for each one of them.
    std::vector<std::string> brokenContacts(currentRep.result_.contactBreaks(contactGenHelper.previousState_));
    for(std::vector<std::string>::const_iterator cit = brokenContacts.begin(); cit != brokenContacts.end() && res.success_; ++cit)
        res = projection::setCollisionFree(contactGenHelper.fullBody_,contactGenHelper.fullBody_->GetCollisionValidation(),*cit,res.result_);
    return res;
}

ProjectionReport maintain_contacts(ContactGenHelper &contactGenHelper)
{
    ProjectionReport rep;
    Q_State& candidates = contactGenHelper.candidates_;
    while(!candidates.empty() && !rep.success_)
    {
        //retrieve latest state
        State cState = candidates.front();
        candidates.pop();
        rep = projectToRootConfiguration(contactGenHelper.fullBody_,contactGenHelper.targetRootConfiguration_,cState);
        if(rep.success_)
            rep = genColFree(contactGenHelper, rep);
    }
    if(rep.success_ && contactGenHelper.checkStability_)
        return maintain_contacts_stability(contactGenHelper, rep);
    return rep;
}

std::vector<std::string> extractEffectorsName(const rbprm::T_Limb& limbs)
{
    std::vector<std::string> res;
    for(rbprm::T_Limb::const_iterator cit = limbs.begin(); cit != limbs.end(); ++cit)
        res.push_back(cit->first);
    return res;
}

void print_string_vect(const std::vector<std::string>& strings)
{
    for(std::vector<std::string>::const_iterator cit = strings.begin(); cit != strings.end(); ++cit)
    {
        std::cout << *cit <<  std::endl;
    }
}

void print_string_string_vec(std::vector<std::vector<std::string> >& strings)
{
    for(std::vector<std::vector<std::string> >::const_iterator cit = strings.begin(); cit != strings.end(); ++cit)
    {
        std::cout << "***one string vector ***" << std::endl;
        print_string_vect(*cit);
        std::cout << "***one string vector ***" << std::endl;
    }
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


std::vector<std::vector<std::string> > stringCombinatorial(const State& previous, const std::vector<std::string>& candidates, const std::size_t maxDepth)
{
    std::vector<std::vector<std::string> > res;
    res.push_back(previous.fixedContacts(previous));
    stringCombinatorialRec(res, candidates, maxDepth);
    return res;
}

void gen_contacts_combinatorial_rec(const std::vector<std::string>& freeEffectors, const State& previous, T_ContactState& res, const std::size_t maxCreatedContacts)
{
    std::vector<std::vector<std::string> > allNewStates = stringCombinatorial(previous, freeEffectors, maxCreatedContacts);
    for(std::vector<std::vector<std::string> >::const_iterator cit = allNewStates.begin(); cit!=allNewStates.end();++cit)
    {
        ContactState contactState; contactState.first = previous; contactState.second = *cit;
        res.push(contactState);
    }
}

T_ContactState gen_contacts_combinatorial(const std::vector<std::string>& freeEffectors, const State& previous, const std::size_t maxCreatedContacts)
{
    T_ContactState res;;
    gen_contacts_combinatorial_rec(freeEffectors, previous, res, maxCreatedContacts);
    return res;
}

T_ContactState gen_contacts_combinatorial(ContactGenHelper& contactGenHelper, const std::size_t maxCreatedContacts)
{
    State cState = contactGenHelper.candidates_.front();
    contactGenHelper.candidates_.pop();
    const std::vector<std::string> freeLimbs = rbprm::freeEffectors(contactGenHelper.previousState_, extractEffectorsName(contactGenHelper.fullBody_->GetLimbs()));
    return gen_contacts_combinatorial(freeLimbs, cState, maxCreatedContacts);
}

} // namespace projection
} // namespace rbprm
} // namespace hpp
