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



namespace hpp {
namespace rbprm {
namespace contact{


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

void maintain_contacts_combinatorial_rec(const hpp::rbprm::State& currentState, const unsigned int depth,
                                         const unsigned int maxBrokenContacts, T_DepthState& res)
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

Q_State maintain_contacts_combinatorial(const hpp::rbprm::State& currentState, const unsigned int maxBrokenContacts)
{
    T_DepthState res(maxBrokenContacts+1);
    maintain_contacts_combinatorial_rec(currentState, 0, maxBrokenContacts,res);
    return flatten(res);
}

using namespace projection;

bool maintain_contacts_stability_rec(hpp::rbprm::RbPrmFullBodyPtr_t fullBody,
                        model::ConfigurationIn_t targetRootConfiguration,
                        Q_State& candidates,const std::size_t contactLength,
                        const fcl::Vec3f& acceleration, ProjectionReport& currentRep)
{
    if(stability::IsStable(fullBody,currentRep.result_)) return true;
    if(!candidates.empty())
    {
        State cState = candidates.front();
        candidates.pop();
         // removed more contacts, cannot be stable if previous state was not
        if(cState.contactOrder_.size() < contactLength) return false;
        ProjectionReport rep = projectToRootConfiguration(fullBody,targetRootConfiguration,cState);
        Q_State copy_candidates = candidates;
        if(maintain_contacts_stability_rec(fullBody,targetRootConfiguration,copy_candidates,contactLength,acceleration, rep))
        {
            currentRep = rep;
            candidates = copy_candidates;
            return true;
        }
    }
    return false;
}

ProjectionReport maintain_contacts_stability(ContactGenHelper &contactGenHelper, ProjectionReport& currentRep)
{
    const std::size_t contactLength(currentRep.result_.contactOrder_.size());
    maintain_contacts_stability_rec(contactGenHelper.fullBody_,
                                    contactGenHelper.targetRootConfiguration_,
                                    contactGenHelper.candidates_,
                                    contactLength, contactGenHelper.acceleration_, currentRep);
    return currentRep;
}

ProjectionReport genColFree(ContactGenHelper &contactGenHelper, ProjectionReport& currentRep)
{
    ProjectionReport res = currentRep;
    // identify broken limbs and find collision free configurations for each one of them.
    std::vector<std::string> brokenContacts(currentRep.result_.contactBreaks(contactGenHelper.previousState_));
    for(std::vector<std::string>::const_iterator cit = brokenContacts.begin(); cit != brokenContacts.end() && res.success_; ++cit)
        res = projection::setCollisionFree(contactGenHelper.fullBody_,*cit,res.result_);
    return res;
}

ProjectionReport maintain_contacts(ContactGenHelper &contactGenHelper)
{
    ProjectionReport rep;
    Q_State& candidates = contactGenHelper.candidates_;
    while(!contactGenHelper.candidates_.empty() || rep.success_)
    {
        //retrieve latest state
        State cState = candidates.front();
        candidates.pop();
        rep = projectToRootConfiguration(contactGenHelper.fullBody_,contactGenHelper.targetRootConfiguration_,cState);
        if(rep.success_)
            rep = genColFree(contactGenHelper, rep);
    }
    if(rep.success_ && contactGenHelper.checkStability_)
    {
        return maintain_contacts_stability(contactGenHelper, rep);
    }
    return rep;
}

} // namespace projection
} // namespace rbprm
} // namespace hpp
