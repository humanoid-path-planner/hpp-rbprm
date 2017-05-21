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
#include <hpp/rbprm/contact_generation/algorithm.hh>
#include <hpp/rbprm/stability/stability.hh>
#include <hpp/rbprm/tools.hh>



namespace hpp {
namespace rbprm {
namespace contact{

ContactReport::ContactReport()
    : projection::ProjectionReport()
    , contactMaintained_(false)
    , multipleBreaks_(false)
    , contactCreated_(false)
    , repositionedInPlace_(false)
{
    // NOTHING
}

ContactReport::ContactReport(const projection::ProjectionReport& parent)
    : projection::ProjectionReport(parent)
    , contactMaintained_(false)
    , multipleBreaks_(false)
    , contactCreated_(false)
    , repositionedInPlace_(false)
{
    // NOTHING
}

ContactReport generateContactReport(const projection::ProjectionReport& parent, const ContactGenHelper& helper, bool repositionedInPlace=false)
{
    const State& previous = helper.previousState_;
    const State& result = parent.result_;
    ContactReport report(parent) ;
    report.contactCreated_ = (result.fixedContacts(previous).size() == previous.nbContacts);
    report.multipleBreaks_ = (result.contactBreaks(previous).size() > helper.maxContactBreaks_);
    report.repositionedInPlace_ = repositionedInPlace;
    report.contactMaintained_ = !repositionedInPlace && !(result.contactCreations(previous).size() > 0);
    return report;
}

projection::ProjectionReport genContactFromOneMaintainCombinatorial(ContactGenHelper& helper)
{
    // retrieve the first feasible result of maintain combinatorial...
    projection::ProjectionReport rep = contact::maintain_contacts(helper);
    if(rep.success_)
    {
        // ... if found, then try to generate feasible contact for this combinatorial.
        helper.workingState_ = rep.result_;
        return gen_contacts(helper);
    }
    else
    {
        std::cout << "can t maintain" << ! rep.success_ << std::endl;
    }
    return rep;
}

// if contact generation failed, tries to reposition the contacts without moving the root
ContactReport handleFailure(ContactGenHelper& helper)
{
    helper.workingState_ = helper.previousState_;
    projection::ProjectionReport rep = repositionContacts(helper);

    std::vector<std::string> breaks = rep.result_.contactBreaks(helper.previousState_);
    std::vector<std::string> creations = rep.result_.contactCreations(helper.previousState_);
    std::cout << "AFTER REPOSITIONNING " << rep.success_ << std::endl;
    {
        std::cout << "\tREMOVING CONTACT " << breaks.size() << std::endl;
        for(std::vector<std::string>::const_iterator cit = breaks.begin(); cit != breaks.end(); ++cit)
            std::cout << "\t \t " << *cit << std::endl;
        std::cout << "\tCREATING CONTACT " << creations.size() << std::endl;
        for(std::vector<std::string>::const_iterator cit = creations.begin(); cit != creations.end(); ++cit)
            std::cout << "\t \t " << *cit << std::endl;
    }
    std::cout << "END AFTER REPOSITIONNING "  << std::endl;
    return generateContactReport(rep,helper,true);
}

ContactReport oneStep(ContactGenHelper& helper)
{
    std::cout << "*********************** ONE STEP PPPPPPPPPPP *************** " << std::endl;
    projection::ProjectionReport rep;
    do
        rep = genContactFromOneMaintainCombinatorial(helper);
    while(!rep.success_ && !helper.candidates_.empty());
    if(!rep.success_) // TODO only possible in quasi static
    {
        std::cout << "*********************** ONE STEP REPOR *************** " << std::endl;
        return handleFailure(helper);
    }
    std::cout << "*********************** END STEP *************** " << std::endl;
    return generateContactReport(rep,helper);
}

} // namespace projection
} // namespace rbprm
} // namespace hpp
