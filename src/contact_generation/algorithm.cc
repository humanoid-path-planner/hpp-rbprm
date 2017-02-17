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
    return rep;
}

projection::ProjectionReport oneStep(ContactGenHelper& helper)
{
    projection::ProjectionReport rep;
    do
    {
        rep = genContactFromOneMaintainCombinatorial(helper);
    }
    while(!rep.success_ && !helper.candidates_.empty());
    std::cout << "yay FINAL??? " << rep.success_ << " stable " << rep.result_.stable << std::endl;
    return rep;
}

} // namespace projection
} // namespace rbprm
} // namespace hpp
