//
// Copyright (c) 2014 CNRS
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
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_RBPRM_STATE_HH
# define HPP_RBPRM_STATE_HH

# include <hpp/rbprm/config.hh>
# include <hpp/pinocchio/device.hh>
# include <hpp/rbprm/rbprm-limb.hh>

# include <queue>
# include <algorithm>

namespace hpp {
namespace rbprm {
struct State;
typedef std::vector<State> T_State;
typedef T_State::const_iterator CIT_State;
typedef std::pair<pinocchio::value_type, rbprm::State> StateFrame;
typedef std::vector<StateFrame> T_StateFrame;
typedef T_StateFrame::const_iterator CIT_StateFrame;

    /// Helper class that maintains active contacts at a given state, as well as their locations
    /// can be used to determine contact transition wrt a previous State
    struct HPP_RBPRM_DLLAPI State{
        State():nbContacts(0), stable(false){}
        State(const State& other);
       ~State(){}

        State& operator= (const State& other);

        /// Removes an active contact from the State
        ///
        /// \param contactId name of the contact to remove
        /// \return true whether the contact was indeed active
        bool RemoveContact(const std::string& contactId);

        /// Removes the first active contact created
        /// in the state.Contact order is maintained in a queue
        ///
        /// \return empty string if no contact was active, otherwise
        /// the id of the removed contact
        std::string RemoveFirstContact();

        /// Given a antecedent State, computes the list of contact changes (creations an destructions)
        ///
        /// \return the list of all modified contacts between two States
        std::vector<std::string> contactVariations(const State& previous) const;

        /// Given an antecedent State and a list of effectors, computes the list of effectors
        /// that were not in contact in any of the two states
        ///
        /// \return the list of all modified contacts between two States
        std::vector<std::string> freeVariations(const State& previous, const std::vector<std::string>& allEffectors) const;

        /// Given an antecedent State and a list of effectors, computes the list of
        /// all the effectors that moved between the two States (ie contact was not maintained)
        ///
        /// \return the list of all modified effectors between two States
        std::vector<std::string> allVariations(const State& previous, const std::vector<std::string>& allEffectors) const;

        /// Given a antecedent State, computes the list of Contacts that were maintained between the two States (both
        /// active at the same location)
        ///
        /// \return the list of all preserved contacts between two States
        std::vector<std::string> fixedContacts(const State& previous) const;

        /// Given a antecedent State, computes the list of Contacts that were created between the two States
        ///
        /// \return the list of all created contacts between two States
        void contactCreations(const State& previous, std::vector<std::string>& outList) const;

        /// Given a antecedent State, computes the list of Contacts that were created between the two States
        ///
        /// \return the list of all created contacts between two States
        std::vector<std::string> contactCreations(const State& previous) const;

        /// Given a antecedent State, computes the list of Contacts that were broken between the two States
        ///
        /// \return the list of all broken contacts between two States
        std::vector<std::string> contactBreaks(const State& previous) const;

        /// Given a antecedent State, computes the list of Contacts that were broken between the two States
        ///
        /// \return the list of all broken contacts between two States
        void contactBreaks(const State& previous, std::vector<std::string>& outList) const;

        void print() const;
        void print(std::stringstream& ss) const;
        void print(std::stringstream& ss, const State& previous) const;
        void printInternal(std::stringstream& ss) const;


        hpp::pinocchio::Configuration_t configuration_;
        std::map<std::string, bool> contacts_;
        std::map<std::string, fcl::Vec3f> contactNormals_;
        std::map<std::string, fcl::Vec3f> contactPositions_;
        std::map<std::string, fcl::Matrix3f> contactRotation_;
        std::queue<std::string> contactOrder_;
        std::size_t nbContacts;
        bool stable;
        double robustness;
    }; // struct State
    /// Given two State, compute the contact effectors distance travelled
    /// between two states
    HPP_RBPRM_DLLAPI pinocchio::value_type effectorDistance(const State& from, const State& to);


    /// Given a State and a list of effectors, computes the list of
    /// all the effectors that moved between the two States (ie contact was not maintained)
    ///
    /// \return the list of all modified effectors between two States
    template<typename Iter>
    HPP_RBPRM_DLLAPI std::vector<std::string> freeEffectors(const State& state, Iter start, Iter end)
    {
        std::vector<std::string> res;
        for(Iter it = start; it != end; ++it)
        {
            const std::string& eff = *it;
            std::map<std::string, bool>::const_iterator cit = state.contacts_.find(eff);
            if(cit == state.contacts_.end() || !cit->second)
            {
                res.push_back(eff);
            }
        }
        return res;
    }
  } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_STATE_HH
