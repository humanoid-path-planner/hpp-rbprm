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

#include <hpp/rbprm/rbprm-state.hh>

namespace hpp {
namespace rbprm {

State::State(const State& other)
    : configuration_(other.configuration_),
      contactOrder_(other.contactOrder_),
      nbContacts(other.nbContacts),
      stable(other.stable),
      robustness(other.robustness) {
  contacts_ = (other.contacts_);
  contactNormals_ = (other.contactNormals_);
  contactPositions_ = (other.contactPositions_);
  contactRotation_ = (other.contactRotation_);
}

State& State::operator=(const State& other) {
  if (this != &other)  // protect against invalid self-assignment
  {
    contacts_ = other.contacts_;
    contactNormals_ = other.contactNormals_;
    contactPositions_ = other.contactPositions_;
    contactRotation_ = other.contactRotation_;
    contactOrder_ = other.contactOrder_;
    nbContacts = other.nbContacts;
    stable = other.stable;
    configuration_ = other.configuration_;
  }
  // by convention, always return *this
  return *this;
}

bool State::RemoveContact(const std::string& contactId) {
  if (contacts_.erase(contactId)) {
    contactNormals_.erase(contactId);
    contactPositions_.erase(contactId);
    contactRotation_.erase(contactId);
    contactNormals_.erase(contactId);
    --nbContacts;
    stable = false;
    std::queue<std::string> newQueue;
    std::string currentContact;
    while (!contactOrder_.empty()) {
      currentContact = contactOrder_.front();
      contactOrder_.pop();
      if (contactId != currentContact) {
        newQueue.push(currentContact);
      }
    }
    contactOrder_ = newQueue;
    return true;
  }
  return false;
}

std::string State::RemoveFirstContact() {
  if (contactOrder_.empty()) return "";
  std::string contactId = contactOrder_.front();
  contactOrder_.pop();
  contacts_.erase(contactId);
  contactNormals_.erase(contactId);
  contactPositions_.erase(contactId);
  contactRotation_.erase(contactId);
  contactNormals_.erase(contactId);
  stable = false;
  --nbContacts;
  return contactId;
}

void State::contactCreations(const State& previous,
                             std::vector<std::string>& outList) const {
  for (std::map<std::string, fcl::Vec3f>::const_iterator cit =
           contactPositions_.begin();
       cit != contactPositions_.end(); ++cit) {
    const std::string& name = cit->first;
    bool newContact(true);
    if (previous.contactPositions_.find(name) !=
        previous.contactPositions_.end()) {
      newContact =
          (previous.contactPositions_.at(name) - cit->second).norm() > 0.01;
    }
    if (newContact &&
        std::find(outList.begin(), outList.end(), name) == outList.end()) {
      outList.push_back(name);
    }
  }
}

void State::contactBreaks(const State& previous,
                          std::vector<std::string>& outList) const {
  previous.contactCreations(*this, outList);
}

std::vector<std::string> State::contactBreaks(const State& previous) const {
  std::vector<std::string> res;
  contactBreaks(previous, res);
  return res;
}

std::vector<std::string> State::contactCreations(const State& previous) const {
  std::vector<std::string> res;
  contactCreations(previous, res);
  return res;
}

namespace {
// Given known contact variations, computes all effector that were not in
// contacts
std::vector<std::string> freeLimbMotions(
    const std::vector<std::string>& allEffectors,
    const std::vector<std::string>& contactVariations, const State& current) {
  std::vector<std::string> res;
  for (std::vector<std::string>::const_iterator cit = allEffectors.begin();
       cit != allEffectors.end(); ++cit) {
    if (std::find(contactVariations.begin(), contactVariations.end(), *cit) ==
            contactVariations.end() &&
        !current.contacts_.at(*cit)) {
      res.push_back(*cit);
    }
  }
  return res;
}
}  // namespace

std::vector<std::string> State::freeVariations(
    const State& previous, const std::vector<std::string>& allEffectors) const {
  return freeLimbMotions(allEffectors, contactVariations(previous), *this);
}

std::vector<std::string> State::contactVariations(const State& previous) const {
  std::vector<std::string> res;
  contactCreations(previous, res);
  contactBreaks(previous, res);
  return res;
}

std::vector<std::string> State::fixedContacts(const State& previous) const {
  std::vector<std::string> res;
  std::vector<std::string> variations = contactVariations(previous);
  for (std::map<std::string, fcl::Vec3f>::const_iterator cit =
           contactPositions_.begin();
       cit != contactPositions_.end(); ++cit) {
    const std::string& name = cit->first;
    if (std::find(variations.begin(), variations.end(), name) ==
        variations.end()) {
      res.push_back(name);
    }
  }
  return res;
}

std::vector<std::string> State::allVariations(
    const State& previous, const std::vector<std::string>& allEffectors) const {
  std::vector<std::string> res;
  std::vector<std::string> fixedContacts = this->fixedContacts(previous);
  for (std::vector<std::string>::const_iterator cit = allEffectors.begin();
       cit != allEffectors.end(); ++cit) {
    if (std::find(fixedContacts.begin(), fixedContacts.end(), *cit) ==
        fixedContacts.end()) {
      res.push_back(*cit);
    }
  }
  return res;
}

void State::print() const {
  std::cout << " State " << std::endl;
  /*std::cout << " \t Configuration " << std::endl;
  for(int i = 0; i< configuration_.rows(); ++i)
  {
    std::cout << configuration_[i] << " ";
  }
  std::cout << std::endl;*/

  std::cout << " \t contacts " << std::endl;
  for (std::map<std::string, bool>::const_iterator cit = contacts_.begin();
       cit != contacts_.end(); ++cit) {
    std::cout << cit->first << ": " << cit->second << std::endl;
  }

  std::cout << "\t stable " << this->stable << std::endl;
  std::cout << "\t robustness " << this->robustness << std::endl;

  /*std::cout << " \t positions " << std::endl;
  for(std::map<std::string, fcl::Vec3f>::const_iterator cit =
    contactPositions_.begin(); cit != contactPositions_.end(); ++cit)
  {
    std::cout << cit->first << ": " <<  cit->second << std::endl;
  }*/
  /*std::cout << " \t contactNormals_ " << std::endl;
  for(std::map<std::string, fcl::Vec3f>::const_iterator cit =
    contactNormals_.begin(); cit != contactNormals_.end(); ++cit)
  {
    std::cout << cit->first << ": " <<  cit->second << std::endl;
  }
  std::cout << std::endl;*/
}

void State::printInternal(std::stringstream& ss) const {
  std::map<std::string, fcl::Vec3f>::const_iterator cit =
      contactNormals_.begin();
  for (unsigned int c = 0; c < nbContacts; ++c, ++cit) {
    const std::string& name = cit->first;
    const fcl::Vec3f& position = contactPositions_.at(name);
    const fcl::Matrix3f& rotation = contactRotation_.at(name);
    ss << name.substr(1);
    for (std::size_t i = 0; i < 3; ++i) {
      ss << " " << position[i];
    }
    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = 0; j < 3; ++j) {
        ss << " " << rotation(i, j);
      }
    }
    ss << "\n";
  }
  ss << "configuration ";
  for (int i = 0; i < configuration_.rows(); ++i) {
    ss << " " << configuration_[i];
  }
  ss << "\n \n";
}

void State::print(std::stringstream& ss) const {
  ss << nbContacts << "\n";
  ss << "";
  std::map<std::string, fcl::Vec3f>::const_iterator cit =
      contactNormals_.begin();
  for (unsigned int c = 0; c < nbContacts; ++c, ++cit) {
    ss << " " << cit->first << " ";
  }
  ss << "\n";
  printInternal(ss);
}

void State::print(std::stringstream& ss, const State& previous) const {
  ss << nbContacts << "\n";
  std::vector<std::string> ncontacts;
  ss << "";
  for (std::map<std::string, fcl::Vec3f>::const_iterator cit =
           contactPositions_.begin();
       cit != contactPositions_.end(); ++cit) {
    const std::string& name = cit->first;
    bool newContact(true);
    if (previous.contactPositions_.find(name) !=
        previous.contactPositions_.end()) {
      newContact =
          (previous.contactPositions_.at(name) - cit->second).norm() > 0.01;
    }
    if (newContact) {
      ncontacts.push_back(name);
      ss << name.substr(1) << " ";
    }
  }
  ss << "\n";
  /*ss << "broken Contacts: ";
  for(std::map<std::string, fcl::Vec3f>::const_iterator cit =
  previous.contactPositions_.begin(); cit != previous.contactPositions_.end();
  ++cit)
  {
      const std::string& name = cit->first;
      if(contactPositions_.find(name) == contactPositions_.end())
      {
          ss << name << " ";
      }
  }
  ss << "\n";*/
  printInternal(ss);
  /*for(std::vector<std::string>::const_iterator cit = ncontacts.begin();
      cit != ncontacts.end(); ++cit)
  {
      const std::string& name = *cit;
      const fcl::Vec3f& normal = contactNormals_.at(name);
      const fcl::Vec3f& position = contactPositions_.at(name);
      ss << " " << name <<": ";
      for(std::size_t i=0; i<3; ++i)
      {
          ss << " " << position[i];
      }
      for(std::size_t i=0; i<3; ++i)
      {
          ss << " " << normal[i];
      }
      ss << "\n";
  }
  for(int i=0; i<configuration_.rows(); ++i)
  {
      ss << " " << configuration_[i];
  }
  ss << "\n \n";*/
}

pinocchio::value_type effectorDistance(const State& from, const State& to) {
  std::vector<std::string> variations = to.contactCreations(from);
  pinocchio::value_type norm = 0.;
  for (std::vector<std::string>::const_iterator cit = variations.begin();
       cit != variations.end(); ++cit) {
    std::string name = *cit;
    if (from.contactPositions_.find(name) != from.contactPositions_.end()) {
      norm = std::max(norm, (from.contactPositions_.at(name) -
                             to.contactPositions_.at(name))
                                .norm());
    }
  }
  return norm;
}

}  // namespace rbprm
}  // namespace hpp
