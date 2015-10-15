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
# include <hpp/model/device.hh>
# include <hpp/rbprm/rbprm-limb.hh>
# include <queue>
namespace hpp {
  namespace rbprm {
  struct HPP_RBPRM_DLLAPI State{
      State():nbContacts(0), stable(false){}
      State(const State& other)
          : configuration_(other.configuration_)
          , com_(other.com_)
          , contactOrder_(other.contactOrder_)
          , nbContacts(other.nbContacts)
          , stable(other.stable)
      {
          contacts_= (other.contacts_);
          contactNormals_ = (other.contactNormals_);
          contactPositions_ = (other.contactPositions_);
          contactRotation_ = (other.contactRotation_);
      }

        hpp::model::Configuration_t configuration_;
        fcl::Vec3f com_;
        std::map<std::string, bool> contacts_;
        std::map<std::string, fcl::Vec3f> contactNormals_;
        std::map<std::string, fcl::Vec3f> contactPositions_;
        std::map<std::string, fcl::Matrix3f> contactRotation_;
        std::queue<std::string> contactOrder_;
        std::size_t nbContacts;
        bool stable;

        void print() const
        {
            std::cout << " State " << std::endl;
            /*std::cout << " \t Configuration " << std::endl;
            for(int i = 0; i< configuration_.rows(); ++i)
            {
                std::cout << configuration_[i] << " ";
            }
            std::cout << std::endl;*/

            std::cout << " \t contacts " << std::endl;
            for(std::map<std::string, bool>::const_iterator cit =
                contacts_.begin(); cit != contacts_.end(); ++cit)
            {
                std::cout << cit->first << ": " <<  cit->second << std::endl;
            }

            std::cout << "\t stable " << this->stable  << std::endl;

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


        void printInternal(std::stringstream& ss) const
        {
            std::map<std::string, fcl::Vec3f>::const_iterator cit = contactNormals_.begin();
            for(unsigned int c=0; c < nbContacts; ++c, ++cit)
            {
                const std::string& name = cit->first;
                //const fcl::Vec3f& normal = contactNormals_.at(name);
                const fcl::Vec3f& position = contactPositions_.at(name);
                ss << " " << name <<" ";
                for(std::size_t i=0; i<3; ++i)
                {
                    ss << " " << position[i];
                }
                /*for(std::size_t i=0; i<3; ++i)
                {
                    ss << " " << normal[i];
                }*/
                ss << "\n";
            }
            ss << "com ";
            for(std::size_t i=0; i<3; ++i)
            {
                ss << " " << com_[i];
            }
            ss << "\n" << "configuration ";
            for(int i=0; i<configuration_.rows(); ++i)
            {
                ss << " " << configuration_[i];
            }
            ss << "\n \n";
        }


        void print(std::stringstream& ss) const
        {
            ss << nbContacts << "\n";
            ss << "";
            std::map<std::string, fcl::Vec3f>::const_iterator cit = contactNormals_.begin();
            for(unsigned int c=0; c < nbContacts; ++c, ++cit)
            {
                ss << " " << cit->first << " ";
            }
            ss << "\n";
            printInternal(ss);
        }

        void print(std::stringstream& ss, const State& previous) const
        {
            ss << nbContacts << "\n";
            std::vector<std::string> ncontacts;
            ss << "";
            for(std::map<std::string, fcl::Vec3f>::const_iterator cit = contactPositions_.begin();
                cit != contactPositions_.end(); ++cit)
            {
                const std::string& name = cit->first;
                bool newContact(true);
                if(previous.contactPositions_.find(name) != previous.contactPositions_.end())
                {
                    newContact = (previous.contactPositions_.at(name) - cit->second).norm() > 0.01;
                }
                if(newContact)
                {
                    ncontacts.push_back(name);
                    ss << name << " ";
                }
            }
            ss << "\n";
            /*ss << "broken Contacts: ";
            for(std::map<std::string, fcl::Vec3f>::const_iterator cit = previous.contactPositions_.begin();
                cit != previous.contactPositions_.end(); ++cit)
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
                for(std::size_t i=0; i<3; ++i)
                {
                    ss << " " << com_[i];
                }
                ss << "\n";
            }
            for(int i=0; i<configuration_.rows(); ++i)
            {
                ss << " " << configuration_[i];
            }
            ss << "\n \n";*/
        }

    }; // struct State
  } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_STATE_HH
