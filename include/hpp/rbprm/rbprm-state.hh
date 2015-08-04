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
        std::map<std::string, bool> contacts_;
        std::map<std::string, fcl::Vec3f> contactNormals_;
        std::map<std::string, fcl::Vec3f> contactPositions_;
        std::map<std::string, fcl::Matrix3f> contactRotation_;
        std::queue<std::string> contactOrder_;
        unsigned int nbContacts;
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

    }; // struct State
  } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_STATE_HH
