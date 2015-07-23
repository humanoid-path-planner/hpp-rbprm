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

#include <hpp/rbprm/stability/stability.hh>
#include <hpp/rbprm/stability/support.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>

#include <Eigen/Dense>

#include <vector>
#include <map>
#include <string>

using namespace hpp;
using namespace hpp::core;
using namespace hpp::model;
using namespace hpp::rbprm;

namespace hpp {
namespace rbprm {
namespace stability{

    const polytope::vector3_t gravity(0,0,-9.81);
    static bool init = false;

    const polytope::ProjectedCone* computeCone(const RbPrmFullBodyPtr_t fullbody, const State& state)
    {
        std::vector<std::string> contacts;
        for(std::map<std::string,bool>::const_iterator cit = state.contacts_.begin();
            cit!=state.contacts_.end(); ++ cit)
        {
            if(cit->second) contacts.push_back(cit->first);
        }
        const std::size_t nbContacts = contacts.size();
        hpp::model::ConfigurationIn_t save = fullbody->device_->currentConfiguration();
        fullbody->device_->currentConfiguration(state.configuration_);
        polytope::T_rotation_t rotations(nbContacts*3, 3);
        polytope::vector_t positions(nbContacts*3);
        polytope::vector_t frictions(nbContacts);
        polytope::vector_t xs(nbContacts);
        polytope::vector_t ys(nbContacts);
        for(std::size_t c = 0; c< nbContacts; ++c)
        {
            const RbPrmLimbPtr_t limb =fullbody->GetLimbs().at(contacts[c]);
            const fcl::Transform3f& transform = limb->effector_->currentTransformation();
            for(int i = 0; i<3; ++i)
            {
                for(int j =0; j<3; ++j)
                {
                    rotations(i+3*c,j) = transform.getRotation()(i,j);
                }
                positions(i+3*c) = transform.getTranslation()[i];
            }
            frictions(c) = 0.3; // TODO parametrize
            xs(c) = limb->x_;
            ys(c) = limb->y_;
        }
        fullbody->device_->currentConfiguration(save);
        return polytope::U_stance(rotations,positions,frictions,xs,ys);
    }

    bool IsStable(const RbPrmFullBodyPtr_t fullbody, const State& state)
    {
        if(!init)
        {
            polytope::init_library();
            init = true;
        }
        std::vector<std::string> contacts;
        for(std::map<std::string,bool>::const_iterator cit = state.contacts_.begin();
            cit!=state.contacts_.end(); ++ cit)
        {
            if(cit->second) contacts.push_back(cit->first);
        }
        const std::size_t nbContacts = contacts.size();
        hpp::model::ConfigurationIn_t save = fullbody->device_->currentConfiguration();
        fullbody->device_->currentConfiguration(state.configuration_);
        polytope::T_rotation_t rotations(nbContacts*3, 3);
        polytope::vector_t positions(nbContacts*3);
        polytope::vector_t frictions(nbContacts);
        polytope::vector_t xs(nbContacts);
        polytope::vector_t ys(nbContacts);
        for(std::size_t c = 0; c< nbContacts; ++c)
        {
            const RbPrmLimbPtr_t limb =fullbody->GetLimbs().at(contacts[c]);
            const fcl::Transform3f& transform = limb->effector_->currentTransformation();
rotations.block<3,3>(3*c,0) = Eigen::Matrix3d::Identity();
            for(int i = 0; i<3; ++i)
            {
                /*for(int j =0; j<3; ++j)
                {
                    rotations(i+3*c,j) = transform.getRotation()(i,j);
                }*/
                positions(i+3*c) = transform.getTranslation()[i];
            }
            frictions(c) = 0.7; // TODO parametrize
            xs(c) = limb->x_;
            ys(c) = limb->y_;
        }

       /* std::cout << "stability test " << std::endl;
        std::cout << "positions \n " << positions << std::endl;
        std::cout << "rotations  \n" << rotations << std::endl;
        std::cout << "frictions  \n" << frictions << std::endl;
        std::cout << "xs  \n" << xs << std::endl;
        std::cout << "ys  \n" << ys << std::endl;
        std::cout << "mass  \n" << fullbody->device_->mass() << std::endl;
        std::cout << "com  \n" << fullbody->device_->positionCenterOfMass() << std::endl;*/


        fullbody->device_->currentConfiguration(save);
        const polytope::ProjectedCone* cone = polytope::U_stance(rotations,positions,frictions,xs,ys);
        if(cone)
        {
            polytope::vector3_t com;
            const fcl::Vec3f comfcl = fullbody->device_->positionCenterOfMass();
            for(int i=0; i< 3; ++i) com(i)=comfcl[i];
            bool res = cone->IsValid(com,gravity,fullbody->device_->mass());
            delete cone;
            return res;
        }
        return false;
    }

    bool IsStablePoly(const RbPrmFullBodyPtr_t fullbody, const State& state)
    {
        std::vector<std::string> contacts;
        for(std::map<std::string,bool>::const_iterator cit = state.contacts_.begin();
            cit!=state.contacts_.end(); ++ cit)
        {
            if(cit->second) contacts.push_back(cit->first);
        }
        const std::size_t nbContacts = contacts.size();
        hpp::model::ConfigurationIn_t save = fullbody->device_->currentConfiguration();
        fullbody->device_->currentConfiguration(state.configuration_);
        polytope::T_rotation_t rotations(nbContacts*3, 3);
        polytope::vector_t positions(nbContacts*3);
        polytope::vector_t frictions(nbContacts);
        polytope::vector_t xs(nbContacts);
        polytope::vector_t ys(nbContacts);
        for(std::size_t c = 0; c< nbContacts; ++c)
        {
            const RbPrmLimbPtr_t limb =fullbody->GetLimbs().at(contacts[c]);
            const fcl::Transform3f& transform = limb->effector_->currentTransformation();
rotations.block<3,3>(3*c,0) = Eigen::Matrix3d::Identity();
            for(int i = 0; i<3; ++i)
            {
                /*for(int j =0; j<3; ++j)
                {
                    rotations(i+3*c,j) = transform.getRotation()(i,j);
                }*/
                positions(i+3*c) = transform.getTranslation()[i];
            }
            frictions(c) = 0.7; // TODO parametrize
            xs(c) = limb->x_;
            ys(c) = limb->y_;
        }
        polytope::vector3_t com;
        const fcl::Vec3f comfcl = fullbody->device_->positionCenterOfMass();
        for(int i=0; i< 3; ++i) com(i)=comfcl[i];
        return Contains(positions,com,xs,ys);
    }
}
}
}
