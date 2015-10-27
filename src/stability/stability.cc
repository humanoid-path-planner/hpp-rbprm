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
#include <hpp/rbprm/tools.hh>

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

    bool IsStable(const RbPrmFullBodyPtr_t fullbody, State& state)
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
        fullbody->device_->computeForwardKinematics();
        polytope::T_rotation_t rotations(nbContacts*3, 3);
        polytope::vector_t positions(nbContacts*3);
        polytope::vector_t frictions(nbContacts);
        polytope::vector_t xs(nbContacts);
        polytope::vector_t ys(nbContacts);
        for(std::size_t c = 0; c< nbContacts; ++c)
        {
            const RbPrmLimbPtr_t limb =fullbody->GetLimbs().at(contacts[c]);
            const fcl::Transform3f& transform = limb->effector_->currentTransformation();
            //fcl::Matrix3f rotationInWorldCoordinates = limb->effectorDefaultRotation_;
            //rotationInWorldCoordinates.transpose();
            //fcl::Matrix3f tg = rotationInWorldCoordinates * transform.getRotation();
            //rotationInWorldCoordinates = transform.getRotation() * rotationInWorldCoordinates;
            fcl::Vec3f z(0,0,1);
            const fcl::Matrix3f alignRotation = tools::GetRotationMatrix(z,state.contactNormals_.at(contacts[c]));
            for(int i = 0; i<3; ++i)
            {
                for(int j =0; j<3; ++j)
                {
                    rotations(i+3*c,j) = alignRotation(i,j);
                }
                positions(i+3*c) = transform.getTranslation()[i];
            }
            frictions(c) = 0.5; // TODO parametrize
            xs(c) = limb->x_;
            ys(c) = limb->y_;
        }

        /*std::cout << "stability test " << std::endl;
        std::cout << "positions \n " << positions << std::endl;
        std::cout << "rotations  \n" << rotations << std::endl;
        std::cout << "frictions  \n" << frictions << std::endl;
        std::cout << "xs  \n" << xs << std::endl;
        std::cout << "ys  \n" << ys << std::endl;
        std::cout << "mass \n" << fullbody->device_->mass() << std::endl;
        std::cout << "com  \n" << fullbody->device_->positionCenterOfMass() << std::endl;*/


        const polytope::ProjectedCone* cone = polytope::U_stance(rotations,positions,frictions,xs,ys); //,true,2);
        if(cone)
        {
            polytope::vector3_t com;
            const fcl::Vec3f comfcl = fullbody->device_->positionCenterOfMass();
            state.com_ = comfcl;
            for(int i=0; i< 3; ++i) com(i)=comfcl[i];
            bool res = cone->IsValid(com,gravity,fullbody->device_->mass()) && cone->A.rows() >1;
            delete cone;
            fullbody->device_->currentConfiguration(save);
            return res;
        }
        fullbody->device_->currentConfiguration(save);
        return false;
    }

    bool IsStablePoly(const RbPrmFullBodyPtr_t fullbody, State& state)
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
        model::Device::Computation_t flag = fullbody->device_->computationFlag ();
        model::Device::Computation_t newflag = static_cast <model::Device::Computation_t> (model::Device::JOINT_POSITION | model::Device::COM);
        fullbody->device_->controlComputation (newflag);
        fullbody->device_->computeForwardKinematics ();
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
        fullbody->device_->currentConfiguration(save);
        fullbody->device_->controlComputation (flag);
        return Contains(positions,com,xs,ys);
    }
}
}
}
