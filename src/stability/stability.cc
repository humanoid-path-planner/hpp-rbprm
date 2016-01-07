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

#include "polytope/stability_margin.h"
#include <robust-equilibrium-lib/static_equilibrium.hh>

#include <Eigen/Dense>

#include <vector>
#include <map>
#include <string>

#ifdef PROFILE
    #include "hpp/rbprm/rbprm-profiler.hh"
#endif

using namespace hpp;
using namespace hpp::core;
using namespace hpp::model;
using namespace hpp::rbprm;
using namespace robust_equilibrium;

namespace hpp {
namespace rbprm {
namespace stability{

    const polytope::vector3_t gravity(0,0,-9.81);

    void computeRectangleContact(const RbPrmLimbPtr_t limb, Ref_matrix43 p)
    {
        const double& lx = limb->x_, ly = limb->y_;
        const fcl::Transform3f& transform = limb->effector_->currentTransformation();
        const fcl::Matrix3f& fclRotation = transform.getRotation();
        const fcl::Vec3f& po = transform.getTranslation();
        Rotation R;
        for(int i =0; i< 3; ++i)
            for(int j =0; j<3;++j)
                R(i,j) = fclRotation(i,j);
        Eigen::Vector3d pos(po[0],po[1],po[2]);
        p << lx,  ly, 0,
             lx, -ly, 0,
            -lx, -ly, 0,
            -lx,  ly, 0;
        p.row(0) = pos + (R*p.row(0).transpose());
        p.row(1) = pos + (R*p.row(1).transpose());
        p.row(2) = pos + (R*p.row(2).transpose());
        p.row(3) = pos + (R*p.row(3).transpose());
    }

    double IsStable(const RbPrmFullBodyPtr_t fullbody, State& state)
    {
        hpp::model::ConfigurationIn_t save = fullbody->device_->currentConfiguration();
        std::vector<std::string> contacts;
        for(std::map<std::string,bool>::const_iterator cit = state.contacts_.begin();
            cit!=state.contacts_.end(); ++ cit)
        {
            if(cit->second) contacts.push_back(cit->first);
        }
        const std::size_t nbContacts = contacts.size();
        fullbody->device_->currentConfiguration(state.configuration_);
        fullbody->device_->computeForwardKinematics();
        robust_equilibrium::MatrixX3 normals  (nbContacts*4,3);
        robust_equilibrium::MatrixX3 positions(nbContacts*4,3);
        double frictions = 1;
        for(std::size_t c = 0; c< nbContacts; ++c)
        {
            const RbPrmLimbPtr_t limb =fullbody->GetLimbs().at(contacts[c]);
            const fcl::Vec3f& n = state.contactNormals_.at(contacts[c]);
            Vector3 normal(n[0],n[1],n[2]);
            computeRectangleContact(limb,positions.middleRows<4>(c*4));
            for(int i =0; i < 4; ++i)
            {
                normals.middleRows<1>(4*c+i) = normal;
            }
        }
        robust_equilibrium::Vector3 com;
        const fcl::Vec3f comfcl = fullbody->device_->positionCenterOfMass();
        state.com_ = comfcl;
        for(int i=0; i< 3; ++i) com(i)=comfcl[i];
        fullbody->device_->currentConfiguration(save);
#ifdef PROFILE
    RbPrmProfiler& watch = getRbPrmProfiler();
    watch.start("test balance");
#endif
        StaticEquilibrium staticEquilibrium(fullbody->device_->name(), fullbody->device_->mass(),4,SOLVER_LP_QPOASES);
        staticEquilibrium.setNewContacts(positions,normals,frictions,STATIC_EQUILIBRIUM_ALGORITHM_DLP);
        double res;

        LP_status status = staticEquilibrium.computeEquilibriumRobustness(com,res);
#ifdef PROFILE
    watch.stop("test balance");
#endif
        if(status != LP_STATUS_OPTIMAL)
        {
            if(status == LP_STATUS_INFEASIBLE || status == LP_STATUS_UNBOUNDED)
                return 1.1; // completely arbitrary: TODO
            return -std::numeric_limits<double>::max();
        }
        return res ;
    }

    //old polytope dd method
   /* bool IsStable(const RbPrmFullBodyPtr_t fullbody, State& state)
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
        else
        {
            std::cout << "no cone" << std::endl;
            return true;
        }
        fullbody->device_->currentConfiguration(save);
        return false;
    }*/

    double IsStablePoly(const RbPrmFullBodyPtr_t fullbody, State& state)
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
            xs(c) = limb->x_;
            ys(c) = limb->y_;
        }
        polytope::vector3_t com;
        const fcl::Vec3f comfcl = fullbody->device_->positionCenterOfMass();
        for(int i=0; i< 3; ++i) com(i)=comfcl[i];
        fullbody->device_->currentConfiguration(save);
        fullbody->device_->controlComputation (flag);
        return Contains(positions,com,xs,ys) ? 1 : -1;
    }
}
}
}
