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

#include <hpp/rbprm/rbprm-limb.hh>
#include <hpp/rbprm/sampling/sample-db.hh>
#include <hpp/model/joint.hh>
#include <hpp/rbprm/tools.hh>

namespace hpp {
  namespace rbprm {

    RbPrmLimbPtr_t RbPrmLimb::create (const model::JointPtr_t limb, const std::string& effectorName, const fcl::Vec3f &offset,
                                      const fcl::Vec3f &normal,const double x, const double y,
                                      const std::size_t nbSamples, const hpp::rbprm::sampling::heuristic evaluate, const double resolution,
                                      hpp::rbprm::ContactType contactType)
    {
        RbPrmLimb* rbprmDevice = new RbPrmLimb(limb, effectorName, offset, normal, x, y, nbSamples,evaluate, resolution, contactType);
        RbPrmLimbPtr_t res (rbprmDevice);
        res->init (res);
        return res;
    }

    RbPrmLimbPtr_t RbPrmLimb::create (const model::DevicePtr_t device, std::ifstream& fileStream,
                                      const hpp::rbprm::sampling::heuristic evaluate)
    {
        RbPrmLimb* rbprmDevice = new RbPrmLimb(device, fileStream, evaluate);
        RbPrmLimbPtr_t res (rbprmDevice);
        res->init (res);
        return res;
    }

    RbPrmLimb::~RbPrmLimb()
    {
        // NOTHING
    }

    // ========================================================================

    void RbPrmLimb::init(const RbPrmLimbWkPtr_t& weakPtr)
    {
        weakPtr_ = weakPtr;
    }

    model::JointPtr_t GetEffector(const model::JointPtr_t limb, const std::string name ="")
    {
        model::JointPtr_t current = limb;
        while(current->numberChildJoints() !=0)
        {
            //assert(current->numberChildJoints() ==1);
            current = current->childJoint(0);
            if(current->name() == name) break;
        }
        return current;
    }

    fcl::Matrix3f GetEffectorTransform(const model::JointPtr_t effector)
    {
        model::Configuration_t save = effector->robot()->currentConfiguration ();
        effector->robot()->currentConfiguration (effector->robot()->neutralConfiguration());
        fcl::Matrix3f rot = effector->currentTransformation().getRotation();
        effector->robot()->currentConfiguration (save);
        return rot.transpose();
    }

    RbPrmLimb::RbPrmLimb (const model::JointPtr_t& limb, const std::string& effectorName,
                          const fcl::Vec3f &offset, const fcl::Vec3f &normal, const double x, const double y, const std::size_t nbSamples,
                          const hpp::rbprm::sampling::heuristic evaluate, const double resolution, ContactType contactType)
        : limb_(limb)
        , effector_(GetEffector(limb, effectorName))
        , effectorDefaultRotation_(GetEffectorTransform(limb))
        , offset_(effectorDefaultRotation_* offset)
        , normal_(effectorDefaultRotation_* normal)
        , x_(x)
        , y_(y)
        , contactType_(contactType)
        , evaluate_(evaluate)
        , sampleContainer_(limb, effector_->name(), nbSamples, offset, resolution)
    {
        // NOTHING
    }

    fcl::Transform3f RbPrmLimb::octreeRoot() const
    {
        return limb_->parentJoint()->currentTransformation();
    }

    bool saveLimbInfoAndDatabase(const hpp::rbprm::RbPrmLimbPtr_t limb, std::ofstream& fp)
    {
        fp << limb->limb_->name() << std::endl;
        fp << limb->effector_->name() << std::endl;
        tools::io::writeRotMatrixFCL(limb->effectorDefaultRotation_, fp); fp << std::endl;
        tools::io::writeVecFCL(limb->offset_, fp); fp << std::endl;
        tools::io::writeVecFCL(limb->normal_, fp); fp << std::endl;
        fp << limb->x_ << std::endl;
        fp << limb->y_ << std::endl;
        fp << (int)limb->contactType_ << std::endl;
        return sampling::saveLimbDatabase(limb->sampleContainer_,fp);
    }
  } // rbprm


    namespace tools
    {
    namespace io
    {
        hpp::model::JointPtr_t extractJoint(const hpp::model::DevicePtr_t device, std::ifstream& myfile)
        {
            std::string name;
            getline(myfile, name);
            return device->getJointByName(name);
        }

        std::ostream& operator << (std::ostream& out, hpp::rbprm::ContactType ctype)
        {
            unsigned u = ctype;
            out << u;
            return out;
        }
    }
    }
    using namespace hpp::tools::io;

    hpp::rbprm::RbPrmLimb::RbPrmLimb (const model::DevicePtr_t device, std::ifstream& fileStream,
                        const hpp::rbprm::sampling::heuristic evaluate)
      : limb_(extractJoint(device,fileStream))
      , effector_(extractJoint(device,fileStream))
      , effectorDefaultRotation_(tools::io::readRotMatrixFCL(fileStream))
      , offset_(readVecFCL(fileStream))
      , normal_(readVecFCL(fileStream))
      , x_(StrToD(fileStream))
      , y_(StrToD(fileStream))
      , contactType_(static_cast<hpp::rbprm::ContactType>(StrToI(fileStream)))
      , evaluate_(evaluate)
      , sampleContainer_(fileStream)
    {
      // NOTHING
    }
} //hpp


