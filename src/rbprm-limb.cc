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
#include <hpp/model/joint.hh>

namespace hpp {
  namespace rbprm {

    RbPrmLimbPtr_t RbPrmLimb::create (const model::JointPtr_t limb, const fcl::Vec3f &offset,
                                      const fcl::Vec3f &normal,const double x, const double y,
                                      const std::size_t nbSamples, const sampling::heuristic evaluate, const double resolution)
    {
        RbPrmLimb* rbprmDevice = new RbPrmLimb(limb, offset, normal, x, y, nbSamples,evaluate, resolution);
        RbPrmLimbPtr_t res (rbprmDevice);
        res->init (res);
        return res;
    }

    RbPrmLimbPtr_t RbPrmLimb::create (const model::JointPtr_t limb, const std::string& effectorName, const fcl::Vec3f &offset,
                                      const fcl::Vec3f &normal,const double x, const double y,
                                      const std::size_t nbSamples, const hpp::rbprm::sampling::heuristic evaluate, const double resolution)
    {
        RbPrmLimb* rbprmDevice = new RbPrmLimb(limb, effectorName, offset, normal, x, y, nbSamples,evaluate, resolution);
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

    RbPrmLimb::RbPrmLimb (const model::JointPtr_t& limb,
                          const fcl::Vec3f &offset, const fcl::Vec3f &normal, const double x, const double y, const std::size_t nbSamples,
                          const hpp::rbprm::sampling::heuristic evaluate, const double resolution)
        : limb_(limb)
        , effector_(GetEffector(limb))
        , effectorDefaultRotation_(GetEffectorTransform(limb))
        , sampleContainer_(limb, effector_->name(), nbSamples, evaluate, offset, resolution)
        , offset_(effectorDefaultRotation_* offset)
        , normal_(effectorDefaultRotation_* normal)
        , x_(x)
        , y_(y)
    {
        // TODO
    }

    RbPrmLimb::RbPrmLimb (const model::JointPtr_t& limb, const std::string& effectorName,
                          const fcl::Vec3f &offset, const fcl::Vec3f &normal, const double x, const double y, const std::size_t nbSamples,
                          const hpp::rbprm::sampling::heuristic evaluate, const double resolution)
        : limb_(limb)
        , effector_(GetEffector(limb, effectorName))
        , effectorDefaultRotation_(GetEffectorTransform(limb))
        , sampleContainer_(limb, effector_->name(), nbSamples, evaluate, offset, resolution)
        , offset_(effectorDefaultRotation_* offset)
        , normal_(effectorDefaultRotation_* normal)
        , x_(x)
        , y_(y)
    {
        // TODO
    }

    fcl::Transform3f RbPrmLimb::octreeRoot() const
    {
        hpp::model::Configuration_t config = limb_->robot()->currentConfiguration().head(7);
        fcl::Quaternion3f quat (config[3],config[4],config[5],config[6]);
        return fcl::Transform3f(quat,fcl::Vec3f(config[0],config[1],config[2]));
    }
  } // model
} //hpp
