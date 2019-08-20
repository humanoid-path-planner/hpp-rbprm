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
#include <hpp/pinocchio/joint.hh>
#include <hpp/rbprm/tools.hh>
#include <hpp/rbprm/contact_generation/kinematics_constraints.hh>
#include <pinocchio/spatial/se3.hpp>
namespace hpp {
  namespace rbprm {

    RbPrmLimbPtr_t RbPrmLimb::create (const pinocchio::JointPtr_t limb, const std::string& effectorName, const fcl::Vec3f &offset,
                                      const fcl::Vec3f &limbOffset, const fcl::Vec3f &normal,const double x, const double y,
                                      const std::size_t nbSamples, const hpp::rbprm::sampling::heuristic evaluate, const double resolution,
                                      hpp::rbprm::ContactType contactType, const bool disableEffectorCollision, const bool grasp,
                                      const std::string& kinematicsConstraintsPath,
                                      const double kinematicConstraintsMinDistance)
    {
        RbPrmLimb* rbprmDevice = new RbPrmLimb(limb, effectorName, offset,limbOffset, normal, x, y, nbSamples,evaluate,
                                               resolution, contactType, disableEffectorCollision, grasp,kinematicsConstraintsPath,kinematicConstraintsMinDistance);
        RbPrmLimbPtr_t res (rbprmDevice);
        res->init (res);
        return res;
    }

    RbPrmLimbPtr_t RbPrmLimb::create (const pinocchio::DevicePtr_t device, std::ifstream& fileStream, const bool loadValues,
                                      const hpp::rbprm::sampling::heuristic evaluate, const bool disableEffectorCollision, const bool grasp)
    {
        RbPrmLimb* rbprmDevice = new RbPrmLimb(device, fileStream, loadValues, evaluate, disableEffectorCollision, grasp);
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

    pinocchio::Frame GetEffector(const pinocchio::JointPtr_t limb, const std::string name ="")
    {        
        if (name != "")
            return limb->robot()->getFrameByName(name);
        pinocchio::JointPtr_t current = limb;
        while(current->numberChildJoints() !=0)
        {
            //assert(current->numberChildJoints() ==1);
            current = current->childJoint(0);
        }
        return limb->robot()->getFrameByName(current->name());
    }

    fcl::Matrix3f GetEffectorTransform(const pinocchio::Frame& effector)
    {
        pinocchio::Frame ef(effector);
        pinocchio::Configuration_t save = ef.robot()->currentConfiguration ();
        ef.robot()->currentConfiguration (ef.robot()->neutralConfiguration());
        const pinocchio::matrix3_t& rot (ef.currentTransformation().rotation());
        ef.robot()->currentConfiguration (save);
        return rot.transpose();
    }

    fcl::Vec3f computeEffectorReferencePosition(const pinocchio::JointPtr_t& limb, const std::string& effectorName){
        core::DevicePtr_t device = limb->robot();
        core::Configuration_t referenceConfig = device->neutralConfiguration();
        pinocchio::Transform3f tRoot(1);
        pinocchio::Transform3f tJoint_world(1),tJoint_robot(1);
        tRoot.translation(fcl::Vec3f(referenceConfig.head<3>()));
        //fcl::Quaternion3f quatRoot(referenceConfig[6],referenceConfig[3],referenceConfig[4],referenceConfig[5]);
        tRoot.rotation(Eigen::Quaterniond(referenceConfig.segment<4>(3)).matrix());
        hppDout(notice,"Create limb, reference root transform : "<<tRoot);
        // retrieve transform of each effector joint
        device->currentConfiguration(referenceConfig);
        device->computeForwardKinematics();
        tJoint_world = GetEffector(limb, effectorName).currentTransformation();
        hppDout(notice,"tJoint of "<<limb->name()<<" : "<<tJoint_world);
        //tJoint_robot = tRoot.inverseTimes(tJoint_world);
        tJoint_robot = tRoot.inverse()*tJoint_world;
        hppDout(notice,"tJoint relative : "<<tJoint_robot);
        return tJoint_robot.translation();
    }

    RbPrmLimb::RbPrmLimb (const pinocchio::JointPtr_t& limb, const std::string& effectorName,
                          const fcl::Vec3f &offset, const fcl::Vec3f &limbOffset, const fcl::Vec3f &normal, const double x, const double y, const std::size_t nbSamples,
                          const hpp::rbprm::sampling::heuristic evaluate, const double resolution, ContactType contactType,
                          bool disableEndEffectorCollision, bool grasps,
                            const std::string &kinematicsConstraintsPath , const double kinematicConstraintsMinDistance)
        : limb_(limb)
        , effector_(GetEffector(limb, effectorName))
        , effectorDefaultRotation_(GetEffectorTransform(effector_))
        , offset_(effectorDefaultRotation_* offset)
        , limbOffset_(limbOffset)
        , normal_(effectorDefaultRotation_* normal)
        //, normal_(normal)
        , x_(x)
        , y_(y)
        , contactType_(contactType)
        , evaluate_(evaluate)
        , sampleContainer_(limb, effector_.name(), nbSamples, offset,limbOffset, resolution)
        , disableEndEffectorCollision_(disableEndEffectorCollision)
        , grasps_(grasps)
        , effectorReferencePosition_(computeEffectorReferencePosition(limb,effectorName))
        , kinematicConstraints_(reachability::loadConstraintsFromObj(kinematicsConstraintsPath.empty() ? ("package://"+limb_->robot()->name()+"-rbprm/com_inequalities/"+limb_->name()+"_com_constraints.obj") : kinematicsConstraintsPath,kinematicConstraintsMinDistance))
    {
        // NOTHING
        hppDout(notice,"Create limb, normal = "<<normal);
        hppDout(notice,"effector default rotation = "<<effectorDefaultRotation_);
        hppDout(notice,"nex normal : "<<normal_);
    }

    pinocchio::Transform3f RbPrmLimb::octreeRoot() const
    {
        if (limb_->parentJoint())
            return limb_->parentJoint()->currentTransformation();
        return limb_->currentTransformation();
    }

    bool saveLimbInfoAndDatabase(const hpp::rbprm::RbPrmLimbPtr_t limb, std::ofstream& fp)
    {
        fp << limb->limb_->name() << std::endl;
        fp << limb->effector_.name() << std::endl;
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
        hpp::pinocchio::JointPtr_t extractJoint(const hpp::pinocchio::DevicePtr_t device, std::ifstream& myfile)
        {
            std::string name;
            getline(myfile, name);
            return device->getJointByName(name);
        }

        hpp::pinocchio::Frame extractFrame(const hpp::pinocchio::DevicePtr_t device, std::ifstream& myfile)
        {
            std::string name;
            getline(myfile, name);
            return device->getFrameByName(name);
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

    hpp::rbprm::RbPrmLimb::RbPrmLimb (const pinocchio::DevicePtr_t device, std::ifstream& fileStream,
                        const bool loadValues, const hpp::rbprm::sampling::heuristic evaluate,
                        bool disableEndEffectorCollision, bool grasps)
      : limb_(extractJoint(device,fileStream))
      , effector_(extractFrame(device,fileStream))
      , effectorDefaultRotation_(tools::io::readRotMatrixFCL(fileStream))
      , offset_(readVecFCL(fileStream))
      , normal_(readVecFCL(fileStream))
      , x_(StrToD(fileStream))
      , y_(StrToD(fileStream))
      , contactType_(static_cast<hpp::rbprm::ContactType>(StrToI(fileStream)))
      , evaluate_(evaluate)
      , sampleContainer_(fileStream, loadValues)
      , disableEndEffectorCollision_(disableEndEffectorCollision)
      , grasps_(grasps)
      , effectorReferencePosition_(computeEffectorReferencePosition(limb_,effector_.name()))
      , kinematicConstraints_(reachability::loadConstraintsFromObj("package://"+limb_->robot()->name()+"-rbprm/com_inequalities/"+limb_->name()+"_com_constraints.obj",0.3))
    {
      // NOTHING
    }
} //hpp


