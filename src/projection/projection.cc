//
// Copyright (c) 2017 CNRS
// Authors: Steve Tonneau
//
// This file is part of hpp-rbprm
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/rbprm/projection/projection.hh>
#include <hpp/rbprm/interpolation/interpolation-constraints.hh>
#include <hpp/model/joint.hh>

#ifdef PROFILE
    #include "hpp/rbprm/rbprm-profiler.hh"
#endif




namespace hpp {
namespace rbprm {
namespace projection{

const double epsilon = 10e-3;

std::vector<bool> setMaintainRotationConstraints()
{
    std::vector<bool> res;
    for(std::size_t i =0; i <3; ++i)
        res.push_back(true);
    return res;
}

void CreateContactConstraints(hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const hpp::rbprm::State& currentState, core::ConfigProjectorPtr_t proj)
{
    model::DevicePtr_t device = fullBody->device_;
    std::vector<bool> cosntraintsR = setMaintainRotationConstraints();
    std::queue<std::string> fixed = currentState.contactOrder_;
    while(!fixed.empty())
    {
        const std::string effector = fixed.front();
        fixed.pop();
        RbPrmLimbPtr_t limb = fullBody->GetLimbs().at(effector);
        const fcl::Vec3f& ppos  = currentState.contactPositions_.at(effector);
        JointPtr_t effectorJoint = device->getJointByName(limb->effector_->name());
        proj->add(core::NumericalConstraint::create (
                                constraints::deprecated::Position::create("",device,
                                                              effectorJoint,fcl::Vec3f(0,0,0), ppos)));
        if(limb->contactType_ == hpp::rbprm::_6_DOF)
        {
            const fcl::Matrix3f& rotation = currentState.contactRotation_.at(effector);
            proj->add(core::NumericalConstraint::create (constraints::deprecated::Orientation::create("", device,
                                                                              effectorJoint,
                                                                              rotation,
                                                                              cosntraintsR)));
        }
    }
}

void CreateRootPosConstraint(hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const fcl::Vec3f& target, core::ConfigProjectorPtr_t proj)
{
    proj->add(core::NumericalConstraint::create (
                            constraints::deprecated::Position::create("",fullBody->device_,
                                                          fullBody->device_->rootJoint(),fcl::Vec3f(0,0,0), target)));
}

ProjectionReport projectToRootPosition(hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const fcl::Vec3f& target,
                                           const hpp::rbprm::State& currentState)
{
    ProjectionReport res;
    core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(fullBody->device_,"proj", 0.001, 40);
    CreateContactConstraints(fullBody, currentState, proj);
    CreateRootPosConstraint(fullBody, target, proj);
    model::Configuration_t configuration = currentState.configuration_;
    res.success_ = proj->apply(configuration);
    res.result_ = currentState;
    res.result_.configuration_ = configuration;
    return res;
}

typedef std::vector<model::JointPtr_t> T_Joint;

T_Joint getJointsFromLimbs(const rbprm::T_Limb limbs)
{
    T_Joint res;
    for(rbprm::CIT_Limb cit = limbs.begin(); cit != limbs.end(); ++cit)
        res.push_back(cit->second->limb_);
    return res;
}

bool not_a_limb(model::JointPtr_t cJoint, T_Joint limbs)
{
    for(T_Joint::const_iterator cit = limbs.begin(); cit != limbs.end(); ++cit)
    {
        if(cJoint->name() == (*cit)->name()) return false;
    }
    return true;
}

void LockFromRoot(hpp::model::DevicePtr_t device, const rbprm::T_Limb& limbs, model::ConfigurationIn_t targetRootConfiguration, core::ConfigProjectorPtr_t& projector)
{
    std::vector<model::JointPtr_t> jointLimbs = getJointsFromLimbs(limbs);
    model::JointPtr_t cJoint = device->rootJoint();
    core::size_type rankInConfiguration;
    while(not_a_limb(cJoint, jointLimbs))
    {
        rankInConfiguration = (cJoint->rankInConfiguration ());
        projector->add(core::LockedJoint::create(cJoint,targetRootConfiguration.segment(rankInConfiguration, cJoint->configSize())));
        if (cJoint->numberChildJoints() !=1)
            return;
        cJoint = cJoint->childJoint(0);
    }
}


ProjectionReport projectToRootConfiguration(hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const model::ConfigurationIn_t conf,
                                           const hpp::rbprm::State& currentState)
{
    ProjectionReport res;
    core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(fullBody->device_,"proj", 0.001, 40);
    CreateContactConstraints(fullBody, currentState, proj);
    LockFromRoot(fullBody->device_, fullBody->GetLimbs(), conf, proj);
    model::Configuration_t configuration = currentState.configuration_;
    res.success_ = proj->apply(configuration);
    res.result_ = currentState;
    res.result_.configuration_ = configuration;
    return res;
}

ProjectionReport setCollisionFree(hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const core::CollisionValidationPtr_t& validation ,
                                  const std::string& limbName,const hpp::rbprm::State& currentState)
{
    ProjectionReport res;
    res.result_ = currentState;
    model::Configuration_t configuration = currentState.configuration_;
    RbPrmLimbPtr_t limb = fullBody->GetLimbs().at(limbName);
    for(sampling::SampleVector_t::const_iterator cit = limb->sampleContainer_.samples_.begin();
        cit != limb->sampleContainer_.samples_.end(); ++cit)
    {
        hpp::core::ValidationReportPtr_t valRep (new hpp::core::CollisionValidationReport);
        if(validation->validate(configuration, valRep) )
        {
            res.result_.configuration_ = configuration;
            res.success_ = true;
            return res;
        }
        // load after to test current configuraiton (so miss the last configuration but that s probably okay..)
        sampling::Load(*cit, configuration);
    }
    return res;
}

std::vector<bool> setRotationConstraints()
{
    std::vector<bool> res;
    for(std::size_t i =0; i <3; ++i)
    {
        res.push_back(true);
    }
    return res;
}

std::vector<bool> setTranslationConstraints()
{
    std::vector<bool> res;
    for(std::size_t i =0; i <3; ++i)
    {
        res.push_back(true);
    }
    return res;
}
hpp::rbprm::State Project(const hpp::rbprm::RbPrmFullBodyPtr_t& body, const std::string& limbId, const hpp::rbprm::RbPrmLimbPtr_t& limb,
                          core::CollisionValidationPtr_t validation, model::ConfigurationOut_t configuration,
                          const fcl::Matrix3f& rotationTarget, const std::vector<bool> &rotationFilter, const fcl::Vec3f& positionTarget, const fcl::Vec3f& normal,
                          const hpp::rbprm::State& current, bool& success)
{
    // Add constraints to resolve Ik
    success = false;
    // Add constraints to resolve Ik
    core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(body->device_,"proj", 1e-4, 20);
    // get current normal orientation
    hpp::tools::LockJointRec(limb->limb_->name(), body->device_->rootJoint(), proj);
    fcl::Transform3f localFrame, globalFrame;
    globalFrame.setTranslation(positionTarget);
    proj->add(core::NumericalConstraint::create (constraints::Position::create("",body->device_,
                                                                               limb->effector_,
                                                                               localFrame,
                                                                               globalFrame,
                                                                               setTranslationConstraints())));

    if(limb->contactType_ == hpp::rbprm::_6_DOF)
    {
        proj->add(core::NumericalConstraint::create (constraints::Orientation::create("",body->device_,
                                                                                      limb->effector_,
                                                                                      fcl::Transform3f(rotationTarget),
                                                                                      rotationFilter)));
    }
#ifdef PROFILE
    RbPrmProfiler& watch = getRbPrmProfiler();
    watch.start("ik");
#endif
    if(proj->apply(configuration))
    {
#ifdef PROFILE
        watch.stop("ik");
#endif
#ifdef PROFILE
        RbPrmProfiler& watch = getRbPrmProfiler();
        watch.start("collision");
#endif
        hpp::core::ValidationReportPtr_t valRep (new hpp::core::CollisionValidationReport);
        if(validation->validate(configuration, valRep))
    {
#ifdef PROFILE
        watch.stop("collision");
#endif
        body->device_->currentConfiguration(configuration);
        body->device_->computeForwardKinematics();
        State tmp (current);
        tmp.contacts_[limbId] = true;
        tmp.contactPositions_[limbId] = limb->effector_->currentTransformation().getTranslation();
        tmp.contactRotation_[limbId] = limb->effector_->currentTransformation().getRotation();
        tmp.contactNormals_[limbId] = normal;
        tmp.configuration_ = configuration;
        ++tmp.nbContacts;
        success = true;
        return tmp;
    }
#ifdef PROFILE
    else
        watch.stop("collision");
#endif
    }
#ifdef PROFILE
    else
        watch.stop("ik");
#endif
    success = false;
    return current;
}

hpp::rbprm::State ProjectSampleToObstacle(const hpp::rbprm::RbPrmFullBodyPtr_t& body,const std::string& limbId, const hpp::rbprm::RbPrmLimbPtr_t& limb,
                   const sampling::OctreeReport& report, core::CollisionValidationPtr_t validation,
                   model::ConfigurationOut_t configuration, const hpp::rbprm::State& current, bool& success)
{
    sampling::Load(*report.sample_, configuration);
    body->device_->currentConfiguration(configuration);
    body->device_->computeForwardKinematics();
    const fcl::Vec3f& normal = report.normal_;
    const fcl::Vec3f& position = report.contact_.pos;
    // the normal is given by the normal of the contacted object
    const fcl::Vec3f z = limb->effector_->currentTransformation().getRotation() * limb->normal_;
    const fcl::Matrix3f alignRotation = tools::GetRotationMatrix(z,normal);
    const fcl::Matrix3f rotation = alignRotation * limb->effector_->currentTransformation().getRotation();
    fcl::Vec3f posOffset = position - rotation * limb->offset_;
    posOffset = posOffset + normal * epsilon;
    return Project(body, limbId, limb, validation, configuration, rotation, setRotationConstraints(),posOffset, normal, current, success);
}

} // namespace projection
} // namespace rbprm
} // namespace hpp
