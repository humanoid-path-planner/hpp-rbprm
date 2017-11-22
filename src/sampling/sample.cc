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

#include <hpp/rbprm/sampling/sample.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>
#include <hpp/model/configuration.hh>
#include <Eigen/Eigen>

using namespace hpp;
using namespace hpp::model;
using namespace hpp::rbprm;
using namespace hpp::rbprm::sampling;


std::size_t ComputeLength(const model::JointPtr_t limb, const model::JointPtr_t effector)
{
    std::size_t start = limb->rankInConfiguration();
    std::size_t end = effector->rankInConfiguration()
            + effector->neutralConfiguration().rows();
    return end - start;
}


fcl::Vec3f ComputeEffectorPosition(const model::JointPtr_t limb, const model::JointPtr_t effector, const fcl::Vec3f& offset)
{
    const fcl::Transform3f& transform = effector->currentTransformation();
    fcl::Transform3f parentT = fcl::inverse(limb->parentJoint()->currentTransformation());
    fcl::Vec3f tr (transform.getTranslation() + offset);
    return (parentT * tr).getTranslation();
}

/**
 * @brief ComputeEffectorPositionInLimbFrame compute the position of the end effector in the frame defined by the world orientation
 * but with the origin at the root of the limb
 * @param limb
 * @param effector
 * @param offset
 * @return
 */
fcl::Vec3f ComputeEffectorPositionInLimbFrame(const model::JointPtr_t limb, const model::JointPtr_t effector, const fcl::Vec3f& offset,const fcl::Vec3f&  limbOffset)
{
    // we want to use the orientation expressed in the world frame, but with the origin in the limb's root :
   // hppDout(notice,"offset = "<<offset);
  //  hppDout(notice,"limbOffset = "<<limbOffset);
    const fcl::Transform3f& transformLimb = limb->currentTransformation();
    fcl::Transform3f transformOffset;
    transformOffset.setTranslation(limbOffset);
    fcl::Vec3f effTr = effector->currentTransformation().getTranslation() + offset;
    fcl::Vec3f limbTr = (transformLimb*transformOffset).getTranslation();
  /*  hppDout(notice,"effTr = "<<effTr);
    hppDout(notice,"limbTr = "<<limbTr);
    hppDout(notice,"res = "<<effTr-limbTr);*/
    return (effTr-limbTr);
}

Eigen::MatrixXd Jacobian(const model::JointPtr_t limb, const model::JointPtr_t effector)
{
    return effector->jacobian().block(0,limb->rankInVelocity(),6, effector->rankInVelocity() - limb->rankInVelocity() + effector->numberDof());
}


double Manipulability(const Eigen::MatrixXd& product)
{
    double det = product.determinant();
    return det > 0 ? sqrt(det) : 0;
}

Sample::Sample(const model::JointPtr_t limb, const model::JointPtr_t effector, const fcl::Vec3f& offset,const fcl::Vec3f& limbOffset, std::size_t id)
    : startRank_(limb->rankInConfiguration())
    , length_ (ComputeLength(limb, effector))
    , configuration_ (limb->robot()->currentConfiguration().segment(startRank_, length_))
    , effectorPosition_(ComputeEffectorPosition(limb, effector,offset))
    , effectorPositionInLimbFrame_(ComputeEffectorPositionInLimbFrame(limb,effector,offset,limbOffset))
    , jacobian_(Jacobian(limb, effector))
    , jacobianProduct_(jacobian_*jacobian_.transpose())
    , id_(id)
    , staticValue_(Manipulability(jacobianProduct_))
{
    // NOTHING
}

Sample::Sample(const std::size_t id, const std::size_t length, const std::size_t startRank, const double staticValue,
               const fcl::Vec3f& effectorPosition,const fcl::Vec3f& effectorPositionInLimbFrame, const model::ConfigurationIn_t configuration, const Eigen::MatrixXd& jacobian,
               const Eigen::Matrix <model::value_type, 6, 6>& jacobianProduct)
    : startRank_(startRank)
    , length_ (length)
    , configuration_ (configuration)
    , effectorPosition_(effectorPosition)
    , effectorPositionInLimbFrame_(effectorPositionInLimbFrame)
    , jacobian_(jacobian)
    , jacobianProduct_(jacobianProduct)
    , id_(id)
    , staticValue_(staticValue)
{
    // NOTHING
}

Sample::Sample(const model::JointPtr_t limb, const model::JointPtr_t effector, model::ConfigurationIn_t configuration,  const fcl::Vec3f& offset,const fcl::Vec3f& limbOffset, std::size_t id)
    : startRank_(limb->rankInConfiguration())
    , length_ (ComputeLength(limb, effector))
    , configuration_ (configuration)
    , effectorPosition_(ComputeEffectorPosition(limb,effector,offset))
    , effectorPositionInLimbFrame_(ComputeEffectorPositionInLimbFrame(limb,effector,offset,limbOffset))
    , jacobian_(Jacobian(limb,effector))
    , jacobianProduct_(jacobian_*jacobian_.transpose())
    , id_(id)
    , staticValue_(Manipulability(jacobianProduct_))
{
    // NOTHING
}

Sample::Sample(const Sample &clone)

    : startRank_(clone.startRank_)
    , length_ (clone.length_)
    , configuration_ (clone.configuration_)
    , effectorPosition_(clone.effectorPosition_)
    , effectorPositionInLimbFrame_(clone.effectorPositionInLimbFrame_)
    , jacobian_(clone.jacobian_)
    , jacobianProduct_(clone.jacobianProduct_)
    , id_(clone.id_)
    , staticValue_(clone.staticValue_)
{
    // NOTHING
}

void hpp::rbprm::sampling::Load(const Sample& sample, ConfigurationOut_t configuration)
{
    configuration.segment(sample.startRank_, sample.length_) = sample.configuration_;
}

hpp::rbprm::sampling::SampleVector_t hpp::rbprm::sampling::GenerateSamples(const model::JointPtr_t model, const std::string& effector
                                                         , const std::size_t nbSamples, const fcl::Vec3f& offset,const fcl::Vec3f& limbOffset)
{
    SampleVector_t result; result.reserve(nbSamples);
    model::DevicePtr_t device (model->robot()->clone());
    Configuration_t config = device->currentConfiguration();
    JointPtr_t clone = device->getJointByName(model->name());
    JointPtr_t effectorClone = device->getJointByName(effector);
    std::size_t startRank_(model->rankInConfiguration());
    std::size_t length_ (ComputeLength(model, effectorClone));
    for(std::size_t i = 0; i< nbSamples; ++i)
    {
        clone->configuration ()->uniformlySample (clone->rankInConfiguration (), config);
        Joint* current = clone;
        while(current->numberChildJoints() !=0)
        {
            current = current->childJoint(0);
            current->configuration ()->uniformlySample (current->rankInConfiguration(), config);
        }
        device->currentConfiguration (config);
        device->computeForwardKinematics();
        result.push_back(Sample(clone, effectorClone, config.segment(startRank_, length_), offset,limbOffset, i));
    }
    return result;
}
