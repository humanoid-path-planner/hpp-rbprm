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


fcl::Vec3f ComputeEffectorPosition(const model::JointPtr_t /*limb*/, const model::JointPtr_t effector, const fcl::Vec3f& offset)
{
    const fcl::Transform3f& transform = effector->currentTransformation();
    return transform.getTranslation() + transform.getRotation() * offset ;
    //fcl::Transform3f parentT = limb->parentJoint()->currentTransformation();
    //fcl::Matrix3f rot = parentT.getRotation();
    //return (transform.getTranslation() + offset) - parentT.getTranslation();
}

Eigen::MatrixXd Jacobian(const model::JointPtr_t limb, const model::JointPtr_t effector)
{
    return effector->jacobian().block(0,limb->rankInVelocity(),6, effector->rankInVelocity() - limb->rankInVelocity());
}


double Manipulability(const Eigen::MatrixXd& product)
{
    double det = product.determinant();
    return det > 0 ? sqrt(det) : 0;
}

Sample::Sample(const model::JointPtr_t limb, const model::JointPtr_t effector, const fcl::Vec3f& offset, std::size_t id)
    : startRank_(limb->rankInConfiguration())
    , length_ (ComputeLength(limb, effector))
    , configuration_ (limb->robot()->currentConfiguration().segment(startRank_, length_))
    , effectorPosition_(ComputeEffectorPosition(limb, effector,offset))
    , jacobian_(Jacobian(limb, effector))
    , jacobianProduct_(jacobian_*jacobian_.transpose())
    , id_(id)
    , manipulability_(Manipulability(jacobianProduct_))
{
    // NOTHING
}

Sample::Sample(const model::JointPtr_t limb, const model::JointPtr_t effector, model::ConfigurationIn_t configuration,  const fcl::Vec3f& offset, std::size_t id)
    : startRank_(limb->rankInConfiguration())
    , length_ (ComputeLength(limb, effector))
    , configuration_ (configuration)
    , effectorPosition_(ComputeEffectorPosition(limb,effector,offset))
    , jacobian_(Jacobian(limb,effector))
    , jacobianProduct_(jacobian_*jacobian_.transpose())
    , id_(id)
    , manipulability_(Manipulability(jacobianProduct_))
{
    // NOTHING
}

Sample::Sample(const Sample &clone)

    : startRank_(clone.startRank_)
    , length_ (clone.length_)
    , configuration_ (clone.configuration_)
    , effectorPosition_(clone.effectorPosition_)
    , jacobian_(clone.jacobian_)
    , jacobianProduct_(clone.jacobianProduct_)
    , id_(clone.id_)
    , manipulability_(clone.manipulability_)
{
    // NOTHING
}

void hpp::rbprm::sampling::Load(const Sample& sample, ConfigurationOut_t configuration)
{
    configuration.segment(sample.startRank_, sample.length_) = sample.configuration_;
}

std::deque<Sample> hpp::rbprm::sampling::GenerateSamples(const model::JointPtr_t model, const std::string& effector
                                                         , const std::size_t nbSamples, const fcl::Vec3f& offset)
{
    std::deque<Sample> result;
    model::DevicePtr_t device(model->robot()->clone());
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
        result.push_back(Sample(clone, effectorClone, config.segment(startRank_, length_), offset, i));
    }
    return result;
}
