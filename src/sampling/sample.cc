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

using namespace hpp;
using namespace hpp::model;
using namespace hpp::rbprm;
using namespace hpp::rbprm::sampling;

model::JointPtr_t GetEffector(const model::JointPtr_t limb)
{
    Joint* current = limb;
    while(current->numberChildJoints() !=0)
    {
        assert(current->numberChildJoints() ==1);
        current = current->childJoint(0);
    }
    return current;
}

std::size_t ComputeLength(const model::JointPtr_t limb)
{
    std::size_t start = limb->rankInConfiguration();
    const Joint* current = GetEffector(limb);
    std::size_t end = current->rankInConfiguration()
            + current->neutralConfiguration().rows();
    return end - start;
}


fcl::Vec3f ComputeEffectorPosition(const model::JointPtr_t limb)
{
    const Joint* current = GetEffector(limb);
    return current->currentTransformation().getTranslation();
}

Eigen::MatrixXd Jacobian(const model::JointPtr_t limb)
{
    const Joint* current = GetEffector(limb);
    return current->jacobian().block(0,limb->rankInVelocity(),6, current->rankInVelocity() - limb->rankInVelocity());
}

Sample::Sample(const model::JointPtr_t limb)
    : startRank_(limb->rankInConfiguration())
    , length_ (ComputeLength(limb))
    , configuration_ (limb->robot()->currentConfiguration().segment(startRank_, length_))
    , effectorPosition_(ComputeEffectorPosition(limb))
    , jacobian_(Jacobian(limb))
    , jacobianProduct_(jacobian_*jacobian_.transpose())
{
    // NOTHING
}

Sample::Sample(const model::JointPtr_t limb, const model::Configuration_t& configuration)
    : startRank_(limb->rankInConfiguration())
    , length_ (ComputeLength(limb))
    , configuration_ (configuration)
    , effectorPosition_(ComputeEffectorPosition(limb))
    , jacobian_(Jacobian(limb))
    , jacobianProduct_(jacobian_*jacobian_.transpose())
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
{
    // NOTHING
}

std::deque<Sample> hpp::rbprm::sampling::GenerateSamples(const model::JointPtr_t model,  const std::size_t nbSamples)
{
    std::deque<Sample> result;
    model::DevicePtr_t device(model->robot()->clone());
    Configuration_t config = device->currentConfiguration();
    JointPtr_t clone = device->getJointByName(model->name());
    std::size_t startRank_(model->rankInConfiguration());
    std::size_t length_ (ComputeLength(model));
    for(std::size_t i = 0; i< nbSamples; ++i)
    {
        clone->configuration ()->uniformlySample (clone->rankInConfiguration (), config);
        Joint* current = clone;
        while(current->numberChildJoints() !=0)
        {
            current = current->childJoint(0);
            current->configuration ()->uniformlySample (current->rankInConfiguration(), config);
        }
        std::cout << "config" << config << std::endl;
        device->currentConfiguration (config);
        device->computeForwardKinematics();
        result.push_back(Sample(clone, config.segment(startRank_, length_)));
    }
    return result;
}
