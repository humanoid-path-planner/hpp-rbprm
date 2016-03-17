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

#include <hpp/rbprm/sampling/analysis.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/model/joint-configuration.hh>
#include <hpp/model/joint.hh>
#include <time.h>

#include <Eigen/Eigen>

using namespace hpp;
using namespace hpp::model;
using namespace hpp::rbprm;
using namespace hpp::rbprm::sampling;

namespace
{
    double manipulability(const SampleDB& /*sampleDB*/, const sampling::Sample& sample)
    {
        double det = sample.jacobianProduct_.determinant();
        return det > 0 ? sqrt(det) : 0;
    }

    double isotropy(const SampleDB& /*sampleDB*/, const sampling::Sample& sample)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svdOfJ(sample.jacobian_);
        const Eigen::VectorXd S = svdOfJ.singularValues();
        double min = std::numeric_limits<double>::max();
        double max = 0;
        for(int i =0; i < S.rows();++i)
        {
            double v = S[i];
            if(v < min) min = v;
            if(v > max) max = v;
        }
        return  (max > 0) ? 1 - sqrt(1 - min*min / max* max) : 0;
    }    

    double minSing(const SampleDB& /*sampleDB*/, const sampling::Sample& sample)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svdOfJ(sample.jacobian_);
        const Eigen::VectorXd S = svdOfJ.singularValues();
        double min = std::numeric_limits<double>::max();
        for(int i =0; i < S.rows();++i)
        {
            double v = S[i];
            if(v < min) min = v;
        }
        return  min;
    }


    struct FullBodyDB
    {
        std::vector<model::Configuration_t> fullBodyConfigs_;
        static FullBodyDB* instance_;

        static FullBodyDB& Instance(model::DevicePtr_t device, std::size_t nbSamples = 10000)
        {
            if(!instance_)
                instance_ = new FullBodyDB;
            if (instance_->fullBodyConfigs_.size() < nbSamples)
                instance_->GenerateFullBodyDB(nbSamples, device);
            return *instance_;
        }


        void GenerateFullBodyDB(std::size_t nbSamples, model::DevicePtr_t device)
        {
            core::BasicConfigurationShooterPtr_t shooter = core::BasicConfigurationShooter::create(device);
            model::Configuration_t save(device->currentConfiguration());
            std::size_t i = nbSamples - fullBodyConfigs_.size();
            while(i>0)
            {
                model::ConfigurationPtr_t conf = shooter->shoot();
                device->currentConfiguration(*conf);
                device->computeForwardKinematics();
                if (!device->collisionTest())
                {
                    fullBodyConfigs_.push_back(*conf);
                    --i;
                }
            }
            device->currentConfiguration(save);
            device->computeForwardKinematics();
        }
    };

    FullBodyDB* FullBodyDB::instance_ = new FullBodyDB;

    // computing probability of auto collision given a large number of full body samples
    double selfCollisionProbability(rbprm::RbPrmFullBodyPtr_t fullBody , const SampleDB& /*sampleDB*/, const sampling::Sample& sample)
    {
        model::DevicePtr_t device = fullBody->device_;
        model::Configuration_t save(device->currentConfiguration());
        FullBodyDB& fullBodyDB = FullBodyDB::Instance(device);
        std::size_t totalSamples = fullBodyDB.fullBodyConfigs_.size(), totalNoCollisions =  0;
        for(std::vector<model::Configuration_t>::const_iterator cit = fullBodyDB.fullBodyConfigs_.begin();
            cit != fullBodyDB.fullBodyConfigs_.end(); ++cit)
        {
            model::Configuration_t conf = *cit;
            sampling::Load(sample,conf);
            device->currentConfiguration(conf);
            device->computeForwardKinematics();
            if (!device->collisionTest())
                ++totalNoCollisions;
        }
        device->currentConfiguration(save);
        device->computeForwardKinematics();
        return (double)(totalNoCollisions) / (double)(totalSamples);
    }

    void distanceRec(const ConfigurationIn_t conf, const std::string& lastJoint, model::JointPtr_t currentJoint, double& currentDistance)
    {
        model::size_type rk = currentJoint->rankInConfiguration();
        model::value_type lb = currentJoint->lowerBound(0), ub = currentJoint->upperBound(0);
        model::value_type val = conf[rk];
        val= (val - lb) * (ub - val) / ((ub - lb) * (ub - lb));
        currentDistance = std::min(currentDistance, val);
        if(lastJoint == currentJoint->name())
            return;
        else return distanceRec(conf, lastJoint, currentJoint->childJoint(0),currentDistance);
    }

    double distanceToLimits(rbprm::RbPrmFullBodyPtr_t fullBody , const SampleDB& /*sampleDB*/, const sampling::Sample& sample)
    {
        // find limb name
        rbprm::T_Limb::const_iterator cit = fullBody->GetLimbs().begin();
        for(; cit != fullBody->GetLimbs().end(); ++cit)
        {
            if(cit->second->limb_->rankInConfiguration() == sample.startRank_)
                break;
        }
        if(cit == fullBody->GetLimbs().end())
        {
            throw std::runtime_error ("Impossible to match sample with a limb");
        }
        model::DevicePtr_t device = fullBody->device_;
        model::Configuration_t conf(device->currentConfiguration());
        double distance = 1; //std::numeric_limits<double>::max();
        sampling::Load(sample,conf);
        distanceRec(conf, cit->second->effector_->name(), cit->second->limb_, distance);
        distance = 1 - exp(-5*distance);
        return distance;
    }
}


AnalysisFactory::AnalysisFactory(hpp::rbprm::RbPrmFullBodyPtr_t device)
    : device_(device)
{
    evaluate_.insert(std::make_pair("manipulability", &manipulability));
    evaluate_.insert(std::make_pair("isotropy", &isotropy));
    evaluate_.insert(std::make_pair("minimumSingularValue", &minSing));
    evaluate_.insert(std::make_pair("selfCollisionProbability", boost::bind(&selfCollisionProbability, boost::ref(device_), _1, _2)));
    evaluate_.insert(std::make_pair("jointLimitsDistance", boost::bind(&distanceToLimits, boost::ref(device_), _1, _2)));
}

AnalysisFactory::~AnalysisFactory(){}

bool AnalysisFactory::AddAnalysis(const std::string& name, const evaluate func)
{
    if(evaluate_.find(name) != evaluate_.end())
        return false;
    evaluate_.insert(std::make_pair(name,func));
    return true;
}
