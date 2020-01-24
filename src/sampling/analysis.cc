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
#include <hpp/core/configuration-shooter/uniform.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/configuration.hh>
#include <time.h>
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <boost/bind.hpp>

using namespace hpp;
using namespace hpp::pinocchio;
using namespace hpp::rbprm;
using namespace hpp::rbprm::sampling;

namespace
{
    enum JacobianMode
    {
      ALL           = 0,  // Jacobian is entirely considered
      ROTATION      = 1,  // Only rotational jacobian is considered
      TRANSLATION   = 2   // Only translational jacobian is considered
    };

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(const sampling::Sample& sample, const JacobianMode mode)
    {
        switch (mode) {
        case ALL:
            return Eigen::JacobiSVD<Eigen::MatrixXd>(sample.jacobian_);
        case TRANSLATION:
            return Eigen::JacobiSVD<Eigen::MatrixXd>(sample.jacobian_.block(0,0,3,sample.jacobian_.cols()));
        case ROTATION:
            return Eigen::JacobiSVD<Eigen::MatrixXd>(sample.jacobian_.block(3,0,3,sample.jacobian_.cols()));
        default:
            throw std::runtime_error ("Can not perform SVD on subjacobian, unknown JacobianMode");
            break;
        }
    }

    double manipulability(const JacobianMode mode, const SampleDB& /*sampleDB*/, const sampling::Sample& sample)
    {
        double det;
        Eigen::MatrixXd sub;
        switch (mode) {
        case ALL:
            det = sample.jacobianProduct_.determinant();
            break;
        case TRANSLATION:
            sub = sample.jacobian_.block(0,0,3,sample.jacobian_.cols());
            det = (sub*sub.transpose()).determinant();
            break;
        case ROTATION:
            sub = sample.jacobian_.block(3,0,3,sample.jacobian_.cols());
            det = (sub*sub.transpose()).determinant();
            break;
        default:
            throw std::runtime_error ("Can not compute subjacobian, unknown JacobianMode");
            break;
        }
        return det > 0 ? sqrt(det) : 0;
    }

    double isotropy(const JacobianMode mode, const SampleDB& /*sampleDB*/, const sampling::Sample& sample)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svdOfJ(svd(sample, mode));
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

    double minSing(const JacobianMode mode, const SampleDB& /*sampleDB*/, const sampling::Sample& sample)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svdOfJ(svd(sample, mode));
        const Eigen::VectorXd S = svdOfJ.singularValues();
        double min = std::numeric_limits<double>::max();
        for(int i =0; i < S.rows();++i)
        {
            double v = S[i];
            if(v < min) min = v;
        }
        return  min;
    }

    double maxSing(const JacobianMode mode, const SampleDB& /*sampleDB*/, const sampling::Sample& sample)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svdOfJ(svd(sample, mode));
        const Eigen::VectorXd S = svdOfJ.singularValues();
        double max = -std::numeric_limits<double>::max();
        for(int i =0; i < S.rows();++i)
        {
            double v = S[i];
            if(v > max) max = v;
        }
        return  max;
    }


    struct FullBodyDB
    {
        std::vector<hpp::pinocchio::Configuration_t> fullBodyConfigs_;
        static FullBodyDB* instance_;

        static FullBodyDB& Instance(hpp::pinocchio::DevicePtr_t device, std::size_t nbSamples = 10000)
        {
            if(!instance_)
                instance_ = new FullBodyDB;
            if (instance_->fullBodyConfigs_.size() < nbSamples)
                instance_->GenerateFullBodyDB(nbSamples, device);
            return *instance_;
        }


        void GenerateFullBodyDB(std::size_t nbSamples, hpp::pinocchio::DevicePtr_t device)
        {
            core::configurationShooter::UniformPtr_t shooter = core::configurationShooter::Uniform::create(device);
            hpp::pinocchio::Configuration_t save(device->currentConfiguration());
            core::CollisionValidationPtr_t colVal = core::CollisionValidation::create(device);
            std::size_t i = nbSamples - fullBodyConfigs_.size();
            hpp::pinocchio::Configuration_t conf;
            while(i>0)
            {
                shooter->shoot(conf);
                device->currentConfiguration(conf);
                device->computeForwardKinematics();
                core::ValidationReportPtr_t colRep(new core::CollisionValidationReport);

                if (colVal->validate(conf,colRep))
                {
                    fullBodyConfigs_.push_back(conf);
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
        hpp::pinocchio::DevicePtr_t device = fullBody->device_;
        hpp::pinocchio::Configuration_t save(device->currentConfiguration());
        FullBodyDB& fullBodyDB = FullBodyDB::Instance(device);
        core::CollisionValidationPtr_t colVal = core::CollisionValidation::create(device);
        std::size_t totalSamples = fullBodyDB.fullBodyConfigs_.size(), totalNoCollisions =  0;
        for(std::vector<hpp::pinocchio::Configuration_t>::const_iterator cit = fullBodyDB.fullBodyConfigs_.begin();
            cit != fullBodyDB.fullBodyConfigs_.end(); ++cit)
        {
            hpp::pinocchio::Configuration_t conf = *cit;
            sampling::Load(sample,conf);
            device->currentConfiguration(conf);
            device->computeForwardKinematics();
            core::ValidationReportPtr_t colRep(new core::CollisionValidationReport);

            if (colVal->validate(conf,colRep))
                ++totalNoCollisions;
        }
        device->currentConfiguration(save);
        device->computeForwardKinematics();
        return (double)(totalNoCollisions) / (double)(totalSamples);
    }

    void distanceRec(const ConfigurationIn_t conf, const std::string& lastJoint, hpp::pinocchio::JointPtr_t currentJoint, double& currentDistance)
    {
        hpp::pinocchio::size_type rk = currentJoint->rankInConfiguration();
        if(currentJoint->configSize() > 0 && currentJoint->isBounded(0))
        {
            hpp::pinocchio::value_type lb = currentJoint->lowerBound(0), ub = currentJoint->upperBound(0);
            hpp::pinocchio::value_type val = conf[rk];
            val= (val - lb) * (ub - val) / ((ub - lb) * (ub - lb));
            currentDistance = std::min(currentDistance, val);
        }
        if(lastJoint == currentJoint->name() || currentJoint->numberChildJoints() == 0)
            return;
        else return distanceRec(conf, lastJoint, currentJoint->childJoint(0),currentDistance);
    }

    rbprm::RbPrmLimbPtr_t getLimbFromStartRank(size_type startRank,rbprm::RbPrmFullBodyPtr_t fullBody  ){
      rbprm::T_Limb::const_iterator cit = fullBody->GetLimbs().begin();
      for(; cit != fullBody->GetLimbs().end(); ++cit)
      {
          if(cit->second->limb_->rankInConfiguration() == startRank)
              break;
      }
      if(cit == fullBody->GetLimbs().end())
      {
        for(cit = fullBody->GetNonContactingLimbs().begin(); cit != fullBody->GetNonContactingLimbs().end(); ++cit)
        {
            if(cit->second->limb_->rankInConfiguration() == startRank)
                break;
        }
        if(cit == fullBody->GetNonContactingLimbs().end())
        {
            throw std::runtime_error ("Impossible to match sample with a limb");
        }
      }
      return cit->second;
    }


    double distanceToLimits(rbprm::RbPrmFullBodyPtr_t fullBody , const SampleDB& /*sampleDB*/, const sampling::Sample& sample)
    {
        // find limb name
        rbprm::RbPrmLimbPtr_t limb = getLimbFromStartRank(sample.startRank_,fullBody);
        hpp::pinocchio::DevicePtr_t device = fullBody->device_;
        hpp::pinocchio::Configuration_t conf(device->currentConfiguration());
        double distance = 1; //std::numeric_limits<double>::max();
        sampling::Load(sample,conf);
        distanceRec(conf, limb->effector_.name(), limb->limb_, distance);
        distance = 1 - exp(-5*distance);
        return distance;
    }

  /** compute default weight vector : (TODO add an API to set custom weight vector)
  * FIXME : lot of assumptions are made here, but they are true for the most common robots used
  * * the robot's limbs only contain revolute joint
  * * the robot's root is oriented with x forward and z up (ie. the most common direction of deplacement is along x)

  * by looking at the jacobian of each joint we can check around wich axis the joint is.
  * We set the weight depending on the axis of each joint :
  * y : used to move forward, low weight
  * z : use to turn, medium weight
  * x : only used for less common behaviour like straffing, high weight
  * prismatic joint ?? high weight
   */
    Configuration_t computeWeightsFromAxis(rbprm::RbPrmLimbPtr_t limb,hpp::pinocchio::DevicePtr_t device){
      Configuration_t weights = Configuration_t::Zero(limb->effector_.joint()->rankInVelocity() - limb->limb_->rankInVelocity()+1);
      size_t i_weight = 0;
      for (size_type i = limb->limb_->rankInVelocity() ; i <= limb->effector_.joint()->rankInVelocity() ; ++i){
        hpp::pinocchio::vector_t jointJacobian= device->getJointAtVelocityRank(i)->jacobian().block<6,1>(0,i).transpose();
        //hppDout(notice,"Jacobian of joint "<<device->getJointAtVelocityRank(i)->name()<<" at id = "<<i);
        //hppDout(notice,"joint column : \n"<<jointJacobian);
        if(fabs(jointJacobian[4]) > 0.5){ // rot y
          weights[i_weight]=1.;
        }else if(fabs(jointJacobian[5]) > 0.5){ // rot z
          weights[i_weight]=10.;
        }else{ // prismatic or rot x
          weights[i_weight]=100.;
        }
        i_weight++;
      }
      return weights;
    }

    /**
     * @brief referenceConfiguration
     * @param fullBody
     * @param sample
     * @param weights vector of size 3, value respectively for x,y,z, rotations
     * @return
     */
    double referenceConfiguration(rbprm::RbPrmFullBodyPtr_t fullBody , const SampleDB& /*sampleDB*/, const sampling::Sample& sample){
      // find limb name
      rbprm::RbPrmLimbPtr_t limb = getLimbFromStartRank(sample.startRank_,fullBody);
      hpp::pinocchio::DevicePtr_t device = fullBody->device_;
      hpp::pinocchio::Configuration_t conf(device->currentConfiguration());
      sampling::Load(sample,conf); // retrieve the configuration of the sample (only for the concerned limb)
      //hppDout(notice,"Reference conf in analysis : "<<pinocchio::displayConfig(fullBody->referenceConfig()));
      double distance = 0;
      Configuration_t diff(device->numberDof());
      Configuration_t weights;
      if(fullBody->postureWeights().size() == fullBody->device_->numberDof()){
        weights = fullBody->postureWeights().segment(limb->limb_->rankInVelocity(),limb->effector_.joint()->rankInVelocity() - limb->limb_->rankInVelocity()+1);
        //hppDout(notice,"Analysis : posture weight defined in fullbody : "<<hpp::pinocchio::displayConfig(weights));
      }else{
        weights = computeWeightsFromAxis(limb,device);
        //hppDout(notice,"Analysis : posture weight not defined in fullbody, computed weight : "<<hpp::pinocchio::displayConfig(weights));
      }
      const int nq = device->configSize() - device->extraConfigSpace().dimension();
      hpp::pinocchio::difference (device, conf.segment(0,nq), fullBody->referenceConfig().segment(0,nq), diff);
     // hppDout(notice,"Reference config in analysis : "<<pinocchio::displayConfig(fullBody->referenceConfig()));
      // the difference vector depend on the index in the velocity vector, not in the configuration
      // we only sum for the index of the current limb
     // hppDout(notice,"ref config rank: "<<cit->second->limb_->rankInVelocity()<<" ; "<<cit->second->effector_->rankInVelocity());
      for (size_type i = limb->limb_->rankInVelocity() ; i <= limb->effector_.joint()->rankInVelocity() ; ++i){
        distance += (diff[i]*diff[i])*weights[i-limb->limb_->rankInVelocity()];
      }
      // This is an heuristic and not a cost, a null distance is the best result
      // TODO : replace hardcoded value with the real max
      // but it increase computation time, and the values will be normalized after anyways ..
      //hppDout(notice,"distance to ref = "<<sqrt(distance));
      if(sqrt(distance)>=100){
        hppDout(error,"WARNING : max distance to config not big enough");
      }
      return 100-(sqrt(distance));
    }

}


AnalysisFactory::AnalysisFactory(hpp::rbprm::RbPrmFullBodyPtr_t device)
    : device_(device)
{
    evaluate_.insert(std::make_pair("manipulability", boost::bind(&manipulability, JacobianMode(0), _1, _2)));
    evaluate_.insert(std::make_pair("isotropy", boost::bind(&isotropy, JacobianMode(0), _1, _2)));
    evaluate_.insert(std::make_pair("minimumSingularValue", boost::bind(&minSing, JacobianMode(0), _1, _2)));
    evaluate_.insert(std::make_pair("maximumSingularValue", boost::bind(&maxSing, JacobianMode(0), _1, _2)));

    evaluate_.insert(std::make_pair("manipulabilityRot", boost::bind(&manipulability, JacobianMode(1), _1, _2)));
    evaluate_.insert(std::make_pair("isotropyRot", boost::bind(&isotropy, JacobianMode(1), _1, _2)));
    evaluate_.insert(std::make_pair("minimumSingularValueRot", boost::bind(&minSing, JacobianMode(1), _1, _2)));
    evaluate_.insert(std::make_pair("maximumSingularValueRot", boost::bind(&maxSing, JacobianMode(1), _1, _2)));

    evaluate_.insert(std::make_pair("manipulabilityTr", boost::bind(&manipulability, JacobianMode(2), _1, _2)));
    evaluate_.insert(std::make_pair("isotropyTr", boost::bind(&isotropy, JacobianMode(2), _1, _2)));
    evaluate_.insert(std::make_pair("minimumSingularValueTr", boost::bind(&minSing, JacobianMode(2), _1, _2)));
    evaluate_.insert(std::make_pair("maximumSingularValueTr", boost::bind(&maxSing, JacobianMode(2), _1, _2)));

    evaluate_.insert(std::make_pair("selfCollisionProbability", boost::bind(&selfCollisionProbability, boost::ref(device_), _1, _2)));
    evaluate_.insert(std::make_pair("jointLimitsDistance", boost::bind(&distanceToLimits, boost::ref(device_), _1, _2)));
    evaluate_.insert(std::make_pair("ReferenceConfiguration", boost::bind(&referenceConfiguration, boost::ref(device_), _1, _2)));

}

AnalysisFactory::~AnalysisFactory(){}

bool AnalysisFactory::AddAnalysis(const std::string& name, const evaluate func)
{
    if(evaluate_.find(name) != evaluate_.end())
        return false;
    evaluate_.insert(std::make_pair(name,func));
    return true;
}
