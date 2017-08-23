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

#include <hpp/core/collision-validation-report.hh>
#include <hpp/rbprm/rbprm-shooter.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/model/joint.hh>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/core/collision-validation.hh>
#include <Eigen/Geometry>
#include <hpp/model/configuration.hh>
#include <hpp/util/timer.hh>

namespace hpp {
using namespace core;
using namespace fcl;
namespace
{
    typedef fcl::BVHModel<OBBRSS> BVHModelOB;
    typedef boost::shared_ptr<const BVHModelOB> BVHModelOBConst_Ptr_t;
		
    BVHModelOBConst_Ptr_t GetModel(const fcl::CollisionObjectConstPtr_t object)
    {
        assert(object->collisionGeometry()->getNodeType() == BV_OBBRSS);
        const BVHModelOBConst_Ptr_t model = boost::static_pointer_cast<const BVHModelOB>(object->collisionGeometry());
        assert(model->getModelType() == BVH_MODEL_TRIANGLES);
        return model;
    }

    double TriangleArea(rbprm::TrianglePoints& tri)
    {
        double a, b, c;
        a = (tri.p1 - tri.p2).norm();
        b = (tri.p2 - tri.p3).norm();
        c = (tri.p3 - tri.p1).norm();
        double s = 0.5 * (a + b + c);
        return sqrt(s * (s-a) * (s-b) * (s-c));
    }

    std::vector<double> getTranslationBounds(const model::RbPrmDevicePtr_t robot)
    {
        const JointPtr_t root = robot->Device::rootJoint();
        std::vector<double> res;
        for(std::size_t i =0; i<3; ++i)
        {
            if(root->isBounded(i))
            {
                res.push_back(root->lowerBound(i));
                res.push_back(root->upperBound(i));
            }
            else            {

                res.push_back(-std::numeric_limits<double>::max());
                res.push_back(std::numeric_limits<double>::max());
            }
        }
        return res;
    }

    void SetConfigTranslation(const model::RbPrmDevicePtr_t robot, ConfigurationPtr_t config, const Vec3f& translation)
    {
        std::vector<double> bounds = getTranslationBounds(robot);
        for(std::size_t i =0; i<3; ++i)
        {
            (*config)(i)= std::min(bounds[2*i+1], std::max(bounds[2*i], translation[i]));
        }
    }

    void Translate(const model::RbPrmDevicePtr_t robot, ConfigurationPtr_t config, const Vec3f& translation)
    {
        // bound to positions limits
        std::vector<double> bounds = getTranslationBounds(robot);
        for(int i =0; i<3; ++i)
        {
            (*config)(i)=std::min(bounds[2*i+1], std::max(bounds[2*i], (*config)(i)+translation[i]));
        }
    }

    void SampleRotationRec(ConfigurationPtr_t config, JointVector_t& jv, std::size_t& current)
    {
        JointPtr_t joint = jv[current++];
        std::size_t rank = joint->rankInConfiguration ();
        joint->configuration ()->uniformlySample (rank, *config);
        if(current<jv.size())
            SampleRotationRec(config,jv,current);
    }

    void SampleRotation(model::DevicePtr_t so3, ConfigurationPtr_t config, JointVector_t& jv)
    {
        std::size_t id = 1;
        if(so3->rootJoint())
        {
            Eigen::Matrix <value_type, 3, 1> confso3;
            id+=1;
            model::JointPtr_t joint = so3->rootJoint();
            for(int i =0; i <3; ++i)
            {
                joint->configuration()->uniformlySample (i, confso3);
                joint = joint->numberChildJoints() > 0 ? joint->childJoint(0) : 0;
            }
            Eigen::Quaterniond qt = Eigen::AngleAxisd(confso3(0), Eigen::Vector3d::UnitZ())
              * Eigen::AngleAxisd(confso3(1), Eigen::Vector3d::UnitY())
              * Eigen::AngleAxisd(confso3(2), Eigen::Vector3d::UnitX());
            std::size_t rank = 3;
            (*config)(rank+0) = qt.w();
            (*config)(rank+1) = qt.x();
            (*config)(rank+2) = qt.y();
            (*config)(rank+3) = qt.z();
        }
        if(id < jv.size())
            SampleRotationRec(config,jv,id);
    }

    model::DevicePtr_t initSo3()
    {
        DevicePtr_t so3Robot = model::Device::create("so3Robot");
        /*model::JointPtr_t res = new model::JointSO3(fcl::Transform3f());
        res->name("defaultSO3");*/
        //so3Robot->rootJoint(0);
        return so3Robot;
    }

    void seRotationtLimits (model::DevicePtr_t so3Robot, const std::vector<double>& limitszyx)
    {
        model::Joint* previous = so3Robot->rootJoint();
        if(previous == 0)
        {
            // init joints
            previous = new model::jointRotation::Bounded(fcl::Transform3f());
            model::Joint * jy = new model::jointRotation::Bounded(fcl::Transform3f());
            model::Joint * jx = new model::jointRotation::Bounded(fcl::Transform3f());
            so3Robot->rootJoint(previous);
            previous->addChildJoint (jy);
            jy->addChildJoint (jx);
            previous->name("so3z");
            jy->name("so3y");
            jx->name("so3x");
        }
        // set limits model::JointPtr_t
        assert(limitszyx.size() == 6);
        int i = 0;
        model::JointPtr_t current = previous;
        for(std::vector<double>::const_iterator cit = limitszyx.begin();
            cit != limitszyx.end(); ++cit, ++i)
        {
            if(i % 2 == 0)
            {
                current->lowerBound(0, *cit);
            }
            else
            {
                current->upperBound(0, *cit);
                current = current->numberChildJoints() > 0 ? current->childJoint(0) : 0;
            }
        }
    }

} // namespace

  namespace rbprm {

    RbPrmShooterPtr_t RbPrmShooter::create (const model::RbPrmDevicePtr_t& robot,
                                            const ObjectVector_t& geometries,
																						const affMap_t& affordances,
                                            const std::vector<std::string>& filter,
                                            const std::map<std::string, std::vector<std::string> >& affFilters,
                                            const std::size_t shootLimit, const std::size_t displacementLimit)
    {
      unsigned int seed = (unsigned int)(time(NULL));
      // seed = 1484926006 ;
      //seed = 1485276927;
      //seed = 1485359407; //darpa
      // seed = 1485441926 ; // prepare_jump
      // seed = 1485973442 ; // stair bauzil (old)
      // seed = 1486137883; // stair bauzil (new, better) interpolation don't work, too far from ramp
     //  seed = 1486140324; // stair bauzil NOT WORKING ??
    // seed = 1486140446; // stair bauzil : work with extract path but not the interpolation (best on yet)
    //  seed = 1486403996; // sideWall
     //  seed = 1486557940; // downSlope, lent ( + 5min mais fonctionne) (minz = 0.3)
       // seed = 1486560453; // downSlop minz = 0.45 (20min)
      // seed = 1486564172 ; // downSlop minz = 0.45 (20min)
      //seed = 1486643780; // downSlope HyQ_large
      // seed = 1486657707 ; // downSlope hrp2
      // seed = 1487238007; // slalom hyq v=1
     //seed = 1487348584 ; // hyq slalom v = 0.5
     // seed = 1488288253; // detour  kino
      //seed = 1488449318; // downSLope (close to ground ... )
      //seed = 1492696043; // bug stairs
      //seed = 149277557 ; // stairs (work)
      //seed = 1502899400; // slalom hrp2 (v = 1 ; a = 5 )
      seed = 1503410641; // slalom hrp2 (v = 0.3 ; a = 0.5 ) (not a lot of clearance)
      srand (seed);
      hppDout(notice,"&&&&&& SEED = "<<seed);
      std::cout<<"seed = "<<seed<<std::endl;
      RbPrmShooter* ptr = new RbPrmShooter (robot, geometries, affordances,
                                            filter, affFilters, shootLimit, displacementLimit);


      RbPrmShooterPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    void RbPrmShooter::init (const RbPrmShooterPtr_t& self)
    {
        ConfigurationShooter::init (self);
        weak_ = self;
    }


    void RbPrmShooter::BoundSO3(const std::vector<double>& limitszyx)
    {
        seRotationtLimits(eulerSo3_, limitszyx);
    }

// TODO: outward

    RbPrmShooter::RbPrmShooter (const model::RbPrmDevicePtr_t& robot,
                              const ObjectVector_t& geometries,
															const affMap_t& affordances,
                              const std::vector<std::string>& filter,
                              const std::map<std::string, std::vector<std::string> >& affFilters,
                              const std::size_t shootLimit,
                              const std::size_t displacementLimit)
    : shootLimit_(shootLimit)
    , displacementLimit_(displacementLimit)
    , filter_(filter)
    , robot_ (robot)
    , validator_(rbprm::RbPrmValidation::create(robot_, filter, affFilters,
																								affordances, geometries))
    , eulerSo3_(initSo3())
    {
        for(hpp::core::ObjectVector_t::const_iterator cit = geometries.begin();
            cit != geometries.end(); ++cit)
        {
            validator_->addObstacle(*cit);
        }
        this->InitWeightedTriangles(geometries);
		}

    void RbPrmShooter::InitWeightedTriangles(const model::ObjectVector_t& geometries)
    {
        double sum = 0;
        for(model::ObjectVector_t::const_iterator objit = geometries.begin();
          objit != geometries.end(); ++objit)
        {
            const  fcl::CollisionObjectPtr_t& colObj = (*objit)->fcl();
            BVHModelOBConst_Ptr_t model =  GetModel(colObj); // TODO NOT TRIANGLES
            for(int i =0; i < model->num_tris; ++i)
            {
                TrianglePoints tri;
                Triangle fcltri = model->tri_indices[i];
                tri.p1 = colObj->getRotation() * model->vertices[fcltri[0]] + colObj->getTranslation();
                tri.p2 = colObj->getRotation() * model->vertices[fcltri[1]] + colObj->getTranslation();
                tri.p3 = colObj->getRotation() * model->vertices[fcltri[2]] + colObj->getTranslation();
                double weight = TriangleArea(tri);
                sum += weight;
                weights_.push_back(weight);
                fcl::Vec3f normal = (tri.p2 - tri.p1).cross(tri.p3 - tri.p1);
                normal.normalize();
                triangles_.push_back(std::make_pair(normal,tri));
            }
            double previousWeight = 0;
            for(std::vector<double>::iterator wit = weights_.begin();
                wit != weights_.end(); ++wit)
            {
                previousWeight += (*wit) / sum;
                (*wit) = previousWeight;
            }
        }
    }

  const RbPrmShooter::T_TriangleNormal &RbPrmShooter::RandomPointIntriangle() const
  {
      return triangles_[rand() % triangles_.size()];
  }

  const RbPrmShooter::T_TriangleNormal& RbPrmShooter::WeightedTriangle() const
  {
      double r = ((double) rand() / (RAND_MAX));
      std::vector<T_TriangleNormal>::const_iterator trit = triangles_.begin();
      for(std::vector<double>::const_iterator wit = weights_.begin();
          wit != weights_.end();
          ++wit, ++trit)
      {
          if(*wit >= r) return *trit;
      }
      return triangles_[triangles_.size()-1]; // not supposed to happen
  }

hpp::core::ConfigurationPtr_t RbPrmShooter::shoot () const
{
    hppDout(notice,"!!! Random shoot");
    HPP_DEFINE_TIMECOUNTER(SHOOT_COLLISION);
    JointVector_t jv = robot_->getJointVector ();
    ConfigurationPtr_t config (new Configuration_t (robot_->Device::currentConfiguration()));
    std::size_t limit = shootLimit_;
    bool found(false);
    while(limit >0 && !found)
    {
        // pick one triangle randomly
        const T_TriangleNormal* sampled(0);
        double r = ((double) rand() / (RAND_MAX));
        if(r > 0.3)
            sampled = &RandomPointIntriangle();
        else
            sampled = &WeightedTriangle();
				const TrianglePoints& tri = sampled->second;
        //http://stackoverflow.com/questions/4778147/sample-random-point-in-triangle
        double r1, r2;
        r1 = ((double) rand() / (RAND_MAX)); r2 = ((double) rand() / (RAND_MAX));
        Vec3f p = (1 - sqrt(r1)) * tri.p1 + (sqrt(r1) * (1 - r2)) * tri.p2
                + (sqrt(r1) * r2) * tri.p3;

        //set configuration position to sampled point
        SetConfigTranslation(robot_,config, p);
        SampleRotation(eulerSo3_, config, jv);
        // rotate and translate randomly until valid configuration found or
        // no obstacle is reachable
        ValidationReportPtr_t reportShPtr(new CollisionValidationReport);
        std::size_t limitDis = displacementLimit_;
        Vec3f lastDirection(0,0,1);
        while(!found && limitDis >0)
        {
            HPP_START_TIMECOUNTER(SHOOT_COLLISION);
            bool valid = validator_->validateTrunk(*config, reportShPtr);
            found = valid && validator_->validateRoms(*config, filter_,reportShPtr);
            HPP_STOP_TIMECOUNTER(SHOOT_COLLISION);
            CollisionValidationReport* report = static_cast<CollisionValidationReport*>(reportShPtr.get());

            if(valid &!found)
            {
                // try to rotate to reach rom
                for(; limitDis>0 && !found && valid ; --limitDis)
                {
                    SampleRotation(eulerSo3_, config, jv);
                    HPP_START_TIMECOUNTER(SHOOT_COLLISION);
                    found = validator_->validate(*config, filter_);
                    HPP_STOP_TIMECOUNTER(SHOOT_COLLISION);
                    if(!found)
                    {
                        Translate(robot_, config, -lastDirection *
                                  0.2 * ((double) rand() / (RAND_MAX)));
                    }
                    HPP_START_TIMECOUNTER(SHOOT_COLLISION);
                    valid = validator_->validateTrunk(*config, reportShPtr);
                    found = valid && validator_->validateRoms(*config, filter_,reportShPtr);
                    //found = validator_->validate(*config, filter_);
                    HPP_STOP_TIMECOUNTER(SHOOT_COLLISION);
                }
                if(!found) break;
            }
            else if (!valid)// move out of collision
            {
                // retrieve Contact information
                //lastDirection = -report.result.getContact(0).normal;
                // mouve out by penetration depth
                // v0 move away from normal
                //get normal from collision tri
                lastDirection = triangles_[report->result.getContact(0).b2].first;
                Translate(robot_,config, lastDirection *
                          (std::abs(report->result.getContact(0).penetration_depth) +0.03));
                 limitDis--;
            }
        }
        // Shoot extra configuration variables
        size_type extraDim = robot_->extraConfigSpace ().dimension ();
        size_type offset = robot_->configSize () - extraDim;
        for (size_type i=0; i<extraDim; ++i)
        {
            value_type lower = robot_->extraConfigSpace ().lower (i);
            value_type upper = robot_->extraConfigSpace ().upper (i);
            value_type range = upper - lower;
            if ((range < 0) ||
              (range == std::numeric_limits<double>::infinity()))
            {
                std::ostringstream oss
                  ("Cannot uniformy sample extra config variable ");
                oss << i << ". min = " << ", max = " << upper << std::endl;
                throw std::runtime_error (oss.str ());
            }
            (*config) [offset + i] = lower + (upper - lower) * rand ()/RAND_MAX;
        }
        // save the normal (code from Mylène)
       /* if(extraDim >= 3 ){
          size_type index = robot_->configSize() -3;  // rempli toujours les 3 derniers
          for (size_type i=0; i<3; ++i)
            (*config) [index + i] = lastDirection [i];
        }*/
        limit--;
    }
    if (!found) std::cout << "no config found" << std::endl;
    hppDout(info,"shoot : "<<model::displayConfig(*config));
    HPP_DISPLAY_TIMECOUNTER(SHOOT_COLLISION);
    return config;
}


  }// namespace rbprm
}// namespace hpp
