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

#include <hpp/rbprm/rbprm-shooter.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/core/collision-validation.hh>

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

    void SetConfigTranslation(ConfigurationPtr_t config, const Vec3f& translation)
    {
        for(int i =0; i<3; ++i)
        {
            (*config)(i)=translation[i];
        }
    }

    void Translate(ConfigurationPtr_t config, const Vec3f& translation)
    {
        for(int i =0; i<3; ++i)
        {
            (*config)(i)+=translation[i];
        }
    }

    void SetConfigRotation(ConfigurationPtr_t config, const Vec3f& rotation)
    {
        for(int i =0; i<3; ++i)
        {
            (*config)(i+3)=rotation[i];
        }
    }

    void SampleRotation(ConfigurationPtr_t config, JointVector_t& jv)
    {
        JointPtr_t joint = jv[1];
        std::size_t rank = joint->rankInConfiguration ();
        joint->configuration ()->uniformlySample (rank, *config);
    }

    void SetConfig6D(ConfigurationPtr_t config, const Vec3f& translation, const Vec3f& rotation)
    {
        SetConfigTranslation(config, translation);
        SetConfigRotation(config, rotation);
    }

} // namespace

  namespace rbprm {
    RbPrmShooterPtr_t RbPrmShooter::create (const model::RbPrmDevicePtr_t& robot,
                                            const T_CollisionObject& geometries,
                                            rbprm::RbPrmValidationPtr_t& validator,
                                            const std::size_t shootLimit,
                                            const std::size_t displacementLimit)
    {
        RbPrmShooter* ptr = new RbPrmShooter (robot, geometries, validator, shootLimit, displacementLimit);
        RbPrmShooterPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
    }

    void RbPrmShooter::init (const RbPrmShooterPtr_t& self)
    {
        ConfigurationShooter::init (self);
        weak_ = self;
    }
// TODO: outward

  RbPrmShooter::RbPrmShooter (const model::RbPrmDevicePtr_t& robot,
                              const T_CollisionObject& geometries,
                              rbprm::RbPrmValidationPtr_t& validator,
                              const std::size_t shootLimit,
                              const std::size_t displacementLimit)
    : shootLimit_(shootLimit)
    , displacementLimit_(displacementLimit)
    , robot_ (robot)
    , validator_(validator)
  {
      this->InitWeightedTriangles(geometries);
  }

  void RbPrmShooter::InitWeightedTriangles(const rbprm::T_CollisionObject& geometries)
  {
      double sum = 0;
      for(rbprm::T_CollisionObject::const_iterator objit = geometries.begin();
          objit != geometries.end(); ++objit)
      {
          BVHModelOBConst_Ptr_t model =  GetModel(*objit); // TODO NOT TRIANGLES
          for(int i =0; i < model->num_tris; ++i)
          {
              TrianglePoints tri;
              Triangle fcltri = model->tri_indices[i];
              tri.p1 =model->vertices[fcltri[0]];
              tri.p2 =model->vertices[fcltri[1]];
              tri.p3 =model->vertices[fcltri[2]];
              double weight = TriangleArea(tri);
              sum += weight;
              weights_.push_back(weight);
              // TODO COMPUTE NORMALS
              fcl::Vec3f normal = (tri.p3 - tri.p1).cross(tri.p2 - tri.p1);
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
          if(*wit <= r) return *trit;
      }
      return triangles_[triangles_.size()-1]; // not supposed to happen
  }

hpp::core::ConfigurationPtr_t RbPrmShooter::shoot () const
{
    JointVector_t jv = robot_->getJointVector ();
    ConfigurationPtr_t config (new Configuration_t (robot_->configSize()));
    std::size_t limit = shootLimit_;
    //std::size_t limit2 = 100; // TODO PARAMETERIZE ?
    while(limit >0)
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
        SetConfigTranslation(config, p);
        // rotate and translate randomly until valid configuration found or
        // no obstacle is reachable
        CollisionValidationReport report;
        bool found(false);
        std::size_t limitDis = displacementLimit_;
        while(!found && limitDis >0)
        {
            if(validator_->trunkValidation_->validate(*config, report)
            && !validator_->romValidation_->validate(*config))
            {
                found = true;
            }
            else if(!report.result.isCollision())
            {
                // try to rotate to reach rom
                for(std::size_t i =0; i< limitDis && !found; ++i, --limitDis)
                {
                    SampleRotation(config, jv);
                    found = validator_->validate(*config);
                }
                if(!found) break;
            }
            else // move out of collision
            {
                // retrieve Contact information
                const Vec3f& direction = report.result.getContact(0).normal;
                // v0 move away from normal
                Translate(config, -direction * report.result.getContact(0).penetration_depth +
                 ((double) rand() / (RAND_MAX)));
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
            (*config) [offset + i] = (upper - lower) * rand ()/RAND_MAX;
        }
        if(found)
        {
        std::cout << "nb trials " << this->shootLimit_ - limit << std::endl;
            return config;
        }
        limit--;
    }
    return config;
}


  }// namespace rbprm
}// namespace hpp
