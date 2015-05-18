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

#include "hpp/rbprm/rbprm-shooter.hh"
#include "hpp/model/collision-object.hh"
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/BVH/BVH_model.h>

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
} // namespace

  namespace rbprm {
  RbPrmShooter::RbPrmShooter (const model::RbPrmDevicePtr_t& robot,
                              const T_CollisionObject& geometries,
                              rbprm::RbPrmValidationPtr_t& validator)
    : robot_ (robot)
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
          BVHModelOBConst_Ptr_t model =  GetModel(*objit);
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
              fcl::Vec3f normal(0,1,0);
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
    const DevicePtr_t& robot = robot_->robotTrunk_;
    JointVector_t jv = robot->getJointVector ();
    ConfigurationPtr_t config (new Configuration_t (robot->configSize ()));
    int limit = 10000;
    while(limit >0)
    {
        for (JointVector_t::const_iterator itJoint = jv.begin ();
         itJoint != jv.end (); itJoint++)
        {
            std::size_t rank = (*itJoint)->rankInConfiguration ();
            (*itJoint)->configuration ()->uniformlySample (rank, *config);
        }
        // Shoot extra configuration variables
        size_type extraDim = robot->extraConfigSpace ().dimension ();
        size_type offset = robot->configSize () - extraDim;
        for (size_type i=0; i<extraDim; ++i)
        {
            value_type lower = robot->extraConfigSpace ().lower (i);
            value_type upper = robot->extraConfigSpace ().upper (i);
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
        if(validator_->validate(*config,false)) return config;
        limit--;
    }
    return config;
}


  }// namespace rbprm
}// namespace hpp
