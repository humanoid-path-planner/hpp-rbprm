/***
 * Helper file to define conversion between eigen and/or fcl geometries type
 *
 * */

#ifndef GEOM_CONVERSIONS_H
#define GEOM_CONVERSIONS_H


#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/src/Core/util/Macros.h>
#include <vector>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/model/collision-object.hh>

namespace geom
{
  typedef fcl::BVHModel<fcl::OBBRSS> BVHModelOB;
  typedef boost::shared_ptr<const BVHModelOB> BVHModelOBConst_Ptr_t;

  BVHModelOBConst_Ptr_t GetModel(const fcl::CollisionObjectConstPtr_t object);





  BVHModelOBConst_Ptr_t GetModel(const fcl::CollisionObjectConstPtr_t object)
  {
    assert(object->collisionGeometry()->getNodeType() == fcl::BV_OBBRSS);
    const BVHModelOBConst_Ptr_t model = boost::static_pointer_cast<const BVHModelOB>(object->collisionGeometry());
    assert(model->getModelType() == fcl::BVH_MODEL_TRIANGLES);
    return model;
  }

}//namespace geom
#endif // CONVERSIONS_H
