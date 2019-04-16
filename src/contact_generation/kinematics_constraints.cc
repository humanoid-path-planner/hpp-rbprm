# include <hpp/rbprm/contact_generation/kinematics_constraints.hh>
# include <hpp/rbprm/rbprm-device.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>
# include <hpp/fcl/BVH/BVH_model.h>
# include <hpp/fcl/mesh_loader/assimp.h>
# include <hpp/pinocchio/urdf/util.hh>
# include <pinocchio/utils/file-explorer.hpp>
# include <pinocchio/parsers/utils.hpp>
# include <pinocchio/multibody/geometry.hpp>

namespace hpp {
  namespace rbprm {
    namespace reachability{

typedef fcl::BVHModel< fcl::OBBRSS > PolyhedronType;
typedef boost::shared_ptr <PolyhedronType> PolyhedronPtrType;

VectorX triangleNormal(const PolyhedronPtrType& obj, size_t index)
{
    VectorX normal(3);
    // access a vertice of faceId with : obj->vertices[obj->tri_indices[index](i)]
    // outward normal of triangle : (tri.p2 - tri.p1).cross(tri.p3 - tri.p1);
    /*hppDout(notice,"Face : "<<index);
    hppDout(notice,"0 - "<<obj->vertices[obj->tri_indices[index][0]]);
    hppDout(notice,"1 - "<<obj->vertices[obj->tri_indices[index][1]]);
    hppDout(notice,"2 - "<<obj->vertices[obj->tri_indices[index][2]]);
    */
    normal = (obj->vertices[obj->tri_indices[index][1]] - obj->vertices[obj->tri_indices[index][0]]).cross(obj->vertices[obj->tri_indices[index][2]] - obj->vertices[obj->tri_indices[index][0]]);
    return normal.normalized();
}

VectorX triangleNormalTransform(const PolyhedronPtrType& obj, size_t index,fcl::Transform3f T)
{
    VectorX normal(3);
    // access a vertice of faceId with : obj->vertices[obj->tri_indices[index](i)]
    // outward normal of triangle : (tri.p2 - tri.p1).cross(tri.p3 - tri.p1);
    /*hppDout(notice,"Face : "<<index);
    hppDout(notice,"0 - "<<obj->vertices[obj->tri_indices[index][0]]);
    hppDout(notice,"1 - "<<obj->vertices[obj->tri_indices[index][1]]);
    hppDout(notice,"2 - "<<obj->vertices[obj->tri_indices[index][2]]);
    */
    fcl::Vec3f t0,t1,t2;
    t0 = T.transform(obj->vertices[obj->tri_indices[index][0]]) ;
    t1 = T.transform(obj->vertices[obj->tri_indices[index][1]]) ;
    t2 = T.transform(obj->vertices[obj->tri_indices[index][2]]) ;
    normal = (t1 - t0).cross(t2 - t0);
    return normal.normalized();
}


} // namespace reachability
} // namespace rbprm
} // namespace hpp

std::pair<hpp::rbprm::MatrixX3, hpp::rbprm::MatrixX3> hpp::rbprm::reachability::loadConstraintsFromObj(const std::string& fileName, double minDistance){
    hppDout(notice,"Load constraints for filename : "<<fileName);

    std::vector<std::string> package_dirs = ::pinocchio::rosPaths();
    std::string meshPath = ::pinocchio::retrieveResourcePath(fileName, package_dirs);
    if (meshPath == "")
    {
        hppDout(warning,"Unable to load kinematics constraints : "<<  "Mesh " << fileName << " could not be found.");
        return std::pair<hpp::rbprm::MatrixX3, hpp::rbprm::MatrixX3>();
    }

    //pinocchio::urdf::Parser parser("anchor",pinocchio::DevicePtr_t());
    hpp::rbprm::reachability::PolyhedronPtrType  polyhedron (new hpp::rbprm::reachability::PolyhedronType);
    // name is stored in link->name
    try
    {
        fcl::loadPolyhedronFromResource(meshPath, hpp::rbprm::Vector3(1,1,1), polyhedron);
    }
    catch (std::runtime_error e)
    {
        hppDout(warning,"Unable to load kinematics constraints : "<< e.what());
        return std::pair<hpp::rbprm::MatrixX3, hpp::rbprm::MatrixX3>();
    }

    // iterate over all faces : for each faces add a line in A : normal and a value in b : position of a vertice.dot(normal)
    size_t numFaces = polyhedron->num_tris;
    hppDout(notice,"Num faces : "<<numFaces);
    size_t numIneq = numFaces;
    if(minDistance > 0){
        numIneq++;
    }
    hpp::rbprm::MatrixX3 N(numIneq,3);
    hpp::rbprm::MatrixX3 V(numIneq,3);



    hpp::rbprm::VectorX n,v;
    for (size_t fId = 0 ; fId < numFaces ; ++fId){
        //hppDout(notice,"For face : "<<fId);
        n = hpp::rbprm::reachability::triangleNormal(polyhedron,fId);
        v = polyhedron->vertices[polyhedron->tri_indices[fId][0]];
        //hppDout(notice,"normal : "<<n.transpose());
        //hppDout(notice,"vertice: "<<v.transpose());
        N.block<1,3>(fId,0) = n;
        V.block<1,3>(fId,0) = v;
    }

    if(minDistance > 0){
        N.block<1,3>(numIneq-1,0) = hpp::rbprm::Vector3(0,0,-1);
        V.block<1,3>(numIneq-1,0) = hpp::rbprm::Vector3(0,0,minDistance);
    }
    hppDout(notice,"End of loading kinematic constraints : ");
    //hppDout(notice,"N : "<<N);
    //hppDout(notice,"v : "<<V);

    return std::make_pair(N,V);
}

namespace hpp {
  namespace rbprm {
    namespace reachability {
std::pair<MatrixX3, VectorX> computeAllKinematicsConstraints(const RbPrmFullBodyPtr_t& fullBody,const pinocchio::ConfigurationPtr_t& configuration){
    fullBody->device_->currentConfiguration(*configuration);
    fullBody->device_->computeForwardKinematics();
    // first loop to compute size required :
    size_t numIneq = 0;
    for(CIT_Limb lit = fullBody->GetLimbs().begin() ; lit != fullBody->GetLimbs().end() ; ++lit){
        numIneq += lit->second->kinematicConstraints_.first.rows();
    }
    MatrixX3 A(numIneq,3);
    VectorX b(numIneq);
    std::pair<MatrixX3,VectorX> Ab_limb;
    size_t currentId = 0;
    for(CIT_Limb lit = fullBody->GetLimbs().begin() ; lit != fullBody->GetLimbs().end() ; ++lit){
        if(lit->second->kinematicConstraints_.first.size()>0){
            Ab_limb = getInequalitiesAtTransform(lit->second->kinematicConstraints_,lit->second->effector_.currentTransformation());
            A.block(currentId,0,Ab_limb.first.rows(),3) = Ab_limb.first;
            b.segment(currentId,Ab_limb.first.rows()) = Ab_limb.second;
            currentId += Ab_limb.first.rows();
        }
    }

    return std::make_pair(A,b);
}

std::pair<MatrixX3, VectorX> computeKinematicsConstraintsForState(const RbPrmFullBodyPtr_t& fullBody, const State& state){
    fullBody->device_->currentConfiguration(state.configuration_);
    fullBody->device_->computeForwardKinematics();
    hppDout(notice,"Compute kinematics constraints :");
    // first loop to compute size required :
    size_t numIneq = 0;
    for(std::map<std::string,bool>::const_iterator cit = state.contacts_.begin();cit!=state.contacts_.end(); ++ cit){
        if(cit->second){ // limb with name cit->first is in contact
            numIneq += fullBody->GetLimb(cit->first)->kinematicConstraints_.first.rows();
            hppDout(notice,"Limb : "<<cit->first<<" is in contact.");
        }
    }
    MatrixX3 A(numIneq,3);
    VectorX b(numIneq);
    std::pair<MatrixX3,VectorX> Ab_limb;
    size_t currentId = 0;
    RbPrmLimbPtr_t limb;
    for(std::map<std::string,bool>::const_iterator cit = state.contacts_.begin();cit!=state.contacts_.end(); ++ cit){
        if(cit->second){ // limb with name cit->first is in contact
            limb = fullBody->GetLimb(cit->first);
            Ab_limb = getInequalitiesAtTransform(limb->kinematicConstraints_,limb->effector_.currentTransformation());
            A.block(currentId,0,Ab_limb.first.rows(),3) = Ab_limb.first;
            b.segment(currentId,Ab_limb.first.rows()) = Ab_limb.second;
            currentId += Ab_limb.first.rows();
        }
    }
    hppDout(notice,"End of kinematics constraints, A size : ("<<A.rows()<<","<<A.cols()<<")");
    return std::make_pair(A,b);
}

std::pair<MatrixX3, VectorX> computeKinematicsConstraintsForLimb(const RbPrmFullBodyPtr_t& fullBody, const State& state,const std::string& limbName){
    fullBody->device_->currentConfiguration(state.configuration_);
    fullBody->device_->computeForwardKinematics();
    hppDout(notice,"Compute kinematics constraints for limb :"<<limbName);
    // first loop to compute size required :
    if(state.contacts_.find(limbName) == state.contacts_.end()){
        hppDout(warning,"No Limbs in contact found with name :"<<limbName);
        return std::pair<MatrixX3, VectorX>();
    }
    if(!state.contacts_.at(limbName)){
        hppDout(warning,"Limb "<<limbName<<" is not in contact for current state");
        return std::pair<MatrixX3, VectorX>();
    }

    RbPrmLimbPtr_t limb = fullBody->GetLimb(limbName);
    return getInequalitiesAtTransform(limb->kinematicConstraints_,limb->effector_.currentTransformation());
}


std::pair<MatrixX3, VectorX> getInequalitiesAtTransform(const std::pair<MatrixX3, MatrixX3> &NV, const hpp::pinocchio::Transform3f& transformPin){
    fcl::Transform3f transform(transformPin.rotation(),transformPin.translation());
    size_t numIneq = NV.first.rows();
    MatrixX3 A(numIneq,3);
    VectorX b(numIneq);
    VectorX n,v;
    for(size_t i = 0 ; i < numIneq ; ++i){
        n = transform.getRotation()*(NV.first.block<1,3>(i,0).transpose());
        v = transform.transform(NV.second.block<1,3>(i,0));
        A.block<1,3>(i,0) = n;
        b[i] = v.dot(n);
    }
   // hppDout(notice,"Transform : "<<transform);
  //  hppDout(notice,"A : "<<A);
  //  hppDout(notice,"b : "<<b);
    return std::make_pair(A,b);
}

bool verifyKinematicConstraints(const std::pair<MatrixX3, MatrixX3>& NV, const hpp::pinocchio::Transform3f &transform, const fcl::Vec3f &point){
    return verifyKinematicConstraints(getInequalitiesAtTransform(NV,transform),point);
}

bool verifyKinematicConstraints(const std::pair<MatrixX3, VectorX>& Ab, const fcl::Vec3f &point){
    hppDout(notice,"verify kinematic constraints : point = "<<point);
    for(size_type i = 0 ; i < Ab.second.size() ; ++i){
        hppDout(notice,"for i = "<<i);
        hppDout(notice,"A = "<<(Ab.first.block<1,3>(i,0)));
        hppDout(notice,"b = "<<Ab.second[i]);
        if(Ab.first.block<1,3>(i,0).dot(point) > Ab.second[i] ){
            hppDout(notice,"Constraint not verified !");
            return false;
        }
    }
    return true;
}

bool verifyKinematicConstraints(const RbPrmFullBodyPtr_t& fullbody,const State& state, fcl::Vec3f point){
    return verifyKinematicConstraints(computeKinematicsConstraintsForState(fullbody,state),point);
}


}
}
}
