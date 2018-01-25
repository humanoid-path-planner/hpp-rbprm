# include <hpp/rbprm/contact_generation/kinematics_constraints.hh>
# include  <hpp/rbprm/rbprm-device.hh>
# include <hpp/model/urdf/parser.hh>
# include <urdf/model.h>


namespace hpp {
  namespace rbprm {

VectorX triangleNormal(const model::urdf::Parser::PolyhedronPtrType& obj, size_t index)
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

std::pair<MatrixXX, VectorX> loadConstraintsFromObj(const std::string& fileName){
    hppDout(notice,"Load constraints for filename : "<<fileName);

    /*std::ifstream f(fileName.c_str());
    if(!f.good()){
        hppDout(warning,"Unable to load kinematic constraints from obj, file doesn't exist : "<<fileName);
        return std::pair<MatrixXX, VectorX>();
    }*/
    model::urdf::Parser parser("anchor",model::DevicePtr_t());
    model::urdf::Parser::PolyhedronPtrType  polyhedron (new model::urdf::Parser::PolyhedronType);
    // name is stored in link->name
    parser.loadPolyhedronFromResource (fileName, ::urdf::Vector3(1,1,1), polyhedron);

    // iterate over all faces : for each faces add a line in A : normal and a value in b : position of a vertice.dot(normal)
    size_t numFaces = polyhedron->num_tris;
    MatrixXX A(numFaces,3);
    VectorX b(numFaces);
    VectorX n,v;
    for (size_t fId = 0 ; fId < numFaces ; ++fId){
        hppDout(notice,"For face : "<<fId);
        n = triangleNormal(polyhedron,fId);
        v = polyhedron->vertices[polyhedron->tri_indices[fId][0]];
        hppDout(notice,"normal : "<<n.transpose());
        hppDout(notice,"vertice: "<<v.transpose());
        A.block<1,3>(fId,0) = n;
        b[fId] = v.dot(n);
    }


    hppDout(notice,"End of loading kinematic constraints : ");
    hppDout(notice,"A : "<<A);
    hppDout(notice,"b : "<<b);
    return std::make_pair(A,b);
}




std::pair<MatrixXX, VectorX> applyTransformToConstraints(const std::pair<MatrixXX, VectorX>& Ab, fcl::Transform3f transform){

    return Ab;
}

bool verifyKinematicConstraints(const std::pair<MatrixXX, VectorX>& Ab, fcl::Vec3f point){
    hppDout(notice,"verify kinematic constraints : point = "<<point);
    for(size_t i = 0 ; i < Ab.second.size() ; ++i){
        hppDout(notice,"for i = "<<i);
        hppDout(notice,"A = "<<(Ab.first.block<1,3>(i,0)));
        hppDout(notice,"b = "<<Ab.second[i]);
        if(Ab.first.block<1,3>(i,0).dot(point) > Ab.second[i] ){
            return false;
        }
    }
    return true;
}

bool verifyKinematicConstraints(const State& state, fcl::Vec3f point){
    return true;
}


}
}

