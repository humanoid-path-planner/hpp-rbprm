#include <hpp/rbprm/contact_generation/reachability.hh>
#include <bezier-com-traj/solve.hh>
#include <bezier-com-traj/common_solve_methods.hh>
#include <hpp/rbprm/rbprm-fullbody.hh>
#include <hpp/rbprm/stability/stability.hh>

namespace hpp {
  namespace rbprm {
   namespace reachability{

   using centroidal_dynamics::Vector3;
   using centroidal_dynamics::Matrix3;


std::pair<MatrixXX, VectorX> stackConstraints(const std::pair<MatrixXX, VectorX>& Ab,const std::pair<MatrixXX, VectorX>& Cd){
    size_t numIneq = Ab.first.rows() + Cd.first.rows();
    MatrixXX M(numIneq,3);
    VectorX  n(numIneq);
    M.block(0,0,Ab.first.rows(),3) = Ab.first;
    M.block(Ab.first.rows(),0,Cd.first.rows(),3) = Cd.first;
    n.segment(0,Ab.first.rows()) = Ab.second;
    n.segment(Ab.first.rows(),Cd.first.rows()) = Cd.second;
    return std::make_pair(M,n);
}

/**
 * @brief computeDistanceCost cost that minimize || x - c ||
 * @param c0 target point (the point that we want to be the closest from)
 * @return the matrices H and g that express the cost
 */
std::pair<MatrixXX, VectorX> computeDistanceCost(const fcl::Vec3f& c0){
    MatrixXX H = Matrix3::Identity();
    VectorX g = -c0;
    return std::make_pair(H,g);
}

bool intersectionExist(const std::pair<MatrixXX, VectorX> &Ab, const fcl::Vec3f& c0, const fcl::Vec3f& c1, fcl::Vec3f c_out){
    fcl::Vec3f init = c0+c1/2.;

    hppDout(notice,"Call solveur solveIntersection");
    hppDout(notice,"init = "<<init);
    bezier_com_traj::ResultData res = bezier_com_traj::solveIntersection(Ab,computeDistanceCost(c0),init);
    c_out = res.x;
    hppDout(notice,"success Solveur solveIntersection = "<<res.success_);
    hppDout(notice,"com = "<<c_out);
    return res.success_;
}


std::pair<MatrixXX, VectorX> computeStabilityConstraints(const centroidal_dynamics::Equilibrium& contactPhase, const fcl::Vec3f &int_point){
    MatrixXX A;
    VectorX b;
    // gravity vector
    hppDout(notice,"Compute stability constraints");
    const Vector3& g = contactPhase.m_gravity;
    const Matrix3 gSkew = bezier_com_traj::skew(g);
    // compute GIWC
    centroidal_dynamics::MatrixXX Hrow; VectorX h;
    contactPhase.getPolytopeInequalities(Hrow,h);
    MatrixXX H = -Hrow;
    H.rowwise().normalize();
    int dimH = (int)(H.rows());
    hppDout(notice,"Dim H rows : "<<dimH<<" ; col : "<<H.cols());

    MatrixXX mH = contactPhase.m_mass * H;
    // constraints : mH[:,3:6] g^  x <= h + mH[:,0:3]g
    // A = mH g^
    // b = h + mHg
    A = mH.block(0,3,dimH,3) * gSkew;
    b = h+mH.block(0,0,dimH,3)*g;
    hppDout(notice,"Stability constraints matrices : ");
    hppDout(notice,"Interior point : \n"<<int_point);
    hppDout(notice,"A = \n"<<A);
    hppDout(notice,"b = \n"<<b);
    return std::make_pair(A,b);
}

std::pair<MatrixXX, VectorX> computeStabilityConstraints(const RbPrmFullBodyPtr_t& fullbody,rbprm::State& state){
    centroidal_dynamics::Equilibrium contactPhase(stability::initLibrary(fullbody));
    centroidal_dynamics::EquilibriumAlgorithm alg = centroidal_dynamics::EQUILIBRIUM_ALGORITHM_PP;
    stability::setupLibrary(fullbody,state,contactPhase,alg);
    return computeStabilityConstraints(contactPhase);
}


}
}
}
