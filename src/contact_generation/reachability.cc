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

   // helper method used to print vectors of string
   std::ostream& operator<<(std::ostream& os, const std::vector<std::string>& vect){
       for(std::vector<std::string>::const_iterator sit = vect.begin() ; sit != vect.end() ; ++sit){
           os << *sit << " ; ";
       }
       return os;
   }



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
    fcl::Vec3f init = (c0+c1)/2.;

    hppDout(notice,"Call solveur solveIntersection");
    hppDout(notice,"init = "<<init);
    bezier_com_traj::ResultData res = bezier_com_traj::solveIntersection(Ab,computeDistanceCost(c0),init);
    c_out = res.x;
    hppDout(notice,"success Solveur solveIntersection = "<<res.success_);
    hppDout(notice,"x = ["<<c_out[0]<<","<<c_out[1]<<","<<c_out[2]<<"]");
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

std::pair<MatrixXX, VectorX> computeStabilityConstraintsForState(const RbPrmFullBodyPtr_t& fullbody, State &state){
    centroidal_dynamics::Equilibrium contactPhase(stability::initLibrary(fullbody));
    centroidal_dynamics::EquilibriumAlgorithm alg = centroidal_dynamics::EQUILIBRIUM_ALGORITHM_PP;
    stability::setupLibrary(fullbody,state,contactPhase,alg);
    return computeStabilityConstraints(contactPhase);
}

std::pair<MatrixXX, VectorX> computeConstraintsForState(const RbPrmFullBodyPtr_t& fullbody, State &state){
    return stackConstraints(computeKinematicsConstraintsForState(fullbody,state),computeStabilityConstraintsForState(fullbody,state));
}

Result isReachableIntermediate(const RbPrmFullBodyPtr_t& fullbody,State &previous,State &intermediate, State& next){
    //TODO
    hppDout(notice,"isReachableIntermadiate :");
    std::vector<std::string> contactsNames = next.contactVariations(previous);
    hppDout(notice,"Contact variations : "<<contactsNames);
    Result resBreak,resCreate;
    Result res;
    resBreak  = isReachable(fullbody,previous,intermediate);
    resCreate = isReachable(fullbody,intermediate,next);
    hppDout(notice,"isReachableIntermediate : ");
    hppDout(notice,"resBreak status    : "<<resBreak.status);
    hppDout(notice,"resCreation status : "<<resCreate.status);
    if(resBreak.success() && resCreate.success()){
        res.status=REACHABLE;
        res.x = (resBreak.x + resCreate.x)/2.;
    }else if(resBreak.status == UNREACHABLE && resCreate.status == UNREACHABLE){
        res.status=UNREACHABLE;
    }else if ( ! resBreak.success()){
        res.status=CONTACT_BREAK_FAILED;
    }else if ( ! resCreate.success()){
        res.status=CONTACT_CREATION_FAILED;
    }
    return res;
}

Result isReachable(const RbPrmFullBodyPtr_t& fullbody,State &previous, State& next){
    std::vector<std::string> contactsCreation, contactsBreak;
    next.contactBreaks(previous,contactsBreak);
    next.contactCreations(previous,contactsCreation);
    hppDout(notice,"IsReachable called : ");
    hppDout(notice,"Contacts break : "<<contactsBreak);
    hppDout(notice,"contacts creation : "<<contactsCreation);
    if(contactsCreation.size() <= 0 && contactsBreak.size() <= 0){
        hppDout(notice,"No contact variation, abort.");
        return Result(NO_CONTACT_VARIATION);
    }

    if(contactsCreation.size() >1 || contactsBreak.size() > 1){
        hppDout(notice,"Too many contact variation, abort.");
        return Result(TOO_MANY_CONTACTS_VARIATION);
    }

    if(contactsBreak.size() == 1 && contactsCreation.size() == 1){
        if(next.contactVariations(previous).size() == 1){ // there is 1 contact repositionning between previous and next
            // we need to create the intermediate state, and call is reachable for the 3 states.
            State intermediate(previous);
            intermediate.RemoveContact(contactsBreak[0]);
            hppDout(notice,"Contact repositionning between the 2 states, create intermediate state and call isReachable");
            return isReachableIntermediate(fullbody,previous,intermediate,next);
        }else{
            hppDout(notice,"Contact break and creation are different. You need to call isReachable with 2 adjacent states");
            return Result(TOO_MANY_CONTACTS_VARIATION);
        }
    }

    bool success;
    Result res;
    std::pair<MatrixXX,VectorX> Ab;
    // there is only one contact creation OR (exclusive) break between the two states
    // test C_p \inter C_n (ie : A_p \inter K_p \inter A_n \inter K_n), with simplifications du to relations between the constraints :
    if(contactsBreak.size() > 0){ // next have one less contact than previous
        hppDout(notice,"Contact break between previous and next state");
        // A_n \inside A_p, thus  A_p is redunbdant
        // K_p \inside K_n, thus  K_n is redunbdant
        // So, we only need to test A_n \inter K_p
        //TODO : K_p can be given as parameter to avoid re-computation
        std::pair<MatrixXX,VectorX> A_n = computeStabilityConstraintsForState(fullbody,next);
        std::pair<MatrixXX,VectorX> K_p = computeKinematicsConstraintsForState(fullbody,previous);
        Ab = stackConstraints(A_n,K_p);
    }else{// next have one more contact than previous
        hppDout(notice,"Contact creation between previous and next");
        // A_p \inside A_n, thus A_n is redunbdant
        // K_n \inside K_p ; and K_n = K_p \inter K_n^m (where K_n^m is the kinematic constraint for the moving limb at state n)
        // we use K_n^m, because C_p can be given as parameter to avoid re-computation
        // So, we only need to test C_n \inter K_p_m
        std::pair<MatrixXX,VectorX> C_p   = computeConstraintsForState(fullbody,previous);
        std::pair<MatrixXX,VectorX> K_n_m = computeKinematicsConstraintsForLimb(fullbody,previous,contactsCreation[0]); // kinematic constraint only for the moving contact for state previous
        Ab = stackConstraints(C_p,K_n_m);
    }


    fcl::Vec3f x;
    success = intersectionExist(Ab,previous.com_,next.com_,x);
    if(success)
        res.status=REACHABLE;
    else
        res.status=UNREACHABLE;
    res.x = x;
    return res;
}

}
}
}
