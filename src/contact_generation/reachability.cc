#include <hpp/rbprm/contact_generation/reachability.hh>
#include <bezier-com-traj/solve.hh>
#include <bezier-com-traj/common_solve_methods.hh>
#include <hpp/rbprm/rbprm-fullbody.hh>
#include <hpp/rbprm/stability/stability.hh>
#include <iostream>
#include <fstream>
#include <hpp/util/timer.hh>

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

   void printQHullFile(const std::pair<MatrixXX, VectorX>& Ab,fcl::Vec3f intPoint,const std::string& fileName,bool clipZ = false){
        std::ofstream file;
        using std::endl;
        std::string path("/home/pfernbac/Documents/com_ineq_test/");
        path.append(fileName);
        hppDout(notice,"print to file : "<<path);
        file.open(path.c_str(),std::ios::out | std::ios::trunc);
        file<<"3 1"<<endl;
        file<<"\t "<<intPoint[0]<<"\t"<<intPoint[1]<<"\t"<<intPoint[2]<<endl;
        file<<"4"<<endl;
        clipZ ? file<<Ab.first.rows()+2<<endl : file<<Ab.first.rows()<<endl;
        for(size_t i = 0 ; i < Ab.first.rows() ; ++i){
            file<<"\t"<<Ab.first(i,0)<<"\t"<<Ab.first(i,1)<<"\t"<<Ab.first(i,2)<<"\t"<<-Ab.second[i]-0.001<<endl;
        }
        if(clipZ){
            file<<"\t"<<0<<"\t"<<0<<"\t"<<1.<<"\t"<<-3.<<endl;
            file<<"\t"<<0<<"\t"<<0<<"\t"<<-1.<<"\t"<<-1.<<endl;
        }
        file.close();
   }


   // Method to print (in hppDout) the inequalities express as halfspace, in a format readable by qHull
   // use qHalf FP | qConvex Ft      to use them
   void printQHull(const std::pair<MatrixXX, VectorX>& Ab,fcl::Vec3f intPoint = fcl::Vec3f::Zero(),const std::string& fileName=std::string(),bool clipZ = false){
        using std::endl;
        std::stringstream ss;
        //ss<<"qHull Output : use qhalf FP | qconvex Ft "<<endl;
        ss<<"3 1"<<endl;
        ss<<"\t "<<intPoint[0]<<"\t"<<intPoint[1]<<"\t"<<intPoint[2]<<endl;
        ss<<"4"<<endl;
        for(size_t i = 0 ; i < Ab.first.rows() ; ++i){
            ss<<"\t"<<Ab.first(i,0)<<"\t"<<Ab.first(i,1)<<"\t"<<Ab.first(i,2)<<"\t"<<-Ab.second[i]-0.001<<endl;
        }
        hppDout(notice,ss.str());
        if(!fileName.empty())
            printQHullFile(Ab,intPoint,fileName,clipZ);
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

bool intersectionExist(const std::pair<MatrixXX, VectorX> &Ab, const fcl::Vec3f& c0, const fcl::Vec3f& c1, fcl::Vec3f& c_out){
    fcl::Vec3f init = (c0+c1)/2.;

    hppDout(notice,"Call solveur solveIntersection");
    hppDout(notice,"init = "<<init);
    bezier_com_traj::ResultData res = bezier_com_traj::solveIntersection(Ab,computeDistanceCost(c0),init);
    c_out = res.x;
    hppDout(notice,"success Solveur solveIntersection = "<<res.success_);
    hppDout(notice,"x = ["<<c_out[0]<<","<<c_out[1]<<","<<c_out[2]<<"]");
    return res.success_;
}


std::pair<MatrixXX, VectorX> computeStabilityConstraints(const centroidal_dynamics::Equilibrium& contactPhase, const fcl::Vec3f &int_point, const fcl::Vec3f &acc){
    MatrixXX A;
    VectorX b;
    // gravity vector
    hppDout(notice,"Compute stability constraints");
    hppDout(notice,"With acceleration = "<<acc);
    const Vector3& g = contactPhase.m_gravity;
    const Matrix3 gSkew = bezier_com_traj::skew(g);
    const Matrix3 accSkew = bezier_com_traj::skew(acc);
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
    A = mH.block(0,3,dimH,3) * (gSkew - accSkew);
    b = h+mH.block(0,0,dimH,3)*(g - acc);
   /* hppDout(notice,"Stability constraints matrices : ");
    hppDout(notice,"Interior point : \n"<<int_point);
    hppDout(notice,"A = \n"<<A);
    hppDout(notice,"b = \n"<<b);*/
    hppDout(notice,"Stability constraints qHull : ");
    printQHull(std::make_pair(A,b),int_point);
    return std::make_pair(A,b);
}

std::pair<MatrixXX, VectorX> computeStabilityConstraintsForState(const RbPrmFullBodyPtr_t& fullbody, State &state,const fcl::Vec3f& acc){
    hppStartBenchmark(REACHABLE_CALL_CENTROIDAL);
    centroidal_dynamics::Equilibrium contactPhase(stability::initLibrary(fullbody));
    centroidal_dynamics::EquilibriumAlgorithm alg = centroidal_dynamics::EQUILIBRIUM_ALGORITHM_PP;
    stability::setupLibrary(fullbody,state,contactPhase,alg);
    hppStopBenchmark(REACHABLE_CALL_CENTROIDAL);
    hppDisplayBenchmark(REACHABLE_CALL_CENTROIDAL);
    return computeStabilityConstraints(contactPhase,state.contactPositions_.at(state.contactOrder_.front()),
                                       acc.isZero() ? state.configuration_.tail<3>() : acc);
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
    std::pair<MatrixXX,VectorX> Ab,Cd;
    resBreak  = isReachable(fullbody,previous,intermediate);
    resCreate = isReachable(fullbody,intermediate,next);
    hppDout(notice,"isReachableIntermediate : ");
    hppDout(notice,"resBreak status    : "<<resBreak.status);
    hppDout(notice,"resCreation status : "<<resCreate.status);
    hppDout(notice,"constraint for contact break : ");
    printQHull(resBreak.constraints_,resBreak.x,"constraints_break.txt");
    hppDout(notice,"constraint for contact creation : ");
    printQHull(resCreate.constraints_,resCreate.x,"constraints_create.txt");

    if(resBreak.success() && resCreate.success()){
        res.status=REACHABLE;
        res.x = (resBreak.x + resCreate.x)/2.;
        // only for test, it take time to compute :
        hppDout(notice,"constraint for intersection : ");
        printQHull(stackConstraints(resBreak.constraints_,resCreate.constraints_),res.x,"constraints.txt");
    }else if(resBreak.status == UNREACHABLE && resCreate.status == UNREACHABLE){
        res.status=UNREACHABLE;
    }else if ( ! resBreak.success()){
        res.status=CONTACT_BREAK_FAILED;
    }else if ( ! resCreate.success()){
        res.status=CONTACT_CREATION_FAILED;
    }
    return res;
}

Result isReachable(const RbPrmFullBodyPtr_t& fullbody, State &previous, State& next,const fcl::Vec3f& acc){
    hppStartBenchmark(IS_REACHABLE);
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
    std::pair<MatrixXX,VectorX> Ab,K_p,K_n,A_p,A_n;
    // there is only one contact creation OR (exclusive) break between the two states
    // test C_p \inter C_n (ie : A_p \inter K_p \inter A_n \inter K_n), with simplifications du to relations between the constraints :
    if(contactsBreak.size() > 0){ // next have one less contact than previous
        hppDout(notice,"Contact break between previous and next state");
        // A_n \inside A_p, thus  A_p is redunbdant
        // K_p \inside K_n, thus  K_n is redunbdant
        // So, we only need to test A_n \inter K_p
        //TODO : K_p can be given as parameter to avoid re-computation
        //std::pair<MatrixXX,VectorX> A_n = computeStabilityConstraintsForState(fullbody,next);
        //std::pair<MatrixXX,VectorX> K_p = computeKinematicsConstraintsForState(fullbody,previous);
        //Ab = stackConstraints(A_n,K_p);
        // develloped computation, needed to display the differents constraints :
        hppStartBenchmark(REACHABLE_STABILITY);
        A_n = computeStabilityConstraintsForState(fullbody,next,acc);
        hppStopBenchmark(REACHABLE_STABILITY);
        hppDisplayBenchmark(REACHABLE_STABILITY);
        hppStartBenchmark(REACHABLE_KINEMATIC);
        K_p = computeKinematicsConstraintsForState(fullbody,previous);
        hppStopBenchmark(REACHABLE_KINEMATIC);
        hppDisplayBenchmark(REACHABLE_KINEMATIC);
        hppStartBenchmark(REACHABLE_STACK);
        Ab = stackConstraints(A_n,K_p);
        hppStopBenchmark(REACHABLE_STACK);
        hppDisplayBenchmark(REACHABLE_STACK);
    }else{// next have one more contact than previous
        hppDout(notice,"Contact creation between previous and next");
        // A_p \inside A_n, thus A_n is redunbdant
        // K_n \inside K_p ; and K_n = K_p \inter K_n^m (where K_n^m is the kinematic constraint for the moving limb at state n)
        // we use K_n^m, because C_p can be given as parameter to avoid re-computation
        // So, we only need to test C_n \inter K_p_m
        //std::pair<MatrixXX,VectorX> C_p   = computeConstraintsForState(fullbody,previous);
        //std::pair<MatrixXX,VectorX> K_n_m = computeKinematicsConstraintsForLimb(fullbody,previous,contactsCreation[0]); // kinematic constraint only for the moving contact for state previous
        //Ab = stackConstraints(C_p,K_n_m);
        hppStartBenchmark(REACHABLE_STABILITY);
        A_p = computeStabilityConstraintsForState(fullbody,previous,acc);
        hppStopBenchmark(REACHABLE_STABILITY);
        hppDisplayBenchmark(REACHABLE_STABILITY);
        //K_p = computeKinematicsConstraintsForState(fullbody,previous);
        hppStartBenchmark(REACHABLE_KINEMATIC);
        K_n = computeKinematicsConstraintsForState(fullbody,next);
        hppStopBenchmark(REACHABLE_KINEMATIC);
        hppDisplayBenchmark(REACHABLE_KINEMATIC);
        hppStartBenchmark(REACHABLE_STACK);
        Ab = stackConstraints(A_p,K_n);
        hppStopBenchmark(REACHABLE_STACK);
        hppDisplayBenchmark(REACHABLE_STACK);
    }


    fcl::Vec3f x;
    hppStartBenchmark(QP_REACHABLE);
    success = intersectionExist(Ab,previous.com_,next.com_,x);
    hppStopBenchmark(QP_REACHABLE);
    hppDisplayBenchmark(QP_REACHABLE);
    if(success)
        res.status=REACHABLE;
    else
        res.status=UNREACHABLE;
    res.x = x;
    res.constraints_=Ab;
    hppStopBenchmark(IS_REACHABLE);
    hppDisplayBenchmark(IS_REACHABLE);

    if(contactsBreak.size() > 0){
        hppDout(notice,"Stability constraint for state i+1 :");
        printQHull(A_n,x,"stability.txt",true);
        hppDout(notice,"Kinematics constraint for state i :");
        printQHull(K_p,x,"kinematics.txt");
    }else{
        hppDout(notice,"Stability constraint for state i :");
        printQHull(A_p,x,"stability.txt",true);
        hppDout(notice,"Kinematics constraint for state i+1 :");
        printQHull(K_n,x,"kinematics.txt");
    }
    hppDout(notice,"Intersection of constraints :");
    printQHull(Ab,x,"constraints.txt");

    return res;
}

}
}
}
