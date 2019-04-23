#include <hpp/rbprm/contact_generation/reachability.hh>
#include <hpp/bezier-com-traj/solve.hh>
#include <hpp/bezier-com-traj/common_solve_methods.hh>
#include <hpp/rbprm/rbprm-fullbody.hh>
#include <hpp/rbprm/stability/stability.hh>
#include <iostream>
#include <fstream>
#include <hpp/util/timer.hh>

#ifndef QHULL
#define QHULL 0
#endif

#ifndef STAT_TIMINGS
#define STAT_TIMINGS 0
#endif

#ifndef FULL_TIME_SAMPLING
#define FULL_TIME_SAMPLING 0
#endif

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
        std::string path("/local/fernbach/qhull/constraints_obj/");
        path.append(fileName);
        hppDout(notice,"print to file : "<<path);
        file.open(path.c_str(),std::ios::out | std::ios::trunc);
        file<<"3 1"<<endl;
        file<<"\t "<<intPoint[0]<<"\t"<<intPoint[1]<<"\t"<<intPoint[2]<<endl;
        file<<"4"<<endl;
        clipZ ? file<<Ab.first.rows()+2<<endl : file<<Ab.first.rows()<<endl;
        for(size_type i = 0 ; i < Ab.first.rows() ; ++i){
            file<<"\t"<<Ab.first(i,0)<<"\t"<<Ab.first(i,1)<<"\t"<<Ab.first(i,2)<<"\t"<<-Ab.second[i]-0.005<<endl;
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
        ss<<"qHull Output : use qhalf FP | qconvex Ft "<<endl;
        ss<<"3 1"<<endl;
        ss<<"\t "<<intPoint[0]<<"\t"<<intPoint[1]<<"\t"<<intPoint[2]<<endl;
        ss<<"4"<<endl;
        for(size_type i = 0 ; i < Ab.first.rows() ; ++i){
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
    bezier_com_traj::ResultData res = bezier_com_traj::solve(Ab,computeDistanceCost(c0),init);
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
   // hppDout(notice,"Hrow : \n"<<Hrow);
    MatrixXX H = -Hrow;
    H.rowwise().normalize();
    int dimH = (int)(H.rows());
    hppDout(notice,"Dim H rows : "<<dimH<<" ; col : "<<H.cols());
   // hppDout(notice,"H : \n"<<H);
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
    #if QHULL
        hppDout(notice,"Stability constraints qHull : ");
        printQHull(std::make_pair(A,b),int_point);
    #else
        (void)int_point; // silent the unused parameter warning in case QHULL 0
    #endif
    return std::make_pair(A,b);
}

centroidal_dynamics::Equilibrium computeContactConeForState(const RbPrmFullBodyPtr_t& fullbody, State &state, bool& success){
    hppStartBenchmark(REACHABLE_CALL_CENTROIDAL);
    centroidal_dynamics::Equilibrium contactCone(fullbody->device_->name(), fullbody->device_->mass(),4,centroidal_dynamics::SOLVER_LP_QPOASES,true,100,false);
    centroidal_dynamics::EquilibriumAlgorithm alg = centroidal_dynamics::EQUILIBRIUM_ALGORITHM_PP;
    try{
        stability::setupLibrary(fullbody,state,contactCone,alg,fullbody->getFriction(),0.05,0.05); // 0.01 : 'safe' support zone, under the flexibility
        success = true;
    }catch(std::runtime_error e){
        hppDout(notice,"Error in setupLibrary : "<<e.what());
        success = false;
    }
    hppStopBenchmark(REACHABLE_CALL_CENTROIDAL);
    hppDisplayBenchmark(REACHABLE_CALL_CENTROIDAL);
    return contactCone;
}

std::pair<MatrixXX, VectorX> computeStabilityConstraintsForState(const RbPrmFullBodyPtr_t& fullbody, State &state,bool& success, const fcl::Vec3f& acc){
    hppDout(notice,"contact order : ");
    hppDout(notice,"  "<<state.contactOrder_.front());
    return computeStabilityConstraints(computeContactConeForState(fullbody,state,success),state.contactPositions_.at(state.contactOrder_.front()),
                                       acc.isZero() ? state.configuration_.tail<3>() : acc);
}

std::pair<MatrixXX, VectorX> computeConstraintsForState(const RbPrmFullBodyPtr_t& fullbody, State &state,bool& success){
    return stackConstraints(computeKinematicsConstraintsForState(fullbody,state),computeStabilityConstraintsForState(fullbody,state,success));
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
    #if QHULL
    hppDout(notice,"constraint for contact break : ");
    printQHull(resBreak.constraints_,resBreak.x,"constraints_break.txt");
    hppDout(notice,"constraint for contact creation : ");
    printQHull(resCreate.constraints_,resCreate.x,"constraints_create.txt");
    #endif
    if(resBreak.success() && resCreate.success()){
        res.status=REACHABLE;
        res.x = (resBreak.x + resCreate.x)/2.;
        res.xBreak_ = resBreak.x;
        res.xCreate_ = resCreate.x;
        hppDout(notice,"reachable intermediate success, x = "<<res.x);
        // only for test, it take time to compute :
        #if QHULL
        hppDout(notice,"constraint for intersection : ");
        printQHull(stackConstraints(resBreak.constraints_,resCreate.constraints_),res.x,"constraints.txt");
        #endif
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
    hppDout(notice,"IsReachable called : for configuration :  r(["<<pinocchio::displayConfig(previous.configuration_)<<"])");
    hppDout(notice,"contact for previous : "<<previous.nbContacts);
    hppDout(notice,"contact for next     : "<<next.nbContacts);
    hppDout(notice,"Contacts break : "<<contactsBreak);
    hppDout(notice,"contacts creation : "<<contactsCreation);

    #if QHULL == 0
    if(contactsCreation.size() <= 0 && contactsBreak.size() <= 0){
        hppDout(notice,"No contact variation, abort.");
        return Result(NO_CONTACT_VARIATION);
    }
    #endif

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
    bool successCone;
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
        A_n = computeStabilityConstraintsForState(fullbody,next,successCone,acc);
        if(!successCone)
            return Result(UNABLE_TO_COMPUTE);

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
        A_p = computeStabilityConstraintsForState(fullbody,previous,successCone,acc);
        if(!successCone)
            return Result(UNABLE_TO_COMPUTE);
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


    // compute COM positions :
    pinocchio::Computation_t flag = fullbody->device_->computationFlag();
    pinocchio::Computation_t newflag = static_cast <pinocchio::Computation_t> (pinocchio::JOINT_POSITION | pinocchio::JACOBIAN | pinocchio::COM);
    fullbody->device_->controlComputation (newflag);
    fullbody->device_->currentConfiguration(previous.configuration_);
    fullbody->device_->computeForwardKinematics();
    fcl::Vec3f com_previous = fullbody->device_->positionCenterOfMass();
    fullbody->device_->currentConfiguration(next.configuration_);
    fullbody->device_->computeForwardKinematics();
    fcl::Vec3f com_next = fullbody->device_->positionCenterOfMass();
    fullbody->device_->controlComputation (flag);

    fcl::Vec3f x;
    hppStartBenchmark(QP_REACHABLE);
    success = intersectionExist(Ab,com_previous,com_next,x);
    hppStopBenchmark(QP_REACHABLE);
    hppDisplayBenchmark(QP_REACHABLE);
    if(success){
        res.status=REACHABLE;
        hppDout(notice,"REACHABLE");
    }
    else{
        res.status=UNREACHABLE;
        hppDout(notice,"UNREACHABLE");
    }
    res.x = x;
    res.constraints_=Ab;
    hppStopBenchmark(IS_REACHABLE);
    hppDisplayBenchmark(IS_REACHABLE);


    // compute interior point to display with qHull (only required for display and debug)
    #if QHULL
    Vector3 int_pt_kin;
    fcl::Vec3f int_pt_stab;
    if(res.success()){
        int_pt_stab = x;
        int_pt_kin = x;
    }
    else{
        if(contactsBreak.size() > 0){
            int_pt_kin = com_previous;
            intersectionExist(A_n,com_previous,com_next,int_pt_stab);
        }else{
            int_pt_kin = com_next;
            intersectionExist(A_p,com_previous,com_next,int_pt_stab);
        }
    }

    if(contactsBreak.size() > 0){
        hppDout(notice,"Stability constraint for state i+1 :");
        printQHull(A_n,int_pt_stab,"stability.txt",true);
        hppDout(notice,"Kinematics constraint for state i :");
        printQHull(K_p,int_pt_kin,"kinematics.txt");
    }else{
        hppDout(notice,"Stability constraint for state i :");
        printQHull(A_p,int_pt_stab,"stability.txt",true);
        hppDout(notice,"Kinematics constraint for state i+1 :");
        printQHull(K_n,int_pt_kin,"kinematics.txt");
    }
    hppDout(notice,"Intersection of constraints :");
    printQHull(Ab,int_pt_stab,"constraints.txt");

    if(contactsCreation.size() <= 0 && contactsBreak.size() <= 0){
        hppDout(notice,"No contact variation, abort.");
        return Result(NO_CONTACT_VARIATION);
    }
    #endif

    return res;
}

void printTimingFile(std::ofstream& file,const VectorX& timings, bool success, bool quasiStaticSuccess){
    using std::endl;
    file<<timings[0]<<" "<<timings[1]<<" "<<timings[2]<<" "<<(success?"1 ":"0 ")<<(quasiStaticSuccess?"1":"0")<<endl;
}

Result isReachableDynamic(const RbPrmFullBodyPtr_t& fullbody, State &previous, State& next, bool tryQuasiStatic, std::vector<double> timings, int numPointsPerPhases){
    Result res;
    std::vector<std::string> contactsCreation, contactsBreak;
    next.contactBreaks(previous,contactsBreak);
    next.contactCreations(previous,contactsCreation);
    hppDout(notice,"IsReachableDynamic called : ");
    hppDout(notice,"Between configuration : "<<pinocchio::displayConfig(previous.configuration_));
    hppDout(notice,"and     configuration : "<<pinocchio::displayConfig(next.configuration_));
    assert(fullbody->device_->extraConfigSpace().dimension() >= 6 && "You need to set at least 6 extraDOF to use the reachability methods.");
    if(previous.configuration_.head<3>() == next.configuration_.head<3>()){
        hppDout(notice,"Same root position, unable to compute");
        return Result(SAME_ROOT_POSITION);
    }
    hppDout(notice,"Contacts break : "<<contactsBreak);
    hppDout(notice,"contacts creation : "<<contactsCreation);
    if(contactsCreation.size() <= 0 && contactsBreak.size() <= 0){
        hppDout(notice,"No contact variation, abort.");
        return Result(NO_CONTACT_VARIATION);
    }
    if ((contactsBreak.size() + contactsCreation.size()) > 2 || contactsBreak.size()>1 || contactsCreation.size()>1){
        hppDout(notice,"More than 2 contact change");
        return Result(TOO_MANY_CONTACTS_VARIATION);
    }
    if (contactsBreak.size() == 0){
        hppDout(notice,"Only contact creation. State Next have more contact than state Previous");
        if(previous.nbContacts <=1){
            hppDout(notice,"Previous is in single support. Unable to compute"); // FIXME : maybe compute in quasiStatic
            return Result(UNABLE_TO_COMPUTE);
        }
    }
    if (contactsCreation.size() == 0){
        hppDout(notice,"Only contact break. State Next have less contact than state Previous");
        if(next.nbContacts <=1){
            hppDout(notice,"Next is in single support. Unable to compute"); // FIXME : maybe compute in quasiStatic
            return Result(UNABLE_TO_COMPUTE);
        }
    }



    // build intermediate state :
    State mid(previous); // build intermediate state
    if(contactsBreak.size() > 0){
        mid.RemoveContact(contactsBreak[0]);
    }


    #if STAT_TIMINGS
    // outputs timings results in file :
    std::ofstream file;
    std::string path("/local/fernbac/bench_iros18/bench/timing_hrp2_darpa.log");
    hppDout(notice,"print to file : "<<path);
    file.open(path.c_str(),std::ios_base::app);
    file<<"# new pair of states"<<std::endl;
    #endif
    bool quasiStaticSucces(false);

    // compute COM positions :
    pinocchio::Computation_t flag = fullbody->device_->computationFlag();
    pinocchio::Computation_t newflag = static_cast <pinocchio::Computation_t> (pinocchio::JOINT_POSITION | pinocchio::JACOBIAN | pinocchio::COM);
    fullbody->device_->controlComputation (newflag);
    fullbody->device_->currentConfiguration(previous.configuration_);
    fullbody->device_->computeForwardKinematics();
    fcl::Vec3f com_previous = fullbody->device_->positionCenterOfMass();
    fullbody->device_->currentConfiguration(next.configuration_);
    fullbody->device_->computeForwardKinematics();
    fcl::Vec3f com_next = fullbody->device_->positionCenterOfMass();
    fullbody->device_->controlComputation (flag);



    if(tryQuasiStatic){
        hppDout(notice,"Try quasi-static reachability : ");
        Result quasiStaticResult = isReachableIntermediate(fullbody,previous,mid,next);
        if(quasiStaticResult.success()){
            hppDout(notice,"REACHABLE in quasi-static");
            quasiStaticResult.status=QUASI_STATIC;
            // build a Bezier curve of order 2 :
            std::vector<Vector3> wps;
            wps.push_back(com_previous);
            wps.push_back(quasiStaticResult.x);
            wps.push_back(com_next);
            bezier_Ptr bezierCurve=bezier_Ptr(new bezier_t(wps.begin(),wps.end(),1.));
            quasiStaticResult.path_ = BezierPath::create(fullbody->device_,bezierCurve,previous.configuration_,next.configuration_, core::interval_t(0.,1));
            quasiStaticSucces = true;
            #if !STAT_TIMINGS
            return quasiStaticResult;
            #endif
        }else{
            hppDout(notice,"UNREACHABLE in quasi-static");
            //return Result(REACHABLE); // testing
        }
    }

    hppStartBenchmark(IS_REACHABLE_DYNAMIC);
    hppStartBenchmark(COMPUTE_DOUBLE_DESCRIPTION);

    // build ProblemData from states object and call solveOneStep()
    bezier_com_traj::ProblemData pData;
    bool successCone(true),successConeCurrent;
    // build contactPhases :
    bezier_com_traj::ContactData previousData,nextData,midData;
    std::pair<MatrixX3,VectorX> Ab = computeKinematicsConstraintsForState(fullbody,previous);
    previousData.Kin_ = Ab.first;
    previousData.kin_ = Ab.second;
    centroidal_dynamics::Equilibrium conePrevious = computeContactConeForState(fullbody,previous,successConeCurrent);
    successCone = successCone && successConeCurrent;
    previousData.contactPhase_ = &conePrevious;
    if(contactsBreak.size() > 0){
        Ab = computeKinematicsConstraintsForState(fullbody,mid);
        midData.Kin_ = Ab.first;
        midData.kin_ = Ab.second;
    }
    centroidal_dynamics::Equilibrium coneMid = computeContactConeForState(fullbody,mid,successConeCurrent);
    successCone = successCone && successConeCurrent;
    midData.contactPhase_ = &coneMid;
    Ab = computeKinematicsConstraintsForState(fullbody,next);
    nextData.Kin_ = Ab.first;
    nextData.kin_ = Ab.second;
    centroidal_dynamics::Equilibrium coneNext = computeContactConeForState(fullbody,next,successConeCurrent);
    successCone = successCone && successConeCurrent;
    nextData.contactPhase_ = &coneNext;

    hppStopBenchmark(COMPUTE_DOUBLE_DESCRIPTION);
    hppDisplayBenchmark(COMPUTE_DOUBLE_DESCRIPTION);


    if(!successCone){
        return Result(UNABLE_TO_COMPUTE);
    }


    pData.contacts_.push_back(previousData);
    if(contactsBreak.size() == 1 && contactsCreation.size() == 1){
        hppDout(notice,"Contact break AND creation, create intermediate state");
        pData.contacts_.push_back(midData);
    }
    pData.contacts_.push_back(nextData);
    pData.constraints_.flag_ = bezier_com_traj::INIT_POS | bezier_com_traj::INIT_VEL | bezier_com_traj::INIT_ACC | bezier_com_traj::END_ACC | bezier_com_traj::END_VEL | bezier_com_traj::END_POS;
    pData.constraints_.constraintAcceleration_=true;
    pData.constraints_.maxAcceleration_=10.;
    pData.constraints_.reduce_h_ = 1e-3;
    // set init/goal values :
    pData.c0_ = com_previous;
    pData.c1_ = com_next;
    size_t id_velocity = fullbody->device_->configSize() - fullbody->device_->extraConfigSpace().dimension();
    pData.dc0_ = previous.configuration_.segment<3>(id_velocity);
    pData.dc1_ = next.configuration_.segment<3>(id_velocity);
    pData.ddc0_ = previous.configuration_.segment<3>(id_velocity+3); // unused for now
    pData.ddc1_ = next.configuration_.segment<3>(id_velocity+3);
    //pData.dc0_ = fcl::Vec3f::Zero();
   // pData.dc1_ = fcl::Vec3f::Zero();
    pData.ddc0_ = fcl::Vec3f::Zero();
    pData.ddc1_ = fcl::Vec3f::Zero();
    hppDout(notice,"Build pData : ");
    hppDout(notice,"c0 = "<<pData.c0_.transpose());
    hppDout(notice,"c1 = "<<pData.c1_.transpose());
    hppDout(notice,"dc0 = "<<pData.dc0_.transpose());
    hppDout(notice,"dc1 = "<<pData.dc1_.transpose());
    hppDout(notice,"ddc0 = "<<pData.ddc0_.transpose());
    hppDout(notice,"ddc1 = "<<pData.ddc1_.transpose());
    assert ((pData.contacts_.size() == 2 || pData.contacts_.size() == 3) && "Error in computation of contact constraints, must be only 2 or 3 phases.");

    // compute initial guess :
    //average of all contact point for the state with the less contacts, z = average of the CoM heigh between previous and next
    State lessContacts;
    if(contactsBreak.size()==1 && contactsCreation.size()==1)
        lessContacts = mid;
    else if (contactsBreak.size()==1)
        lessContacts = next;
    else if (contactsCreation.size() == 1)
        lessContacts = previous;
    else
        lessContacts = previous;


    VectorX current_timings;
    double total_time = 0;
    bool timing_provided(false);
    int t_id = 1;
    #if !FULL_TIME_SAMPLING
    MatrixXX timings_matrix;
    #endif
    hppDout(notice," timings provided size :  "<<timings.size());
    if(timings.size() != pData.contacts_.size()){
      // build timing vector
      // TODO : retrieve timing found by planning ?? how ?? (pass it as argument or store it inside the states ?)
      hppDout(notice,"Contact break and creation. Use hardcoded timing matrice");
      #if FULL_TIME_SAMPLING
        const double time_increment = 0.05;
        const double min_SS = 0.6;
        const double max_SS = 1.6;
        const double min_DS = 0.3;
        const double max_DS = 1.5;
        if(contactsBreak.size() == 1 && contactsCreation.size() == 1){
          current_timings = VectorX(3);
          //current_timings<<0.6,0.4,0.6; //hrp2
          //total_time = 1.6;
          current_timings<<min_DS,min_SS,min_DS; // hrp2
        }else{
          hppDout(notice,"Only two phases.");
          current_timings = VectorX(2);
          current_timings<<1.,1.;
        }
      #else
        timings_matrix = MatrixXX(18,3); // contain the timings of the first 3 phases, the total time and the discretization step
        timings_matrix <<
            0.6, 1.2, 0.6,
             1. , 1.2, 1.,
            0.8 , 0.7, 0.8,
            0.3 , 0.6, 0.3,
            0.5 , 0.6, 0.5,
            0.6 , 0.6, 0.6,
            0.8 , 0.6, 0.8,
            0.3 , 0.8, 0.3,
            0.3 , 0.7, 0.3,
            0.6 , 0.8, 0.6,
            1, 0.7, 1, // found with script
            1.5, 0.7, 1,
            0.8,0.8,0.8, // script good
            1. , 0.6, 1.,
            1.2 , 0.6, 1.2,
            1.5 , 0.6, 1.5,
            0.1 , 0.2, 0.1,
            0.2, 0.3, 0.2;
        current_timings = timings_matrix.block(0,0,1,pData.contacts_.size()).transpose();
      #endif
      total_time = 0;
      for(size_t i = 0 ; i < pData.contacts_.size() ; ++ i ){
        total_time += current_timings[i];
      }
    }else{
      hppDout(notice,"Timing vector is provided");
      current_timings = VectorX(timings.size());
      for(size_t i = 0 ; i < timings.size() ; ++i){
        current_timings[i] = timings[i];
        total_time += timings[i];
      }
      timing_provided = true;
    }

    //pData.representation_ = bezier_com_traj::FORCE;

    // loop over all possible timings :
    bool success(false);
    bool no_timings_left(false);
    bezier_com_traj::ResultDataCOMTraj resBezier;
    #if STAT_TIMINGS
    while(!no_timings_left){
    #else
    while(!success && !no_timings_left){
    #endif
        // call solveur :
        hppDout(notice,"Try with timings : "<<current_timings.transpose());
        hppDout(notice,"Call solveOneStep");
        hppStartBenchmark(SOLVE_TRANSITION_ONE_STEP);
        if(numPointsPerPhases > 0){
            hppDout(notice,"Call computeCOMTraj discretized");
            resBezier = bezier_com_traj::computeCOMTrajFixedSize(pData,current_timings,numPointsPerPhases);
        }
        else{
            hppDout(notice,"Call computeCOMTraj continuous");
            resBezier = bezier_com_traj::computeCOMTraj(pData,current_timings);
        }

        hppStopBenchmark(SOLVE_TRANSITION_ONE_STEP);
        hppDisplayBenchmark(SOLVE_TRANSITION_ONE_STEP);

        //wrap the result :
        if(resBezier.success_){
            hppDout(notice,"REACHABLE");
            res.status = REACHABLE;
            bezier_Ptr bezierCurve=bezier_Ptr(new bezier_t(resBezier.c_of_t_));
            hppDout(notice,"Resulting bezier curve have timing : "<<bezierCurve->max());
            for(bezier_t::cit_point_t wpit = bezierCurve->waypoints().begin() ; wpit != bezierCurve->waypoints().end() ; ++wpit){
                hppDout(notice,"with waypoint : "<<(*wpit).transpose());
            }
            // replace extra dof in next.configuration to fit the final velocity and acceleration found :
            next.configuration_.segment<3>(id_velocity) = resBezier.dc1_;
            next.configuration_.segment<3>(id_velocity+3) = resBezier.ddc1_;
            hppDout(notice,"new final configuration : "<<pinocchio::displayConfig(next.configuration_));
            res.path_ = BezierPath::create(fullbody->device_,bezierCurve,previous.configuration_,next.configuration_, core::interval_t(0.,total_time));
            hppDout(notice,"position of the waypoint : "<<resBezier.x.transpose());
            hppDout(notice,"With timings : "<< current_timings.transpose());
            hppDout(notice,"total time = "<<total_time);
            hppDout(notice,"new final velocity : "<<resBezier.dc1_.transpose());
            res.timings_ = current_timings;
            res.pathPerPhases_.push_back(BezierPath::create(fullbody->device_,bezierCurve,previous.configuration_,next.configuration_, core::interval_t(0.,current_timings[0])));
            res.pathPerPhases_.push_back(BezierPath::create(fullbody->device_,bezierCurve,previous.configuration_,next.configuration_, core::interval_t(current_timings[0],current_timings[0]+current_timings[1])));
            if(current_timings.size() > 2)
                res.pathPerPhases_.push_back(BezierPath::create(fullbody->device_,bezierCurve,previous.configuration_,next.configuration_, core::interval_t(current_timings[0]+current_timings[1],total_time)));
            success = true;
        }else{
            hppDout(notice,"UNREACHABLE");
            res.status = UNREACHABLE;
        }
        #if STAT_TIMINGS
        // print result in file :
        printTimingFile(file,current_timings,res.success(),quasiStaticSucces);
        #else
          (void)quasiStaticSucces; // silent warning
        #endif
        // build the new timing vector :

        if(!timing_provided){
            #if FULL_TIME_SAMPLING
            current_timings[0] +=time_increment;
            if(current_timings[0] > max_DS){
                current_timings[0] = min_DS;
                current_timings[1] += time_increment;
                if(current_timings[1] > max_SS){
                    if(current_timings.size() == 3){
                        current_timings[1] = min_SS;
                        current_timings[2] += time_increment;
                        if(current_timings[2] > max_DS){
                            no_timings_left = true;
                        }
                    }else{
                        no_timings_left = true;
                    }
                }
            }
            #else
            current_timings = timings_matrix.block(t_id,0,1,pData.contacts_.size()).transpose();
            t_id ++;
            if(t_id == timings_matrix.rows())
                no_timings_left = true;
            #endif
            total_time = 0;
            for(size_t i = 0 ; i < pData.contacts_.size() ; ++ i ){
                total_time += current_timings[i];
            }
        }else{
            no_timings_left = true;
        }
    }

    hppStopBenchmark(IS_REACHABLE_DYNAMIC);
    hppDisplayBenchmark(IS_REACHABLE_DYNAMIC);

    delete pData.contacts_[0].contactPhase_;
    delete pData.contacts_[1].contactPhase_;
    if(pData.contacts_.size() > 2)
        delete pData.contacts_[2].contactPhase_;



    #if STAT_TIMINGS
    file.close();
    return Result(REACHABLE);
    #endif

    if(!success){
        hppDout(notice,"No valid timings found, always UNREACHABLE");
    }

    if(success && tryQuasiStatic){
        hppDout(notice,"ONLY REACHABLE IN DYNAMIC !!!");
    }
    hppDout(notice,"Between configuration : r(["<<pinocchio::displayConfig(previous.configuration_)<<"])");
    hppDout(notice,"and     configuration : r(["<<pinocchio::displayConfig(next.configuration_)<<"])");


    return res;
}

}
}
}
