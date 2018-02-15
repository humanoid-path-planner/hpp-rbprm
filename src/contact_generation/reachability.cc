#include <hpp/rbprm/contact_generation/reachability.hh>
#include <bezier-com-traj/solve.hh>
#include <bezier-com-traj/common_solve_methods.hh>
#include <hpp/rbprm/rbprm-fullbody.hh>
#include <hpp/rbprm/stability/stability.hh>
#include <iostream>
#include <fstream>
#include <hpp/util/timer.hh>

#ifndef QHULL
#define QHULL 1
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
        //hppDout(notice,ss.str());
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
    #endif
    return std::make_pair(A,b);
}

centroidal_dynamics::Equilibrium computeContactConeForState(const RbPrmFullBodyPtr_t& fullbody, State &state){
    hppStartBenchmark(REACHABLE_CALL_CENTROIDAL);
    centroidal_dynamics::Equilibrium contactCone(fullbody->device_->name(), fullbody->device_->mass(),4,centroidal_dynamics::SOLVER_LP_QPOASES,true,10,false);
    centroidal_dynamics::EquilibriumAlgorithm alg = centroidal_dynamics::EQUILIBRIUM_ALGORITHM_PP;
    stability::setupLibrary(fullbody,state,contactCone,alg);
    hppStopBenchmark(REACHABLE_CALL_CENTROIDAL);
    hppDisplayBenchmark(REACHABLE_CALL_CENTROIDAL);
    return contactCone;
}

std::pair<MatrixXX, VectorX> computeStabilityConstraintsForState(const RbPrmFullBodyPtr_t& fullbody, State &state,const fcl::Vec3f& acc){
    return computeStabilityConstraints(computeContactConeForState(fullbody,state),state.contactPositions_.at(state.contactOrder_.front()),
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
    #if QHULL
    hppDout(notice,"constraint for contact break : ");
    printQHull(resBreak.constraints_,resBreak.x,"constraints_break.txt");
    hppDout(notice,"constraint for contact creation : ");
    printQHull(resCreate.constraints_,resCreate.x,"constraints_create.txt");
    #endif
    if(resBreak.success() && resCreate.success()){
        res.status=REACHABLE;
        res.x = (resBreak.x + resCreate.x)/2.;
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
    hppDout(notice,"IsReachable called : for configuration :  r(["<<model::displayConfig(previous.configuration_)<<"])");
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
            int_pt_kin = previous.com_;
            intersectionExist(A_n,previous.com_,next.com_,int_pt_stab);
        }else{
            int_pt_kin = next.com_;
            intersectionExist(A_p,previous.com_,next.com_,int_pt_stab);
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
    printQHull(Ab,int_pt_kin,"constraints.txt");

    if(contactsCreation.size() <= 0 && contactsBreak.size() <= 0){
        hppDout(notice,"No contact variation, abort.");
        return Result(NO_CONTACT_VARIATION);
    }
    #endif

    return res;
}

Result isReachableDynamic(const RbPrmFullBodyPtr_t& fullbody, State &previous, State& next,bool tryQuasiStatic, std::vector<double> timings, int numPointsPerPhases,double feasabilityTreshold){
    Result res;
    std::vector<std::string> contactsCreation, contactsBreak;
    next.contactBreaks(previous,contactsBreak);
    next.contactCreations(previous,contactsCreation);
    hppDout(notice,"IsReachableDynamic called : ");
    hppDout(notice,"Between configuration : "<<model::displayConfig(previous.configuration_));
    hppDout(notice,"and     configuration : "<<model::displayConfig(next.configuration_));

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


    if(tryQuasiStatic){
        hppDout(notice,"Try quasi-static reachability : ");
        Result quasiStaticResult = isReachableIntermediate(fullbody,previous,mid,next);
        if(quasiStaticResult.success()){
            hppDout(notice,"REACHABLE in quasi-static");
            quasiStaticResult.status=QUASI_STATIC;
            // build a Bezier curve of order 2 :
            std::vector<Vector3> wps;
            wps.push_back(previous.com_);
            wps.push_back(quasiStaticResult.x);
            wps.push_back(next.com_);
            bezier_Ptr bezierCurve=bezier_Ptr(new bezier_t(wps.begin(),wps.end(),1.));
            quasiStaticResult.path_ = BezierPath::create(fullbody->device_,bezierCurve,previous.configuration_,next.configuration_, core::interval_t(0.,1));
            return quasiStaticResult;
        }else{
            hppDout(notice,"UNREACHABLE in quasi-static");
        }
    }


    // build ProblemData from states object and call solveOneStep()
    bezier_com_traj::ProblemData pData;
    // build contactPhases :
    bezier_com_traj::ContactData previousData,nextData,midData;
    std::pair<MatrixX3,VectorX> Ab = computeKinematicsConstraintsForState(fullbody,previous);
    previousData.Kin_ = Ab.first;
    previousData.kin_ = Ab.second;
    centroidal_dynamics::Equilibrium conePrevious = computeContactConeForState(fullbody,previous);
    previousData.contactPhase_ = &conePrevious;
    if(contactsBreak.size() > 0){
        Ab = computeKinematicsConstraintsForState(fullbody,mid);
        midData.Kin_ = Ab.first;
        midData.kin_ = Ab.second;
    }
    centroidal_dynamics::Equilibrium coneMid = computeContactConeForState(fullbody,mid);
    midData.contactPhase_ = &coneMid;
    Ab = computeKinematicsConstraintsForState(fullbody,next);
    nextData.Kin_ = Ab.first;
    nextData.kin_ = Ab.second;
    centroidal_dynamics::Equilibrium coneNext = computeContactConeForState(fullbody,next);
    nextData.contactPhase_ = &coneNext;

    pData.contacts_.push_back(previousData);
    if(contactsBreak.size() == 1 && contactsCreation.size() == 1){
        hppDout(notice,"Contact break AND creation, create intermediate state");
        pData.contacts_.push_back(midData);
    }
    pData.contacts_.push_back(nextData);

    // set init/goal values :
    pData.c0_ = previous.com_;
    pData.c1_ = next.com_;
    size_t id_velocity = fullbody->device_->configSize() - fullbody->device_->extraConfigSpace().dimension();
    pData.dc0_ = previous.configuration_.segment<3>(id_velocity);
    pData.dc1_ = next.configuration_.segment<3>(id_velocity);
    pData.ddc0_ = previous.configuration_.segment<3>(id_velocity+3); // unused for now
    pData.ddc1_ = next.configuration_.segment<3>(id_velocity+3);
    pData.dc0_ = fcl::Vec3f::Zero();
    pData.dc1_ = fcl::Vec3f::Zero();
    pData.ddc0_ = fcl::Vec3f::Zero();
    pData.ddc1_ = fcl::Vec3f::Zero();
    hppDout(notice,"Build pData : ");
    hppDout(notice,"c0 = "<<pData.c0_.transpose());
    hppDout(notice,"c1 = "<<pData.c1_.transpose());
    hppDout(notice,"dc0 = "<<pData.dc0_.transpose());
    hppDout(notice,"dc1 = "<<pData.dc1_.transpose());
    hppDout(notice,"ddc0 = "<<pData.ddc0_.transpose());
    hppDout(notice,"ddc1 = "<<pData.ddc1_.transpose());


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
    fcl::Vec3f init_guess = fcl::Vec3f::Zero();
    for(std::map<std::string, fcl::Vec3f>::const_iterator cit = lessContacts.contactPositions_.begin() ; cit != lessContacts.contactPositions_.end() ; ++cit){
        init_guess += cit->second;
    }
    init_guess /= lessContacts.contactPositions_.size();
    init_guess[2] = (previous.com_[2] + next.com_[2])/2.;




   // MatrixXX timings_matrix; // contain the timings of the first 3 phases, the total time and the discretization step
    VectorX current_timings;
    VectorX times;
    double total_time = 0;
    double time_increment = 0.2;
    bool timing_provided(false);
    hppDout(notice," timings provided size :  "<<timings.size());
    if(timings.size() != pData.contacts_.size()){
        // build timing vector, it should be a multiple of the timeStep
        // TODO : retrieve timing found by planning ?? how ?? (pass it as argument or store it inside the states ?)
        // hardcoded value for now : 0.2 s double support, 0.8s single support
        if(contactsBreak.size() == 1 && contactsCreation.size() == 1){
            hppDout(notice,"Contact break and creation. Use hardcoded timing matrice");
            /*timings_matrix = MatrixXX(10,5);
            timings_matrix << 1.,1.,1.,3.,0.1,
                            0.4,0.6,0.4,1.4,0.1,
                            0.6,0.4,0.6,1.6,0.1,
                            0.6,0.6,0.6,1.8,0.1,
                            0.2,0.2,0.2,0.6,0.1,
                            1.,1.,1.,3.,0.2,
                            1.6,1.6,1.6,4.8,0.2,
                            2.,1.,2.,5.,0.2,
                            2.,1.,2.,5.,0.5,
                            2.,2.,2.,5.,0.5;
                            */
            current_timings = VectorX(3);
            //current_timings<<0.6,0.4,0.6;
            //total_time = 1.6;
            current_timings<<0.2,0.2,0.2;
            total_time = 0.6;
        }else{
            hppDout(notice,"Only two phases.");
            current_timings = VectorX(2);
            current_timings<<1.,1.;
            total_time = 2.;
        }
    }else{
        hppDout(notice,"Timing vector is provided");
        /*timings_matrix = MatrixXX(1,5);
        timings_matrix(0,0) = timings[0];
        timings_matrix(0,1) = timings[1];
        timings_matrix(0,2) = timings[2];
        timings_matrix(0,3) = timings[0] + timings[1] + timings[2];
        timings_matrix(0,4) = timeStep;*/
        current_timings = VectorX(timings.size());
        for(size_t i = 0 ; i < timings.size() ; ++i){
            current_timings[i] = timings[i];
            total_time += timings[i];
        }
        timing_provided = true;
    }


    // loop over all possible timings :
    bool success(false);
    bool no_timings_left(false);
    bezier_com_traj::ResultDataCOMTraj resBezier;
    while(!success && !no_timings_left){
        // call solveur :
        hppDout(notice,"Try with timings : "<<current_timings.transpose());
        hppDout(notice,"Call solveOneStep");
        resBezier = bezier_com_traj::solveOnestep(pData,current_timings,init_guess,numPointsPerPhases,feasabilityTreshold);
        //wrap the result :
        if(resBezier.success_){
            hppDout(notice,"REACHABLE");
            res.status = REACHABLE;
            bezier_Ptr bezierCurve=bezier_Ptr(new bezier_t(resBezier.c_of_t_));
            // replace extra dof in next.configuration to fit the final velocity and acceleration found :
            next.configuration_.segment<3>(id_velocity) = resBezier.dc1_;
            next.configuration_.segment<3>(id_velocity+3) = resBezier.ddc1_;
            hppDout(notice,"new final configuration : "<<model::displayConfig(next.configuration_));
            res.path_ = BezierPath::create(fullbody->device_,bezierCurve,previous.configuration_,next.configuration_, core::interval_t(0.,total_time));
            hppDout(notice,"position of the waypoint : "<<resBezier.x.transpose());
            hppDout(notice,"With timings : "<< current_timings.transpose());
            success = true;
        }else{
            hppDout(notice,"UNREACHABLE");
            res.status = UNREACHABLE;
        }
        // build the new timing vector :
        if(!timing_provided){
            current_timings[0] +=time_increment;
            if(current_timings[0] > 2.){
                current_timings[0] = 0.2;
                current_timings[1] += time_increment;
                if(current_timings[1] > 2.){
                    if(current_timings.size() == 3){
                        current_timings[1] = 0.2;
                        current_timings[2] += time_increment;
                        if(current_timings[2] > 2.){
                            no_timings_left = true;
                        }
                    }else{
                        no_timings_left = true;
                    }
                }
            }
            total_time = current_timings[0] + current_timings[1] + current_timings[2];
        }else{
            no_timings_left = true;
        }
    }
    if(!resBezier.success_){
        hppDout(notice,"No valid timings found, always UNREACHABLE");
    }

    if(res.success() && tryQuasiStatic){
        hppDout(notice,"ONLY REACHABLE IN DYNAMIC !!!");
    }



    // test : compare to 0step :
    // only work with 2 phases !!
    /*
    hppDout(notice,"Compare with 0Step : ");
    hppDout(notice," time = "<<total_time);
    bezier_com_traj::ProblemData pData0;
    pData0.c0_ = pData.c0_;
    pData0.c1_ = pData.c1_;
    pData0.dc0_ = pData.dc0_;
    pData0.dc1_ = pData.dc1_;
    pData0.ddc0_ = pData.ddc0_;
    pData0.ddc1_ = pData.ddc1_;
    pData0.contacts_.push_back(previousData);
    std::vector<double> Ts;
    Ts.push_back(total_time);
    double timeStep0 = total_time/6.;
    bezier_com_traj::ResultDataCOMTraj res0 = bezier_com_traj::solve0step(pData0,Ts,timeStep0);
    hppDout(notice,"success 0 step : "<<res0.success_);
    hppDout(notice,"x 0 step : "<<res0.x.transpose());
    */
    return res;
}

}
}
}
