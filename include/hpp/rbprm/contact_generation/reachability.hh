#ifndef HPP_RBPRM_REACHABILITY_HH
#define HPP_RBPRM_REACHABILITY_HH

#include <hpp/rbprm/rbprm-limb.hh>
#include <centroidal-dynamics-lib/centroidal_dynamics.hh>
#include <hpp/rbprm/rbprm-state.hh>
#include <hpp/rbprm/contact_generation/kinematics_constraints.hh>
#include <hpp/core/path.hh>
namespace hpp {
  namespace rbprm {

  HPP_PREDEF_CLASS(RbPrmFullBody);
  class RbPrmFullBody;
  typedef boost::shared_ptr <RbPrmFullBody> RbPrmFullBodyPtr_t;


    namespace reachability{


    enum Status{
        UNREACHABLE,
        UNABLE_TO_COMPUTE,
        SAME_ROOT_POSITION,
        TOO_MANY_CONTACTS_VARIATION,
        NO_CONTACT_VARIATION,
        CONTACT_BREAK_FAILED,
        CONTACT_CREATION_FAILED,
        QUASI_STATIC,
        REACHABLE
    };

    struct Result{

        Result():
            status(UNABLE_TO_COMPUTE),
            x(fcl::Vec3f()),
            constraints_(),
            path_()
            {}

        Result(Status status):
            status(status),
            x(fcl::Vec3f()),
            constraints_(),
            path_()
            {}

        Result(Status status, fcl::Vec3f x):
            status(status),
            x(x),
            constraints_(),
            path_()
            {}

        bool success(){return (status == REACHABLE) || (status == NO_CONTACT_VARIATION) || (status == SAME_ROOT_POSITION) || status == QUASI_STATIC;}

        bool pathExist(){
            if(path_){
                if (path_->length() > 0){
                    return true;
                }
            }
            return false;
        }

        Status status;
        fcl::Vec3f x;
        std::pair<MatrixXX, VectorX> constraints_;
        core::PathPtr_t path_;
        VectorX timings_;
        std::vector<core::PathPtr_t> pathPerPhases_;
    };


std::pair<MatrixXX, VectorX> stackConstraints(const std::pair<MatrixXX, VectorX> &Ab, const std::pair<MatrixXX, VectorX> &Cd);

bool intersectionExist(const std::pair<MatrixXX, VectorX>& Ab, const fcl::Vec3f& c0, const fcl::Vec3f& c1, fcl::Vec3f &c_out);

std::pair<MatrixXX, VectorX> computeStabilityConstraints(const centroidal_dynamics::Equilibrium& contactPhase,const fcl::Vec3f& int_point = fcl::Vec3f(0,0,0),const fcl::Vec3f& acc = fcl::Vec3f(0,0,0));

std::pair<MatrixXX, VectorX> computeStabilityConstraintsForState(const RbPrmFullBodyPtr_t& fullbody, State &state, const fcl::Vec3f &acc = fcl::Vec3f::Zero());

std::pair<MatrixXX, VectorX> computeConstraintsForState(const RbPrmFullBodyPtr_t& fullbody, State &state);

Result isReachable(const RbPrmFullBodyPtr_t& fullbody,State &previous, State& next,const fcl::Vec3f& acc = fcl::Vec3f::Zero());

Result isReachableDynamic(const RbPrmFullBodyPtr_t& fullbody, State &previous, State& next, bool tryQuasiStatic = true, std::vector<double> timings = std::vector<double>(), int numPointsPerPhases = 4, double feasabilityTreshold = 1e-3);



}
}
}
#endif // REACHABILITY_HH
