#ifndef HPP_RBPRM_REACHABILITY_HH
#define HPP_RBPRM_REACHABILITY_HH

#include <hpp/rbprm/rbprm-limb.hh>
#include <hpp/centroidal-dynamics/centroidal_dynamics.hh>
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
            x(fcl::Vec3f::Zero()),
            xBreak_(fcl::Vec3f::Zero()),
            xCreate_(fcl::Vec3f::Zero()),
            constraints_(),
            path_()
            {}

        Result(Status status):
            status(status),
            x(fcl::Vec3f::Zero()),
            xBreak_(fcl::Vec3f::Zero()),
            xCreate_(fcl::Vec3f::Zero()),
            constraints_(),
            path_()
            {}

        Result(Status status, fcl::Vec3f x):
            status(status),
            x(x),
            xBreak_(fcl::Vec3f::Zero()),
            xCreate_(fcl::Vec3f::Zero()),
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
        fcl::Vec3f xBreak_;
        fcl::Vec3f xCreate_;
        std::pair<MatrixXX, VectorX> constraints_;
        core::PathPtr_t path_;
        VectorX timings_;
        std::vector<core::PathPtr_t> pathPerPhases_;
    };


std::pair<MatrixXX, VectorX> stackConstraints(const std::pair<MatrixXX, VectorX> &Ab, const std::pair<MatrixXX, VectorX> &Cd);

bool intersectionExist(const std::pair<MatrixXX, VectorX>& Ab, const fcl::Vec3f& c, fcl::Vec3f &c_out);

std::pair<MatrixXX, VectorX> computeStabilityConstraints(const centroidal_dynamics::Equilibrium& contactPhase,const fcl::Vec3f& int_point = fcl::Vec3f(0,0,0),const fcl::Vec3f& acc = fcl::Vec3f(0,0,0));

std::pair<MatrixXX, VectorX> computeStabilityConstraintsForState(const RbPrmFullBodyPtr_t& fullbody, State &state, bool &success, const fcl::Vec3f &acc = fcl::Vec3f::Zero());

std::pair<MatrixXX, VectorX> computeConstraintsForState(const RbPrmFullBodyPtr_t& fullbody, State &state,bool& success);

/**
 * @brief isReachable Compute the feasibility of the contact transition between the two state, with the quasiStatic formulation of 2-PAC (https://hal.archives-ouvertes.fr/hal-01609055)
 * @param fullbody
 * @param previous the first state of the transition
 * @param next the last state of the transition
 * @param acc the CoM acceleration
 * @param useIntermediateState boolean only relevant in the case of a contact repositionning. If true, use an intermediate state such that there is only one contact change between each state.
 *                              (and thus compute two intersection between 3 set of constrants). If false it only compute one intersection between two set of constraints.
 * @return
 */
Result isReachable(const RbPrmFullBodyPtr_t& fullbody,State &previous, State& next,const fcl::Vec3f& acc = fcl::Vec3f::Zero(), bool useIntermediateState = false);

Result isReachableDynamic(const RbPrmFullBodyPtr_t& fullbody, State &previous, State& next, bool tryQuasiStatic = false, std::vector<double> timings = std::vector<double>(), int numPointsPerPhases = 0);



}
}
}
#endif // REACHABILITY_HH
