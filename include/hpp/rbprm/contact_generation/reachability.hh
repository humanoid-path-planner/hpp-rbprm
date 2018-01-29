#ifndef HPP_RBPRM_REACHABILITY_HH
#define HPP_RBPRM_REACHABILITY_HH

#include <hpp/rbprm/rbprm-limb.hh>
#include <centroidal-dynamics-lib/centroidal_dynamics.hh>
#include <hpp/rbprm/rbprm-state.hh>
#include <hpp/rbprm/contact_generation/kinematics_constraints.hh>

namespace hpp {
  namespace rbprm {

  HPP_PREDEF_CLASS(RbPrmFullBody);
  class RbPrmFullBody;
  typedef boost::shared_ptr <RbPrmFullBody> RbPrmFullBodyPtr_t;


    namespace reachability{


    enum Status{
        UNREACHABLE,
        UNABLE_TO_COMPUTE,
        TOO_MANY_CONTACTS_VARIATION,
        NO_CONTACT_VARIATION,
        CONTACT_BREAK_FAILED,
        CONTACT_CREATION_FAILED,
        REACHABLE
    };

    struct Result{

        Result():
            status(UNABLE_TO_COMPUTE),
            x(fcl::Vec3f())
            {}

        Result(Status status):
            status(status),
            x(fcl::Vec3f())
            {}

        Result(Status status, fcl::Vec3f x):
            status(status),
            x(x)
            {}

        bool success(){return status == REACHABLE;}

        Status status;
        fcl::Vec3f x;
    };

std::pair<MatrixXX, VectorX> stackConstraints(const std::pair<MatrixXX, VectorX> &Ab, const std::pair<MatrixXX, VectorX> &Cd);

bool intersectionExist(const std::pair<MatrixXX, VectorX>& Ab, const fcl::Vec3f& c0,const fcl::Vec3f& c1, fcl::Vec3f c_out);

std::pair<MatrixXX, VectorX> computeStabilityConstraints(const centroidal_dynamics::Equilibrium& contactPhase,const fcl::Vec3f& int_point = fcl::Vec3f(0,0,0));

std::pair<MatrixXX, VectorX> computeStabilityConstraintsForState(const RbPrmFullBodyPtr_t& fullbody,State &state);

std::pair<MatrixXX, VectorX> computeConstraintsForState(const RbPrmFullBodyPtr_t& fullbody, State &state);

Result isReachable(const RbPrmFullBodyPtr_t& fullbody,State &previous, State& next);

}
}
}
#endif // REACHABILITY_HH
