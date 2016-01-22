#ifndef STEERINGDYNAMIC_HH
#define STEERINGDYNAMIC_HH

# include <hpp/core/steering-method.hh>
# include <hpp/core/straight-path.hh>
# include <hpp/core/weighed-distance.hh>


namespace hpp {
  namespace rbprm {
    /// \addtogroup steering_method
    /// \{


    // forward declaration of class
    HPP_PREDEF_CLASS (SteeringDynamic);
    // Planner objects are manipulated only via shared pointers
    typedef boost::shared_ptr <SteeringDynamic> SteeringDynamicPtr_t;


    /// Steering method that creates StraightPath instances
    ///
    class SteeringDynamic : public core::SteeringMethod
    {
    public:
      /// Create instance and return shared pointer
      static SteeringDynamicPtr_t create (const core::ProblemPtr_t& problem)
      {
        SteeringDynamic* ptr = new SteeringDynamic (problem);
        SteeringDynamicPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }
      /// Create instance and return shared pointer
      static SteeringDynamicPtr_t create
        (const core::ProblemPtr_t& problem, const core::WeighedDistancePtr_t& distance)
      {
        SteeringDynamic* ptr = new SteeringDynamic (problem, distance);
        SteeringDynamicPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }
      /// Copy instance and return shared pointer
      static SteeringDynamicPtr_t createCopy
        (const SteeringDynamicPtr_t& other)
      {
        SteeringDynamic* ptr = new SteeringDynamic (*other);
        SteeringDynamicPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }
      /// Copy instance and return shared pointer
      virtual core::SteeringMethodPtr_t copy () const
      {
        return createCopy (weak_.lock ());
      }

      /// create a path between two configurations
      virtual core::PathPtr_t impl_compute (core::ConfigurationIn_t q1,
                                      core::ConfigurationIn_t q2) const
      {
        core::value_type length = (*distance_) (q1, q2);
        core::PathPtr_t path = core::StraightPath::create (problem_->robot(), q1, q2, length,constraints ());
        return path;
      }
    protected:
      /// Constructor with robot
      /// Weighed distance is created from robot
      SteeringDynamic (const core::ProblemPtr_t& problem) :
        SteeringMethod (problem),
        distance_ (core::WeighedDistance::create (problem->robot())), weak_ ()
      {
      }
      /// Constructor with weighed distance
      SteeringDynamic (const core::ProblemPtr_t& problem,
                              const core::WeighedDistancePtr_t& distance) :
        SteeringMethod (problem),
        distance_ (distance), weak_ ()
      {
      }
      /// Copy constructor
      SteeringDynamic (const SteeringDynamic& other) :
        SteeringMethod (other),
        distance_ (other.distance_), weak_ ()
      {
      }

      /// Store weak pointer to itself
      void init (SteeringDynamicWkPtr_t weak)
      {
        SteeringMethod::init (weak);
        weak_ = weak;
      }
    private:
      core::WeighedDistancePtr_t distance_;
      SteeringDynamicWkPtr_t weak_;
    }; // SteeringMethodStraight
    /// \}
  } // namespace core
} // namespace hpp

#endif // STEERINGDYNAMIC_HH
