#ifndef DynamicPlanner_H
#define DynamicPlanner_H


#include <hpp/util/pointer.hh>

#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/problem.hh>


namespace hpp {
  namespace rbprm {
    // forward declaration of class Planner
    HPP_PREDEF_CLASS (DynamicPlanner);
    // Planner objects are manipulated only via shared pointers
    typedef boost::shared_ptr <DynamicPlanner> DynamicPlannerPtr_t;

    /// Example of path planner
    class DynamicPlanner : public core::PathPlanner
    {
      public:
        /// Create an instance and return a shared pointer to the instance
        static DynamicPlannerPtr_t create (const core::Problem& problem);
        static DynamicPlannerPtr_t createWithRoadmap(const core::Problem& problem, const core::RoadmapPtr_t& roadmap);
        void configurationShooter (const core::ConfigurationShooterPtr_t& shooter);

        virtual void startSolve ();

        virtual void oneStep ();

      protected:
        /// Protected constructor
        /// Users need to call Planner::create in order to create instances.
        DynamicPlanner (const core::Problem& problem,const core::RoadmapPtr_t& roadmap);
        DynamicPlanner (const core::Problem &problem);

        /// Extend a node in the direction of a configuration
        /// \param near node in the roadmap,
        /// \param target target configuration
        virtual core::PathPtr_t extend (const core::NodePtr_t& near,
                                  const core::ConfigurationPtr_t& target);

        /// Store weak pointer to itself
        void init (const DynamicPlannerWkPtr_t& weak);
      private:
        /// Configuration shooter to uniformly shoot random configurations
        core::ConfigurationShooterPtr_t configurationShooter_;
        mutable core::Configuration_t qProj_;
        /// weak pointer to itself
        DynamicPlannerWkPtr_t weakPtr_;
        core::SteeringMethodPtr_t sm_;
    }; // class Planner
  } // namespace rbprm
} // namespace hpp



#endif
