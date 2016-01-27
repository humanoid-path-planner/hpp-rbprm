//
// Copyright (c) 2016 CNRS
// Authors: Fernbach Pierre
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_RBPRM_DYNAMIC_PLANNER_HH
# define HPP_RBPRM_DYNAMIC_PLANNER_HH

# include <hpp/core/path-planner.hh>

namespace hpp {
  namespace rbprm {
    /// \addtogroup path_planning
    /// \{

    // forward declaration of class Planner
    HPP_PREDEF_CLASS (DynamicPlanner);
    // Planner objects are manipulated only via shared pointers
    typedef boost::shared_ptr <DynamicPlanner> DynamicPlannerPtr_t;


    /// Generic implementation of RRT algorithm
    class  DynamicPlanner : public core::PathPlanner
    {
    public:
      /// Return shared pointer to new object.
      static DynamicPlannerPtr_t createWithRoadmap
        (const core::Problem& problem, const core::RoadmapPtr_t& roadmap);
      /// Return shared pointer to new object.
      static DynamicPlannerPtr_t create (const core::Problem& problem);
      /// One step of extension.
      virtual void oneStep ();

      virtual void startSolve ();

      /// Set configuration shooter.
      void configurationShooter (const core::ConfigurationShooterPtr_t& shooter);
    protected:
      /// Constructor
      DynamicPlanner (const core::Problem& problem, const core::RoadmapPtr_t& roadmap);
      /// Constructor with roadmap
      DynamicPlanner (const core::Problem& problem);
      /// Store weak pointer to itself
      void init (const DynamicPlannerWkPtr_t& weak);
      /// Extend a node in the direction of a configuration
      /// \param near node in the roadmap,
      /// \param target target configuration
      virtual core::PathPtr_t extend (const core::NodePtr_t& near,
                                const core::ConfigurationPtr_t& target);
    private:
      core::ConfigurationShooterPtr_t configurationShooter_;
      mutable core::Configuration_t qProj_;
      DynamicPlannerWkPtr_t weakPtr_;
    };
    /// \}
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_DIFFUSING_PLANNER_HH
