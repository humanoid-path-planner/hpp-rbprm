// Copyright (c) 2014, LAAS-CNRS
// Authors: Steve Tonneau (steve.tonneau@laas.fr)
//
// This file is part of hpp-rbprm.
// hpp-rbprm is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-rbprm is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-rbprm. If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_RBPRM_PLANNER_HH
# define HPP_RBPRM_PLANNER_HH

#include <hpp/util/pointer.hh>

#include <hpp/core/path-planner.hh>

namespace hpp {
// forward declaration of class Planner
HPP_PREDEF_CLASS (RbPrmPlanner);
// Planner objects are manipulated only via shared pointers
typedef boost::shared_ptr <RbPrmPlanner> RbPrmPlannerPtr_t;

    /// \addtogroup path_planning
    /// \{

    /// RBPRM planner
    ///
    /// Implementation of the Reachability based algorithm.
    /// Planning generates configurations respecting the "Reachability condition":
    /// a dual abstract representation of the robot is used, representing the reachability space
    /// of its limbs one the one hand (A_{ROM}), and the integrity of the robot trunk on the other hand (A_{TRUNK}).
    /// If the generated configuration has a collision between A_{ROM} and the environment, and if A_{TRUNK} is collision free,
    /// then the configuration is considered valid.
    class RbPrmPlanner : public core::PathPlanner
    {
    public:
      /// Create an instance and return a shared pointer to the instance
      static RbPrmPlannerPtr_t create (const core::Problem& problem,
                  const core::RoadmapPtr_t& roadmap)
      {
        RbPrmPlanner* ptr = new RbPrmPlanner (problem, roadmap);
        RbPrmPlannerPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      virtual void oneStep ();

    protected:
      /// Protected constructor
      /// Users need to call RbPrmPlanner::create in order to create instances.
      RbPrmPlanner (const core::Problem& problem,
           const core::RoadmapPtr_t& roadmap);

      void init (const RbPrmPlannerWkPtr_t& weak);

    private:
      /// weak pointer to itself
      RbPrmPlannerWkPtr_t weakPtr_;
      /// \}
    }; // class RbPrmPlanner
} // namespace hpp
# endif // HPP_RBPRM_PLANNER_HH
