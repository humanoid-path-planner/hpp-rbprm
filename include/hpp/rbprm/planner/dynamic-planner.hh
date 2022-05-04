//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
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
#define HPP_RBPRM_DYNAMIC_PLANNER_HH

#include <hpp/core/bi-rrt-planner.hh>
#include <hpp/rbprm/planner/rbprm-steering-kinodynamic.hh>
#include <hpp/rbprm/planner/steering-method-parabola.hh>
#include <hpp/rbprm/rbprm-path-validation.hh>

namespace hpp {
namespace rbprm {
/// \addtogroup path_planning
/// \{
// forward declaration of class Planner
HPP_PREDEF_CLASS(DynamicPlanner);
// Planner objects are manipulated only via shared pointers
typedef shared_ptr<DynamicPlanner> DynamicPlannerPtr_t;

using core::Configuration_t;
using core::Path;
using core::PathPtr_t;
using core::Problem;
using core::Roadmap;
using core::RoadmapPtr_t;

/// Generic implementation of RRT algorithm
class DynamicPlanner : public core::BiRRTPlanner {
 public:
  /// Return shared pointer to new object.
  static DynamicPlannerPtr_t createWithRoadmap(core::ProblemConstPtr_t problem,
                                               const RoadmapPtr_t& roadmap);
  /// Return shared pointer to new object.
  static DynamicPlannerPtr_t create(core::ProblemConstPtr_t problem);
  /// One step of extension.
  virtual void oneStep();
  /// Try to make direct connection between init and goal
  /// configurations, in order to avoid a random shoot.
  virtual void tryConnectInitAndGoals();

  // we need both method, because smart_pointer inheritance is not implemented
  // (compiler don't know that rbprmRoadmapPtr_t derive from RoadmapPtr_t).
  virtual const core::RoadmapPtr_t& roadmap() const { return roadmap_; }

  virtual core::PathVectorPtr_t finishSolve(const core::PathVectorPtr_t& path);

 protected:
  /// Constructor
  DynamicPlanner(core::ProblemConstPtr_t problem, const RoadmapPtr_t& roadmap);
  /// Constructor with roadmap
  DynamicPlanner(core::ProblemConstPtr_t problem);
  /// Store weak pointer to itself
  void init(const DynamicPlannerWkPtr_t& weak);

  /**
   * @brief computeGIWC compute the GIWC for the node configuration and fill the
   * node attribut
   * @param x the node
   * @param report the RBPRM report corresponding to the node's configuration
   */
  void computeGIWC(const core::NodePtr_t x, core::ValidationReportPtr_t report);

  /**
   * @brief computeGIWC compute the GIWC for the node configuration and fill the
   * node attribut, get validation report and call the second method
   * @param x the node
   */
  void computeGIWC(const core::NodePtr_t x, bool use_bestReport = true);

  core::PathPtr_t extendInternal(core::ConfigurationPtr_t& qProj_,
                                 const core::NodePtr_t& near,
                                 const core::ConfigurationPtr_t& target,
                                 bool reverse = false);

  bool tryParabolaPath(const core::NodePtr_t& near,
                       core::ConfigurationPtr_t q_jump,
                       const core::ConfigurationPtr_t& target, bool reverse,
                       core::NodePtr_t& x_jump, core::NodePtr_t& nodeReached,
                       core::PathPtr_t& kinoPath, core::PathPtr_t& paraPath);

  core::PathPtr_t extendParabola(const core::ConfigurationPtr_t& from,
                                 const core::ConfigurationPtr_t& target,
                                 bool reverse);

 private:
  core::ConfigurationPtr_t qProj_;
  DynamicPlannerWkPtr_t weakPtr_;
  const core::RoadmapPtr_t roadmap_;
  const SteeringMethodKinodynamicPtr_t sm_;
  const SteeringMethodParabolaPtr_t smParabola_;
  const RbPrmPathValidationPtr_t rbprmPathValidation_;
  double sizeFootX_, sizeFootY_;
  bool rectangularContact_;
  bool tryJump_;
  double mu_;
};
/// \}
}  // namespace rbprm
}  // namespace hpp
#endif  // HPP_RBPRM_DYNAMIC_PLANNER_HH
