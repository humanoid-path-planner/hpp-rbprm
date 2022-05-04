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

#ifndef HPP_RBPRM_PARABOLA_PLANNER_HH
#define HPP_RBPRM_PARABOLA_PLANNER_HH

#include <boost/tuple/tuple.hpp>
#include <hpp/core/path-planner.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/rbprm/planner/rbprm-roadmap.hh>
#include <hpp/rbprm/planner/steering-method-parabola.hh>

namespace hpp {
namespace rbprm {
/// \addtogroup path_planning
/// \{

// forward declaration of class Planner
HPP_PREDEF_CLASS(ParabolaPlanner);
// Planner objects are manipulated only via shared pointers
typedef shared_ptr<ParabolaPlanner> ParabolaPlannerPtr_t;
typedef boost::tuple<core::NodePtr_t, core::ConfigurationPtr_t, core::PathPtr_t>
    DelayedEdge_t;
typedef std::vector<DelayedEdge_t> DelayedEdges_t;

/// Generic implementation of RRT algorithm
class ParabolaPlanner : public core::PathPlanner {
 public:
  /// Return shared pointer to new object.
  static ParabolaPlannerPtr_t createWithRoadmap(
      const core::Problem& problem, const core::RoadmapPtr_t& roadmap);
  /// Return shared pointer to new object.
  static ParabolaPlannerPtr_t create(const core::Problem& problem);
  /// One step of extension.
  virtual void oneStep();

  // for debugging
  // virtual core::PathVectorPtr_t solve ();

  virtual void startSolve();

  // disabled during testing
  virtual void tryDirectPath();

  // This method call SteeringMethodParabola, but we don't try to connect two
  // confuration, instead we shoot a random alpha0 and V0 valide for the
  // initiale configuration and then compute the final point. Then we check for
  // collision (for the trunk)  and we check if the final point is in a valide
  // configuration (trunk not in collision but limbs in accessible contact
  // zone). (Not anymore ) If this is true we do a reverse collision check until
  // we find the first valide configuration, then we check for the friction cone
  // and impact velocity constraint.(Not anymore : can't find normal after this)

  /**
   * @brief computeRandomParabola
   * @param x_start initial node
   * @param q_target target configuration (we shoot in this direction)
   * @param delayedEdges delayedEdges, node and edges waiting to be added in
   * roadmap at each iteration
   * @return the path computed (valid or not)
   */
  core::PathPtr_t computeRandomParabola(core::NodePtr_t x_start,
                                        core::ConfigurationPtr_t q_target,
                                        DelayedEdges_t& delayedEdges);

  /// Set configuration shooter.
  void configurationShooter(const core::ConfigurationShooterPtr_t& shooter);

  // we need both method, because smart_pointer inheritance is not implemented
  // (compiler don't know that rbprmRoadmapPtr_t derive from RoadmapPtr_t).
  virtual const core::RoadmapPtr_t& roadmap() const { return roadmap_; }

  const core::RbprmRoadmapPtr_t& rbprmRoadmap() const { return rbRoadmap_; }

 protected:
  /// Constructor
  ParabolaPlanner(const core::Problem& problem,
                  const core::RoadmapPtr_t& roadmap);
  /// Constructor with roadmap
  ParabolaPlanner(const core::Problem& problem);
  /// Store weak pointer to itself
  void init(const ParabolaPlannerWkPtr_t& weak);
  /// Extend a node in the direction of a configuration
  /// \param near node in the roadmap,
  /// \param target target configuration
  virtual core::PathPtr_t extend(const core::NodePtr_t& near,
                                 const core::ConfigurationPtr_t& target);
  virtual core::PathPtr_t extendParabola(
      const core::NodePtr_t& near, const core::ConfigurationPtr_t& target);

 private:
  /**
   * @brief computeGIWC compute the GIWC for the node configuration and fill the
   * node attribut
   * @param x the node
   * @param report the RBPRM report corresponding to the node's configuration
   */
  void computeGIWC(const core::RbprmNodePtr_t x,
                   core::ValidationReportPtr_t report);

  /**
   * @brief computeGIWC compute the GIWC for the node configuration and fill the
   * node attribut, get validation report and call the second method
   * @param x the node
   */
  void computeGIWC(const core::RbprmNodePtr_t x);

  void computeGIWC(const core::NodePtr_t x) {
    computeGIWC(static_cast<core::RbprmNodePtr_t>(x));
  }

  void computeGIWC(const core::NodePtr_t node,
                   core::ValidationReportPtr_t report) {
    computeGIWC(static_cast<core::RbprmNodePtr_t>(node), report);
  }

  core::ConfigurationShooterPtr_t configurationShooter_;
  mutable core::Configuration_t qProj_;
  ParabolaPlannerWkPtr_t weakPtr_;
  SteeringMethodParabolaPtr_t smParabola_;
  const core::RbprmRoadmapPtr_t rbRoadmap_;
  const core::RoadmapPtr_t roadmap_;
};
/// \}
}  // namespace rbprm
}  // namespace hpp
#endif  // HPP_CORE_DIFFUSING_PLANNER_HH
