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
#include <hpp/rbprm/planner/dynamic-planner.hh>
#include <boost/tuple/tuple.hpp>
#include <hpp/util/debug.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/node.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/steering-method-straight.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/rbprm/planner/steering-method-parabola.hh>
#include <hpp/rbprm/rbprm-path-validation.hh>
namespace hpp {
  namespace rbprm {
    using model::displayConfig;

    DynamicPlannerPtr_t DynamicPlanner::createWithRoadmap
    (const core::Problem& problem, const core::RoadmapPtr_t& roadmap)
    {
      DynamicPlanner* ptr = new DynamicPlanner (problem, roadmap);
      return DynamicPlannerPtr_t (ptr);
    }

    DynamicPlannerPtr_t DynamicPlanner::create (const core::Problem& problem)
    {
      DynamicPlanner* ptr = new DynamicPlanner (problem);
      return DynamicPlannerPtr_t (ptr);
    }

    DynamicPlanner::DynamicPlanner (const core::Problem& problem):
      PathPlanner (problem),
      configurationShooter_ (problem.configurationShooter()),
      qProj_ (problem.robot ()->configSize ()),
      smParabola_(rbprm::SteeringMethodParabola::create((core::ProblemPtr_t(&problem))))
    {
    }

    DynamicPlanner::DynamicPlanner (const core::Problem& problem,
                                        const core::RoadmapPtr_t& roadmap) :
      PathPlanner (problem, roadmap),
      configurationShooter_ (problem.configurationShooter()),
      qProj_ (problem.robot ()->configSize ()),
      smParabola_(rbprm::SteeringMethodParabola::create((core::ProblemPtr_t(&problem))))
    {
    }

    void DynamicPlanner::init (const DynamicPlannerWkPtr_t& weak)
    {
      PathPlanner::init (weak);
      weakPtr_ = weak;
    }

    bool belongs (const core::ConfigurationPtr_t& q, const core::Nodes_t& nodes)
    {
      for (core::Nodes_t::const_iterator itNode = nodes.begin ();
           itNode != nodes.end (); ++itNode) {
        if (*((*itNode)->configuration ()) == *q) return true;
      }
      return false;
    }

    void DynamicPlanner::startSolve ()
    {      
      // add 3 extraDof to save contact normal (used for parabola computation)
      //hppDout(notice,"set extra conf");

      //problem().robot()->setDimensionExtraConfigSpace(problem().robot()->extraConfigSpace().dimension() + 3);
    //  PathPlanner::startSolve();
      hppDout(notice,"startsolve");
      problem().checkProblem ();
      // Tag init and goal configurations in the roadmap
      roadmap()->resetGoalNodes ();
      roadmap()->initNode (problem().initConfig ());
      const core::Configurations_t goals (problem().goalConfigs ());
      for (core::Configurations_t::const_iterator itGoal = goals.begin ();
           itGoal != goals.end (); ++itGoal) {
        roadmap()->addGoalNode (*itGoal);
      }
      hppDout(notice,"startSolve OK");
    }

    core::PathPtr_t DynamicPlanner::extend (const core::NodePtr_t& near,
                                        const core::ConfigurationPtr_t& target)
    {
      const core::SteeringMethodPtr_t& sm (problem ().steeringMethod ());
      const core::ConstraintSetPtr_t& constraints (sm->constraints ());
      core::PathPtr_t path;
      if (constraints) {
        core::ConfigProjectorPtr_t configProjector (constraints->configProjector ());
        if (configProjector) {
          configProjector->projectOnKernel (*(near->configuration ()), *target,
                                            qProj_);
        } else {
          qProj_ = *target;
        }
        if (constraints->apply (qProj_)) {
          path = (*sm) (*(near->configuration ()), qProj_);
        } else {
          return core::PathPtr_t ();
        }
      }else{
        path = (*sm) (*(near->configuration ()), *target);
      }
      return path;
    }

    core::PathPtr_t DynamicPlanner::extendParabola (const core::NodePtr_t& near,
                                        const core::ConfigurationPtr_t& target)
    {
      const core::SteeringMethodPtr_t& sm (problem ().steeringMethod ());
      const core::ConstraintSetPtr_t& constraints (sm->constraints ());
      core::PathPtr_t path;
      if (constraints) {
        core::ConfigProjectorPtr_t configProjector (constraints->configProjector ());
        if (configProjector) {
          configProjector->projectOnKernel (*(near->configuration ()), *target,
                                            qProj_);
        } else {
          qProj_ = *target;
        }
        if (constraints->apply (qProj_)) {
          path = (*smParabola_) (*(near->configuration ()), qProj_);
        } else {
          return core::PathPtr_t ();
        }
      }else{
        path = (*smParabola_) (*(near->configuration ()), *target);
      }
      return path;
    }


    /// This method performs one step of RRT extension as follows
    ///  1. a random configuration "q_rand" is shot,
    ///  2. for each connected component,
    ///    2.1. the closest node "q_near" is chosen,
    ///    2.2. "q_rand" is projected first on the tangent space of the
    ///         non-linear constraint at "q_near", this projection yields
    ///         "q_tmp", then "q_tmp" is projected on the non-linear constraint
    ///         manifold as "q_proj" (method extend)
    ///    2.3. the steering method is called between "q_near" and "q_proj" that
    ///         returns "path",
    ///    2.4. a valid connected part of "path", called "validPath" starting at
    ///         "q_near" is extracted, if "path" is valid (collision free),
    ///         the full "path" is returned, "q_new" is the end configuration of
    ///         "validPath",
    ///    2.5  a new node containing "q_new" is added to the connected
    ///         component and a new edge is added between nodes containing
    ///         "q_near" and "q_new".
    ///  3. Try to connect new nodes together using the steering method and
    ///     the current PathValidation instance.
    ///
    ///  Note that edges are actually added to the roadmap after step 2 in order
    ///  to avoid iterating on the list of connected components while modifying
    ///  this list.

    void DynamicPlanner::oneStep ()
    {
      hppDout(notice,"oneStep begin");
      typedef boost::tuple <core::NodePtr_t, core::ConfigurationPtr_t, core::PathPtr_t>
        DelayedEdge_t;
      typedef std::vector <DelayedEdge_t> DelayedEdges_t;
      DelayedEdges_t delayedEdges;
      core::DevicePtr_t robot (problem ().robot ());
      core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
      RbPrmPathValidationPtr_t rbprmPathValidation = boost::dynamic_pointer_cast<RbPrmPathValidation>(pathValidation);
      core::SteeringMethodPtr_t sm = problem().steeringMethod();
      core::Nodes_t newNodes;
      std::vector<std::string> filter;
      core::PathPtr_t validPath, path;
      hppDout(notice,"random shoot begin");
      // Pick a random node
      core::ConfigurationPtr_t q_rand = configurationShooter_->shoot ();
      hppDout(notice,"random shoot OK");

      //
      // First extend each connected component toward q_rand
      //
      int i = 1;
      for (core::ConnectedComponents_t::const_iterator itcc =
             roadmap ()->connectedComponents ().begin ();
           itcc != roadmap ()->connectedComponents ().end (); ++itcc) {
        hppDout(notice, "###### for connected components "<<i);
        i++;
        // Find nearest node in roadmap
        core::value_type distance;
        core::NodePtr_t near = roadmap ()->nearestNode (q_rand, *itcc, distance);
        path = extend (near, q_rand);
        if (path) {
          hppDout(notice, "### path exist");
          core::PathValidationReportPtr_t report;
          bool pathValid = pathValidation->validate (path, false, validPath,report);

          // Insert new path to q_near in roadmap
          core::value_type t_final = validPath->timeRange ().second;
          if (t_final != path->timeRange ().first) {

            hppDout(notice, "### path's length not null");
            core::ConfigurationPtr_t q_new (new core::Configuration_t(validPath->end ()));
            if (!pathValid || !belongs (q_new, newNodes)) {
              hppDout(notice, "### add new node and edges: ");
              hppDout(notice, displayConfig(*q_new));
              core::NodePtr_t x_new = roadmap ()->addNodeAndEdges(near, q_new, validPath);
              newNodes.push_back (x_new);
              if(!pathValid){
                hppDout(notice,"### Straight path not fully valid, try parabola path between qnew and qrand");
                path = extendParabola(x_new, q_rand);
                if (path) {
                  hppDout(notice,"### Parabola path exist");
                  // call validate without constraint on limbs
                  bool paraPathValid = rbprmPathValidation->validate (path, false, validPath, report, filter);
                  if (paraPathValid) { // only add if the full path is valid, otherwise it's the same as the straight line (because we can't extract a subpath of a parabola path)
                    hppDout(notice, "#### parabola path valid !");
                    core::ConfigurationPtr_t q_last (new core::Configuration_t(validPath->end ()));
                    delayedEdges.push_back (DelayedEdge_t (x_new, q_last, validPath));

                }
              }
            }
            } else {
              hppDout(notice, "### add delayed edge");
              // Store edges to add for later insertion.
              // Adding edges while looping on connected components is indeed
              // not recommended.
              delayedEdges.push_back (DelayedEdge_t (near, q_new, validPath));
            }
          }
        }else
          hppDout(notice, "### path dosen't exist");

      }
      hppDout(notice,"extend OK");
      // Insert delayed edges
      for (DelayedEdges_t::const_iterator itEdge = delayedEdges.begin ();
           itEdge != delayedEdges.end (); ++itEdge) {
        const core::NodePtr_t& near = itEdge-> get <0> ();
        const core::ConfigurationPtr_t& q_new = itEdge-> get <1> ();
        const core::PathPtr_t& validPath = itEdge-> get <2> ();
        core::NodePtr_t newNode = roadmap ()->addNode (q_new);
        roadmap ()->addEdge (near, newNode, validPath);
        roadmap ()->addEdge (newNode, near, validPath->reverse());
      }
      hppDout(notice,"add delayed edge OK");

      //
      // Second, try to connect new nodes together
      //
      for (core::Nodes_t::const_iterator itn1 = newNodes.begin ();
           itn1 != newNodes.end (); ++itn1) {
        for (core::Nodes_t::const_iterator itn2 = boost::next (itn1);
             itn2 != newNodes.end (); ++itn2) {
          core::ConfigurationPtr_t q1 ((*itn1)->configuration ());
          core::ConfigurationPtr_t q2 ((*itn2)->configuration ());
          assert (*q1 != *q2);
          path = (*sm) (*q1, *q2);
          core::PathValidationReportPtr_t report;
        /*  if(!(path && pathValidation->validate (path, false, validPath, report))){
            hppDout(notice, "## parabola path fail, compute straight path");
            path = (*sm) (*q1, *q2);
          }*/
          if (path && pathValidation->validate (path, false, validPath, report)) {
            roadmap ()->addEdge (*itn1, *itn2, path);
            roadmap ()->addEdge (*itn2, *itn1, path->reverse());
          }
        }
      }
      hppDout(notice,"connect new nodes OK");

    }

    void DynamicPlanner::tryDirectPath ()
    {
      // call steering method here to build a direct conexion
      core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
      core::PathPtr_t validPath,  path;
      std::vector<std::string> filter;
      RbPrmPathValidationPtr_t rbprmPathValidation = boost::dynamic_pointer_cast<RbPrmPathValidation>(pathValidation);
      core::NodePtr_t initNode = roadmap ()->initNode();
      for (core::Nodes_t::const_iterator itn = roadmap ()->goalNodes ().begin();itn != roadmap ()->goalNodes ().end (); ++itn) {
        core::ConfigurationPtr_t q1 ((initNode)->configuration ());
        core::ConfigurationPtr_t q2 ((*itn)->configuration ());
        assert (*q1 != *q2);
        path = extend(initNode,q2);
        if (path) {
          core::PathValidationReportPtr_t report;
          bool pathValid = pathValidation->validate (path, false, validPath,report);
          if (pathValid ) {
            roadmap ()->addEdge (initNode, *itn, path);
            roadmap ()->addEdge (*itn, initNode, path->reverse());
          }else if(validPath->timeRange ().second != path->timeRange ().first){
            core::ConfigurationPtr_t q_new(new core::Configuration_t(validPath->end()));
            core::NodePtr_t x_new = roadmap()->addNodeAndEdges(initNode,q_new,validPath);

            hppDout(notice,"### Straight path not fully valid, try parabola path between qnew and qGoal");
            path = extendParabola(x_new, q2);
            if (path) {
              hppDout(notice,"### Parabola path exist");
              // call validate without constraint on limbs
              bool paraPathValid = rbprmPathValidation->validate (path, false, validPath, report, filter);
              if (paraPathValid) { // only add if the full path is valid, otherwise it's the same as the straight line (because we can't extract a subpath of a parabola path)
                hppDout(notice, "#### parabola path valid !");
                roadmap()->addEdge(x_new,*itn,validPath);
              }
            }// if path
          } //else if path lenght not null
        } //if path exist
      } //for qgoals
    }

    void DynamicPlanner::configurationShooter
    (const core::ConfigurationShooterPtr_t& shooter)
    {
      configurationShooter_ = shooter;
    }


  } // namespace core
} // namespace hpp
