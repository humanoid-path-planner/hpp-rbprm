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
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/rbprm/planner/dynamic-planner.hh>
#include <hpp/rbprm/planner/steering-dynamic.hh>
/**
 *  Motion planning algorithm with dynamic : planning in state space and not in configuration space
 *
 *  each node is a state
 *
 *  the velocity of the randomly sampled state are not constrained : they only depend of the selected x_near velocity and the control applied by the steering method
 */

namespace hpp{
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
      PathPlanner (problem),configurationShooter_ (problem.configurationShooter()),
      qProj_ (problem.robot ()->configSize ())/*,sm_(SteeringDynamic::create(problem))*/
    {
    }

    DynamicPlanner::DynamicPlanner (const core::Problem& problem,
                                        const core::RoadmapPtr_t& roadmap) :
      PathPlanner (problem, roadmap),configurationShooter_ (problem.configurationShooter()),
      qProj_ (problem.robot ()->configSize ())/*,sm_(SteeringDynamic::create(problem))*/
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

    core::PathPtr_t DynamicPlanner::extend (const core::NodePtr_t& near,
                                        const core::ConfigurationPtr_t& target)
    {
      const core::SteeringMethodPtr_t& sm_ (problem ().steeringMethod ());
      const core::ConstraintSetPtr_t& constraints (sm_->constraints ());
      if (constraints) {
        core::ConfigProjectorPtr_t configProjector (constraints->configProjector ());
        if (configProjector) {
          configProjector->projectOnKernel (*(near->configuration ()), *target,
                                            qProj_);
        } else {
          qProj_ = *target;
        }
        if (constraints->apply (qProj_)) {
          return (*sm_) (*(near->configuration ()), qProj_);
        } else {
          return core::PathPtr_t ();
        }
      }
      return (*sm_) (*(near->configuration ()), *target);
    }

    void DynamicPlanner::startSolve (){
      PathPlanner::startSolve();  // inherited method
      // done in python now, otherwise we can't specify initial or goal state with non null velocity
     // problem().robot()->setDimensionExtraConfigSpace(problem().robot()->numberDof());  // add extra config space for velocity

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
      typedef boost::tuple <core::NodePtr_t, core::ConfigurationPtr_t, core::PathPtr_t>
        DelayedEdge_t;
      typedef std::vector <DelayedEdge_t> DelayedEdges_t;
      DelayedEdges_t delayedEdges;
      core::DevicePtr_t robot (problem ().robot ());
      core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
      core::Nodes_t newNodes;
      core::PathPtr_t validPath, path;
      // Pick a random node
      core::ConfigurationPtr_t q_rand = configurationShooter_->shoot ();
      //
      // First extend each connected component toward q_rand
      //
      for (core::ConnectedComponents_t::const_iterator itcc =
             roadmap ()->connectedComponents ().begin ();
           itcc != roadmap ()->connectedComponents ().end (); ++itcc) {
        // Find nearest node in roadmap
        core::value_type distance;
        core::NodePtr_t near = roadmap ()->nearestNode (q_rand, *itcc, distance);
        path = extend (near, q_rand);
        if (path) {
          bool pathValid = pathValidation->validate (path, false, validPath);
          // Insert new path to q_near in roadmap
          core::value_type t_final = validPath->timeRange ().second;
          if (t_final != path->timeRange ().first) {
            core::ConfigurationPtr_t q_new (new core::Configuration_t
                                      ((*validPath) (t_final)));
            if (!pathValid || !belongs (q_new, newNodes)) {
              newNodes.push_back (roadmap ()->addNodeAndEdges
                                  (near, q_new, validPath));
            } else {
              // Store edges to add for later insertion.
              // Adding edges while looping on connected components is indeed
              // not recommended.
              delayedEdges.push_back (DelayedEdge_t (near, q_new, validPath));
            }
          }
        }
      }
      // Insert delayed edges
      for (DelayedEdges_t::const_iterator itEdge = delayedEdges.begin ();
           itEdge != delayedEdges.end (); ++itEdge) {
        const core::NodePtr_t& near = itEdge-> get <0> ();
        const core::ConfigurationPtr_t& q_new = itEdge-> get <1> ();
        const core::PathPtr_t& validPath = itEdge-> get <2> ();
       core:: NodePtr_t newNode = roadmap ()->addNode (q_new);
        roadmap ()->addEdge (near, newNode, validPath);
        core::interval_t timeRange = validPath->timeRange ();
        roadmap ()->addEdge (newNode, near, validPath->extract
                             (core::interval_t (timeRange.second ,
                                          timeRange.first)));
      }

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
          path = (*sm_) (*q1, *q2);
          if (path && pathValidation->validate (path, false, validPath)) {
            roadmap ()->addEdge (*itn1, *itn2, path);
            core::interval_t timeRange = path->timeRange ();
            roadmap ()->addEdge (*itn2, *itn1, path->extract
                                 (core::interval_t (timeRange.second,
                                              timeRange.first)));
          }
        }
      }
    }

    void DynamicPlanner::configurationShooter
    (const core::ConfigurationShooterPtr_t& shooter)
    {
      configurationShooter_ = shooter;
    }



  }//namespace dyn
}//namespace hpp
