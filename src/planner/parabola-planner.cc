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

#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/intersect.h>
#include <polytope/stability_margin.h>

#include <hpp/core/config-projector.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/configuration-shooter/uniform.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/node.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/path.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/rbprm/planner/parabola-path.hh>
#include <hpp/rbprm/planner/parabola-planner.hh>
#include <hpp/rbprm/planner/steering-method-parabola.hh>
#include <hpp/rbprm/rbprm-path-validation.hh>
#include <hpp/rbprm/rbprm-validation-report.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/timer.hh>

#include "hpp/rbprm/utils/algorithms.h"

namespace hpp {
namespace rbprm {
using model::displayConfig;

ParabolaPlannerPtr_t ParabolaPlanner::createWithRoadmap(
    const core::Problem& problem, const core::RoadmapPtr_t& roadmap) {
  ParabolaPlanner* ptr = new ParabolaPlanner(problem, roadmap);
  return ParabolaPlannerPtr_t(ptr);
}

ParabolaPlannerPtr_t ParabolaPlanner::create(const core::Problem& problem) {
  ParabolaPlanner* ptr = new ParabolaPlanner(problem);
  return ParabolaPlannerPtr_t(ptr);
}

ParabolaPlanner::ParabolaPlanner(const core::Problem& problem)
    : PathPlanner(problem),
      configurationShooter_(problem.configurationShooter()),
      qProj_(problem.robot()->configSize()),
      smParabola_(rbprm::SteeringMethodParabola::create(
          (core::ProblemPtr_t(&problem)))),
      rbRoadmap_(
          core::RbprmRoadmap::create(problem.distance(), problem.robot())),
      roadmap_(boost::dynamic_pointer_cast<core::Roadmap>(rbRoadmap_)) {}

ParabolaPlanner::ParabolaPlanner(const core::Problem& problem,
                                 const core::RoadmapPtr_t& roadmap)
    : PathPlanner(problem, roadmap),
      configurationShooter_(problem.configurationShooter()),
      qProj_(problem.robot()->configSize()),
      smParabola_(rbprm::SteeringMethodParabola::create(
          (core::ProblemPtr_t(&problem)))),
      rbRoadmap_(
          core::RbprmRoadmap::create(problem.distance(), problem.robot())),
      roadmap_(boost::dynamic_pointer_cast<core::Roadmap>(rbRoadmap_)) {}

void ParabolaPlanner::init(const ParabolaPlannerWkPtr_t& weak) {
  PathPlanner::init(weak);
  weakPtr_ = weak;
}

bool belongs(const core::ConfigurationPtr_t& q, const core::Nodes_t& nodes) {
  for (core::Nodes_t::const_iterator itNode = nodes.begin();
       itNode != nodes.end(); ++itNode) {
    if (*((*itNode)->configuration()) == *q) return true;
  }
  return false;
}

void ParabolaPlanner::startSolve() {
  // add 3 extraDof to save contact normal (used for parabola computation)
  // hppDout(notice,"set extra conf");

  // problem().robot()->setDimensionExtraConfigSpace(problem().robot()->extraConfigSpace().dimension()
  // + 3);
  //  PathPlanner::startSolve();
  hppDout(notice, "startsolve");
  problem().checkProblem();
  // Tag init and goal configurations in the roadmap
  roadmap()->resetGoalNodes();

  roadmap()->initNode(problem().initConfig());
  const core::Configurations_t goals(problem().goalConfigs());
  for (core::Configurations_t::const_iterator itGoal = goals.begin();
       itGoal != goals.end(); ++itGoal) {
    roadmap()->addGoalNode(*itGoal);
  }
  hppDout(notice, "startSolve OK");

  polytope::init_library();
}

core::PathPtr_t ParabolaPlanner::extend(
    const core::NodePtr_t& near, const core::ConfigurationPtr_t& target) {
  const core::SteeringMethodPtr_t& sm(problem().steeringMethod());
  const core::ConstraintSetPtr_t& constraints(sm->constraints());
  core::PathPtr_t path;
  if (constraints) {
    core::ConfigProjectorPtr_t configProjector(constraints->configProjector());
    if (configProjector) {
      configProjector->projectOnKernel(*(near->configuration()), *target,
                                       qProj_);
    } else {
      qProj_ = *target;
    }
    if (constraints->apply(qProj_)) {
      path = (*sm)(*(near->configuration()), qProj_);
    } else {
      return core::PathPtr_t();
    }
  } else {
    path = (*sm)(*(near->configuration()), *target);
  }
  return path;
}

core::PathPtr_t ParabolaPlanner::extendParabola(
    const core::NodePtr_t& near, const core::ConfigurationPtr_t& target) {
  const core::SteeringMethodPtr_t& sm(problem().steeringMethod());
  const core::ConstraintSetPtr_t& constraints(sm->constraints());
  core::PathPtr_t path;
  if (constraints) {
    core::ConfigProjectorPtr_t configProjector(constraints->configProjector());
    if (configProjector) {
      configProjector->projectOnKernel(*(near->configuration()), *target,
                                       qProj_);
    } else {
      qProj_ = *target;
    }
    if (constraints->apply(qProj_)) {
      path = (*smParabola_)(*(near->configuration()), qProj_);
    } else {
      return core::PathPtr_t();
    }
  } else {
    path = (*smParabola_)(*(near->configuration()), *target);
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

void ParabolaPlanner::oneStep() {
  hppDout(notice, "# oneStep BEGIN");
  DelayedEdges_t delayedEdges;
  core::DevicePtr_t robot(problem().robot());
  core::PathValidationPtr_t pathValidation(problem().pathValidation());
  RbPrmPathValidationPtr_t rbprmPathValidation =
      boost::dynamic_pointer_cast<RbPrmPathValidation>(pathValidation);
  core::SteeringMethodPtr_t sm = problem().steeringMethod();
  core::Nodes_t newNodes;
  std::vector<std::string> filter;
  core::PathPtr_t validPath, path;
  // Pick a random node
  hppDout(notice, "# random shoot begin");
  core::ConfigurationPtr_t q_rand = configurationShooter_->shoot();
  hppDout(notice, "# random shoot OK : ");
  hppDout(notice, displayConfig(*q_rand));

  //
  // First extend each connected component toward q_rand
  //
  int i = 1;
  for (core::ConnectedComponents_t::const_iterator itcc =
           roadmap()->connectedComponents().begin();
       itcc != roadmap()->connectedComponents().end(); ++itcc) {
    hppDout(notice, "###### for connected components " << i);
    i++;
    // Find nearest node in roadmap
    core::value_type distance;
    core::NodePtr_t near = roadmap()->nearestNode(q_rand, *itcc, distance);

    hppStartBenchmark(EXTEND);
    path = extend(near, q_rand);
    hppStopBenchmark(EXTEND);
    hppDisplayBenchmark(EXTEND);
    if (path) {
      hppDout(notice, "### path exist");
      core::PathValidationReportPtr_t report;
      bool pathValid = pathValidation->validate(path, false, validPath, report);

      // Insert new path to q_near in roadmap
      core::value_type t_final = validPath->timeRange().second;
      if (t_final != path->timeRange().first) {
        hppDout(notice, "### path's length not null");
        core::ConfigurationPtr_t q_new(
            new core::Configuration_t(validPath->end()));
        if (!pathValid || !belongs(q_new, newNodes)) {
          hppDout(notice, "### add new node and edges: ");
          hppDout(notice, displayConfig(*q_new));
          core::NodePtr_t x_new =
              rbprmRoadmap()->addNodeAndEdges(near, q_new, validPath);
          computeGIWC(x_new);

          newNodes.push_back(x_new);
          if (!pathValid) {
            hppDout(notice,
                    "### Straight path not fully valid, try parabola path "
                    "between qnew and qrand");
            // get contact normal and update the extraDof
            // TODO : aaprès modif validation report, recup normal des ROM et
            // non du tronc
            /* if(report->configurationReport)  {
              fcl::Vec3f normal ( -
            boost::dynamic_pointer_cast<core::CollisionValidationReport>(report->configurationReport)->result.getContact(0).normal);

              hppDout(notice,"normal = "<<normal);
              // fill extraDof with normal :
              core::size_type size = problem().robot()->configSize ();
              (*q_new)[size-3]=normal[0];
              (*q_new)[size-2]=normal[1];
              (*q_new)[size-1]=normal[2];
            }*/
            hppStartBenchmark(EXTENDPARA);
            path = extendParabola(x_new, q_rand);
            hppStopBenchmark(EXTENDPARA);
            hppDisplayBenchmark(EXTENDPARA);
            if (path) {
              hppDout(notice, "### Parabola path exist");
              // call validate without constraint on limbs
              bool paraPathValid = rbprmPathValidation->validate(
                  path, false, validPath, report, filter);
              if (paraPathValid) {  // only add if the full path is valid,
                                    // otherwise it's the same as the straight
                                    // line (because we can't extract a subpath
                                    // of a parabola path)
                hppDout(notice, "#### parabola path valid !");
                core::ConfigurationPtr_t q_last(
                    new core::Configuration_t(validPath->end()));
                delayedEdges.push_back(DelayedEdge_t(x_new, q_last, validPath));
              } else {
                hppDout(notice,
                        "#### Direct parabola path not valid, compute random "
                        "parabola :");
                computeRandomParabola(x_new, q_rand, delayedEdges);
              }
            } else {
              hppDout(
                  notice,
                  "#### No direct parabola path, compute random parabola :");
              computeRandomParabola(x_new, q_rand, delayedEdges);
            }
          } else {
            hppDout(notice, "### straight path fully valid");
          }
        } else {
          hppDout(notice, "### add delayed edge");
          // Store edges to add for later insertion.
          // Adding edges while looping on connected components is indeed
          // not recommended.
          delayedEdges.push_back(DelayedEdge_t(near, q_new, validPath));
        }
      } else {
        hppDout(notice, "### path lenght null");
      }
    } else
      hppDout(notice, "### path dosen't exist");
  }
  hppDout(notice, "# extend OK");
  // Insert delayed edges
  for (DelayedEdges_t::const_iterator itEdge = delayedEdges.begin();
       itEdge != delayedEdges.end(); ++itEdge) {
    const core::NodePtr_t& near = itEdge->get<0>();
    const core::ConfigurationPtr_t& q_new = itEdge->get<1>();
    const core::PathPtr_t& validPath = itEdge->get<2>();
    core::NodePtr_t newNode = rbprmRoadmap()->addNode(q_new);
    core::RbprmNodePtr_t newNodeCast =
        static_cast<core::RbprmNodePtr_t>(newNode);
    computeGIWC(newNodeCast);
    ParabolaPathPtr_t parabolaPath =
        boost::dynamic_pointer_cast<ParabolaPath>(validPath);
    if (parabolaPath) {
      newNodeCast->setAlphas(parabolaPath->alpha_, parabolaPath->alphaMin_,
                             parabolaPath->alphaMax_);
      newNodeCast->Z(parabolaPath->Z_);
      newNodeCast->xTheta(parabolaPath->Xtheta_);
    }
    roadmap()->addEdge(near, newNode, validPath);
    roadmap()->addEdge(newNode, near, validPath->reverse());
  }
  hppDout(notice, "# add delayed edge OK");

  //
  // Second, try to connect new nodes together
  //
  for (core::Nodes_t::const_iterator itn1 = newNodes.begin();
       itn1 != newNodes.end(); ++itn1) {
    for (core::Nodes_t::const_iterator itn2 = boost::next(itn1);
         itn2 != newNodes.end(); ++itn2) {
      core::ConfigurationPtr_t q1((*itn1)->configuration());
      core::ConfigurationPtr_t q2((*itn2)->configuration());
      assert(*q1 != *q2);
      path = (*sm)(*q1, *q2);
      core::PathValidationReportPtr_t report;
      /*  if(!(path && pathValidation->validate (path, false, validPath,
      report))){ hppDout(notice, "## parabola path fail, compute straight
      path"); path = (*sm) (*q1, *q2);
      }*/
      if (path && pathValidation->validate(path, false, validPath, report)) {
        roadmap()->addEdge(*itn1, *itn2, path);
        roadmap()->addEdge(*itn2, *itn1, path->reverse());
      }
    }
  }
  hppDout(notice, "# OneStep END");
}

// This method call SteeringMethodParabola, but we don't try to connect two
// confuration, instead we shoot a random alpha0 and V0 valide for the initiale
// configuration and then compute the final point. Then we check for collision
// (for the trunk)  and we check if the final point is in a valide configuration
// (trunk not in collision but limbs in accessible contact zone). (Not anymore )
// If this is true we do a reverse collision check until we find the first
// valide configuration, then we check for the friction cone and impact velocity
// constraint.(Not anymore : can't find normal after this)
core::PathPtr_t ParabolaPlanner::computeRandomParabola(
    core::NodePtr_t x_start, core::ConfigurationPtr_t q_target,
    DelayedEdges_t& delayedEdges) {
  hppDout(notice, "### compute random parabola :");
  std::vector<std::string> filter;
  core::PathPtr_t validPath, landingPath;
  core::PathValidationPtr_t pathValidation(problem().pathValidation());
  RbPrmPathValidationPtr_t rbprmPathValidation =
      boost::dynamic_pointer_cast<RbPrmPathValidation>(pathValidation);
  core::PathValidationReportPtr_t report;
  core::ConfigurationPtr_t q_start(x_start->configuration());
  value_type alpha0, v0;
  core::PathPtr_t path =
      smParabola_->compute_random_3D_path(*(q_start), *q_target, &alpha0, &v0);
  if (!path) {
    hppDout(notice, "No path returned");
    return path;
  }
  bool valid =
      rbprmPathValidation->validate(path, false, validPath, report, filter);
  if ((!valid) && validPath->timeRange().second > path->timeRange().first) {
    hppDout(notice, "Random parabola have collision");
    // check if obstacle are present in accessible zone of the limbs :
    core::ConfigurationPtr_t q_new(new core::Configuration_t(validPath->end()));
    bool contactValid =
        rbprmPathValidation->getValidator()->validateRoms(*q_new);
    if (contactValid) {
      hppDout(notice, "Random parabola land in obstacle");
      fcl::Vec3f normal(
          -boost::dynamic_pointer_cast<core::CollisionValidationReport>(
               report->configurationReport)
               ->result.getContact(0)
               .normal);
      hppDout(notice, "normal = " << normal);
      // fill extraDof with normal :
      core::size_type size = problem().robot()->configSize();
      (*q_new)[size - 3] = normal[0];
      (*q_new)[size - 2] = normal[1];
      (*q_new)[size - 1] = normal[2];
      // compute theta :
      value_type X = (*q_new)[0] - (*q_start)[0];
      value_type Y = (*q_new)[1] - (*q_start)[1];
      value_type Z = (*q_new)[2] - (*q_start)[2];
      const value_type theta = atan2(Y, X);
      const value_type X_theta = X * cos(theta) + Y * sin(theta);
      value_type delta;
      // test impact velocity
      const value_type x_theta_0_dot =
          sqrt((9.81 * X_theta * X_theta) / (2 * (X_theta * tan(alpha0) - Z)));
      const value_type inv_x_th_dot_0_sq = 1 / (x_theta_0_dot * x_theta_0_dot);
      const value_type Vimp =
          sqrt(1 + (-9.81 * X * inv_x_th_dot_0_sq + tan(alpha0)) *
                       (-9.81 * X * inv_x_th_dot_0_sq + tan(alpha0))) *
          x_theta_0_dot;  // x_theta_0_dot > 0
      hppDout(notice, "Vimp: " << Vimp);
      if (Vimp < 0 || Vimp > smParabola_->getVImpMax()) {
        hppDout(notice, "Impact velocity invalid : vImp = " << Vimp);
        return path;
      }

      hppDout(notice, "compute friction cone for landing point : ");
      bool coneOK = smParabola_->fiveth_constraint(*q_new, theta, 1, &delta);
      if (!coneOK) {
        hppDout(notice,
                "Failed to compute 2D cone (intersection with plan empty)");
        return path;
      }
      hppDout(notice, "delta = " << delta);
      const value_type n_angle =
          atan2(normal[2],
                cos(theta) * normal[0] + sin(theta) * normal[1]);  // normal
      value_type alpha_imp_min =
          n_angle - M_PI - delta;  // friction cone at landing point
      value_type alpha_imp_max = n_angle - M_PI + delta;
      if (n_angle < 0) {
        alpha_imp_min = n_angle + M_PI - delta;
        alpha_imp_max = n_angle + M_PI + delta;
      }
      hppDout(notice, "n_angle = " << n_angle);
      hppDout(notice, "alpha_f_min = " << alpha_imp_min);
      hppDout(notice, "alpha_f_max = " << alpha_imp_max);
      value_type alpha_imp_inf;  // bound on alpha0 wrt to landing friction cone
                                 // constraint
      value_type alpha_imp_sup;
      bool fail3 = smParabola_->third_constraint(0, X_theta, Z, alpha_imp_min,
                                                 alpha_imp_max, &alpha_imp_sup,
                                                 &alpha_imp_inf, n_angle);
      if (fail3) {
        hppDout(notice, "failed to apply 3rd constraint");
        return path;
      }
      hppDout(notice, "alpha_imp_sup: " << alpha_imp_sup);
      hppDout(notice, "alpha_imp_inf: " << alpha_imp_inf);

      value_type alpha_inf_bound = 0;
      value_type alpha_sup_bound = 0;
      value_type alpha_inf4;
      alpha_inf4 = atan(Z / X_theta);
      /* Define alpha_0 interval satisfying constraints (cf steeringMethod
       * Mylène) */
      if (n_angle > 0) {
        alpha_inf_bound = std::max(alpha_imp_inf, alpha_inf4);
        if (alpha_imp_min < -M_PI / 2) {
          alpha_sup_bound = M_PI / 2;
        } else {  // alpha_imp_sup is worth
          alpha_sup_bound = std::min(M_PI / 2, alpha_imp_sup);
        }
      } else {  // down-oriented cone
        if (alpha_imp_max < M_PI / 2) {
          alpha_inf_bound = std::max(alpha_imp_inf, alpha_inf4);
        } else {  // alpha_imp_max >= M_PI/2 so alpha_imp_inf inaccurate
          alpha_inf_bound = alpha_inf4;
        }
        alpha_sup_bound = std::min(M_PI / 2, alpha_imp_sup);
      }

      hppDout(notice, "alpha_inf_bound: " << alpha_inf_bound);
      hppDout(notice, "alpha_sup_bound: " << alpha_sup_bound);

      if (alpha0 > alpha_sup_bound || alpha0 < alpha_inf_bound) {
        hppDout(notice, "Parabola doesn't land inside friction cone");
        return path;
      }
      hppDout(notice, "# Landing point valid, add node and edges");
      // Here everything is valid, adding node and edge to roadmap :
      delayedEdges.push_back(DelayedEdge_t(x_start, q_new, validPath));

    }  // contactValid
  } else if (valid) {
    hppDout(notice, "random parabola doesn't land on obstacle");
    return path;
  }
  return path;
}

void ParabolaPlanner::tryDirectPath() {
  // call steering method here to build a direct conexion
  core::PathValidationPtr_t pathValidation(problem().pathValidation());
  core::PathPtr_t validPath, path;
  std::vector<std::string> filter;
  RbPrmPathValidationPtr_t rbprmPathValidation =
      boost::dynamic_pointer_cast<RbPrmPathValidation>(pathValidation);
  core::NodePtr_t initNode = roadmap()->initNode();
  for (core::Nodes_t::const_iterator itn = roadmap()->goalNodes().begin();
       itn != roadmap()->goalNodes().end(); ++itn) {
    core::ConfigurationPtr_t q1((initNode)->configuration());
    core::ConfigurationPtr_t q2((*itn)->configuration());
    assert(*q1 != *q2);
    hppStartBenchmark(EXTEND);
    path = extend(initNode, q2);
    hppStopBenchmark(EXTEND);
    hppDisplayBenchmark(EXTEND);
    if (path) {
      core::PathValidationReportPtr_t report;
      bool pathValid = pathValidation->validate(path, false, validPath, report);
      if (pathValid) {
        roadmap()->addEdge(initNode, *itn, path);
        roadmap()->addEdge(*itn, initNode, path->reverse());
      } else if (validPath->timeRange().second != path->timeRange().first) {
        core::ConfigurationPtr_t q_new(
            new core::Configuration_t(validPath->end()));
        core::NodePtr_t x_new =
            rbprmRoadmap()->addNodeAndEdges(initNode, q_new, validPath);
        computeGIWC(x_new);
        hppDout(notice,
                "### Straight path not fully valid, try parabola path between "
                "qnew and qGoal");
        hppStartBenchmark(EXTENDPARA);
        path = extendParabola(x_new, q2);
        hppStopBenchmark(EXTENDPARA);
        hppDisplayBenchmark(EXTENDPARA);
        if (path) {
          hppDout(notice, "### Parabola path exist");
          // call validate without constraint on limbs
          bool paraPathValid = rbprmPathValidation->validate(
              path, false, validPath, report, filter);
          if (paraPathValid) {  // only add if the full path is valid, otherwise
                                // it's the same as the straight line (because
                                // we can't extract a subpath of a parabola
                                // path)
            hppDout(notice, "#### parabola path valid !");
            core::ConfigurationPtr_t q_last(
                new core::Configuration_t(validPath->end()));
            core::NodePtr_t x_last = rbprmRoadmap()->addNode(q_last);
            computeGIWC(x_last);
            rbprmRoadmap()->addEdge(x_new, x_last, validPath);
            rbprmRoadmap()->addEdge(x_last, x_new, validPath->reverse());
          } else {
            hppDout(notice, "#### parabola path not valid !");
          }
        } else {
          hppDout(
              notice,
              "### Cannot compute parabola path, shoot random alpha0 and V0 :");
          DelayedEdges_t delayedEdges;
          computeRandomParabola(x_new, q2,
                                delayedEdges);  // TODO passer en reference
          hppDout(notice, "sizeDelayedEdge = " << delayedEdges.size());
          for (DelayedEdges_t::const_iterator itEdge = delayedEdges.begin();
               itEdge != delayedEdges.end(); ++itEdge) {
            const core::NodePtr_t& near = itEdge->get<0>();
            const core::ConfigurationPtr_t& q_new = itEdge->get<1>();
            const core::PathPtr_t& validPath = itEdge->get<2>();
            core::NodePtr_t newNode = rbprmRoadmap()->addNode(q_new);
            core::RbprmNodePtr_t newNodeCast =
                static_cast<core::RbprmNodePtr_t>(newNode);
            computeGIWC(newNodeCast);
            ParabolaPathPtr_t parabolaPath =
                boost::dynamic_pointer_cast<ParabolaPath>(validPath);
            if (parabolaPath) {
              newNodeCast->setAlphas(parabolaPath->alpha_,
                                     parabolaPath->alphaMin_,
                                     parabolaPath->alphaMax_);
              newNodeCast->Z(parabolaPath->Z_);
              newNodeCast->xTheta(parabolaPath->Xtheta_);
            }
            roadmap()->addEdge(near, newNode, validPath);
            roadmap()->addEdge(newNode, near, validPath->reverse());
          }
          hppDout(notice, "add delayed edge OK");
        }
      }  // else if path lenght not null
    }    // if path exist
  }      // for qgoals
}

void ParabolaPlanner::configurationShooter(
    const core::ConfigurationShooterPtr_t& shooter) {
  configurationShooter_ = shooter;
}

// for debugging
/* core::PathVectorPtr_t DynamicPlanner::solve (){
  startSolve();
  // call steering method here to build a direct conexion
  core::PathVectorPtr_t pathVector;
  core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
  core::PathPtr_t validPath,  path;
  std::vector<std::string> filter;
  RbPrmPathValidationPtr_t rbprmPathValidation =
boost::dynamic_pointer_cast<RbPrmPathValidation>(pathValidation);
  core::NodePtr_t initNode = roadmap ()->initNode();
  for (core::Nodes_t::const_iterator itn = roadmap ()->goalNodes ().begin();itn
!= roadmap ()->goalNodes ().end ();
++itn) { core::ConfigurationPtr_t q1 ((initNode)->configuration ());
core::ConfigurationPtr_t q2 ((*itn)->configuration
()); assert (*q1 != *q2); hppStartBenchmark(EXTEND); path = extend(initNode,q2);
hppStopBenchmark (EXTEND); hppDisplayBenchmark (EXTEND); if (path) {

      core::PathValidationReportPtr_t report;
      bool pathValid = pathValidation->validate (path, false, validPath,report);
      if (pathValid ) {
        roadmap ()->addEdge (initNode, *itn, path);
        roadmap ()->addEdge (*itn, initNode, path->reverse());
      }else if(validPath->timeRange ().second != path->timeRange ().first){
        core::ConfigurationPtr_t q_new(new
core::Configuration_t(validPath->end())); core::NodePtr_t x_new =
rbprmRoadmap()->addNodeAndEdges(initNode,q_new,validPath); pathVector =
core::PathVector::create (validPath->outputSize (),
validPath->outputDerivativeSize ()); pathVector->appendPath (validPath);
        hppDout(notice,"### Straight path not fully valid, try parabola path
between qnew and qGoal"); hppStartBenchmark(EXTENDPARA); path =
extendParabola(x_new, q2); hppStopBenchmark (EXTENDPARA); hppDisplayBenchmark
(EXTENDPARA); if (path) { hppDout(notice,"### Parabola path exist");
          // call validate without constraint on limbs
          bool paraPathValid = rbprmPathValidation->validate (path, false,
validPath, report, filter); if (paraPathValid) { // only add if the full path is
valid, otherwise it's the same as the straight line (because we can't extract a
subpath of a parabola path) hppDout(notice, "#### parabola path valid !");
            core::ConfigurationPtr_t q_last (new
core::Configuration_t(validPath->end ())); core::NodePtr_t x_last =
rbprmRoadmap()->addNode(q_last);
            rbprmRoadmap()->addEdge(x_new,x_last,validPath);
            pathVector->appendPath (validPath);
          }else {
            hppDout(notice, "#### parabola path not valid !");
          }
        }else{
          hppDout(notice,"### Cannot compute parabola path, shoot random alpha0
and V0 :"); DelayedEdges_t delayedEdges; core::PathPtr_t pathPara =
computeRandomParabola(x_new,q2,delayedEdges); if(pathPara)
            pathVector->appendPath(pathPara);
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
        }
      } //else if path lenght not null
    } //if path exist
  } //for qgoals
  return pathVector;
}*/

void ParabolaPlanner::computeGIWC(const core::RbprmNodePtr_t x) {
  core::ValidationReportPtr_t report;
  problem().configValidations()->validate(*(x->configuration()), report);
  computeGIWC(x, report);
}

void ParabolaPlanner::computeGIWC(const core::RbprmNodePtr_t node,
                                  core::ValidationReportPtr_t report) {
  hppDout(notice, "## compute GIWC");
  core::ConfigurationPtr_t q = node->configuration();
  // fil normal information in node
  if (node) {
    size_t cSize = problem().robot()->configSize();
    hppDout(info, "~~ NODE cast correctly");
    node->normal((*q)[cSize - 3], (*q)[cSize - 2], (*q)[cSize - 1]);
    hppDout(info, "~~ normal = " << node->getNormal());

  } else {
    hppDout(error, "~~ NODE cannot be cast");
    return;
  }

  hppDout(info, "~~ q = " << displayConfig(*q));

  core::RbprmValidationReportPtr_t rbReport =
      boost::dynamic_pointer_cast<core::RbprmValidationReport>(report);
  // checks :
  if (!rbReport) {
    hppDout(error, "~~ Validation Report cannot be cast");
    return;
  }
  if (rbReport->trunkInCollision) {
    hppDout(warning,
            "~~ ComputeGIWC : trunk is in collision");  // shouldn't happen
  }
  if (!rbReport->romsValid) {
    hppDout(warning,
            "~~ ComputeGIWC : roms filter not respected");  // shouldn't happen
  }

  // TODO
  polytope::T_rotation_t rotContact(3 * rbReport->ROMReports.size(), 3);
  polytope::vector_t posContact(3 * rbReport->ROMReports.size());

  // get the 2 object in contact for each ROM :
  hppDout(info,
          "~~ Number of roms in collision : " << rbReport->ROMReports.size());
  size_t indexRom = 0;
  for (std::map<std::string,
                core::CollisionValidationReportPtr_t>::const_iterator it =
           rbReport->ROMReports.begin();
       it != rbReport->ROMReports.end(); ++it) {
    hppDout(info, "~~ for rom : " << it->first);
    core::CollisionObjectPtr_t obj1 = it->second->object1;
    core::CollisionObjectPtr_t obj2 = it->second->object2;
    hppDout(notice, "~~ collision between : " << obj1->name() << " and "
                                              << obj2->name());
    fcl::CollisionResult result = it->second->result;
    /* size_t numContact =result.numContacts();
    hppDout(notice,"~~ number of contact : "<<numContact);
    std::ostringstream ss;
    ss<<"[";
    for(size_t i = 0 ; i < numContact ; i++)
    { // print with python formating :
      ss<<"["<<result.getContact(i).pos[0]<<","<<result.getContact(i).pos[1]<<","<<result.getContact(i).pos[2]<<"]";
      if(i< (numContact-1))
         ss<<",";
    }
    ss<<"]";
    std::cout<<ss.str()<<std::endl;
  */

    // get intersection between the two objects :
    obj1->fcl();
    geom::T_Point vertices1;
    geom::BVHModelOBConst_Ptr_t model1 = geom::GetModel(obj1->fcl());
    hppDout(info, "vertices obj1 : " << obj1->name() << " ( "
                                     << model1->num_vertices << " ) ");
    std::ostringstream ss1;
    ss1 << "[";
    for (int i = 0; i < model1->num_vertices; ++i) {
      vertices1.push_back(Eigen::Vector3d(model1->vertices[i][0],
                                          model1->vertices[i][1],
                                          model1->vertices[i][2]));
      // hppDout(notice,"vertices : "<<model1->vertices[i]);
      ss1 << "[" << model1->vertices[i][0] << "," << model1->vertices[i][1]
          << "," << model1->vertices[i][2] << "]";
      if (i < (model1->num_vertices - 1)) ss1 << ",";
    }
    ss1 << "]";
    // std::cout<<ss1.str()<<std::endl;

    obj2->fcl();
    geom::T_Point vertices2;
    geom::BVHModelOBConst_Ptr_t model2 = geom::GetModel(obj2->fcl());
    hppDout(info, "vertices obj2 : " << obj2->name() << " ( "
                                     << model2->num_vertices << " ) ");
    std::ostringstream ss2;
    ss2 << "[";
    for (int i = 0; i < model2->num_vertices; ++i) {
      vertices2.push_back(Eigen::Vector3d(model2->vertices[i][0],
                                          model2->vertices[i][1],
                                          model2->vertices[i][2]));
      // hppDout(notice,"vertices : "<<model2->vertices[i]);
      ss2 << "[" << model2->vertices[i][0] << "," << model2->vertices[i][1]
          << "," << model2->vertices[i][2] << "]";
      if (i < (model2->num_vertices - 1)) ss2 << ",";
    }
    ss2 << "]";
    // std::cout<<ss2.str()<<std::endl;

    hppStartBenchmark(COMPUTE_INTERSECTION);
    geom::T_Point hull = geom::intersectPolygonePlane(
        model1, model2, fcl::Vec3f(0, 0, 1), geom::ZJUMP, result);
    hppStopBenchmark(COMPUTE_INTERSECTION);
    hppDisplayBenchmark(COMPUTE_INTERSECTION);
    if (hull.size() == 0) {
      hppDout(error, "No intersection between rom and environnement");
      // node->giwc(0);
      return;
    }

    // todo : compute center point of the hull
    polytope::vector3_t normal, tangent0, tangent1;
    geom::Point center = geom::center(hull.begin(), hull.end());
    posContact.segment<3>(indexRom * 3) = center;
    std::cout << center << std::endl << std::endl;
    polytope::rotation_t rot;
    normal = -result.getContact(0).normal;
    hppDout(notice, " !!! normal for GIWC : " << normal);
    // compute tangent vector :
    tangent0 = normal.cross(polytope::vector3_t(1, 0, 0));
    if (tangent0.dot(tangent0) < 0.001)
      tangent0 = normal.cross(polytope::vector3_t(0, 1, 0));
    tangent1 = normal.cross(tangent0);
    rot(0, 0) = tangent0(0);
    rot(0, 1) = tangent1(0);
    rot(0, 2) = normal(0);
    rot(1, 0) = tangent0(1);
    rot(1, 1) = tangent1(1);
    rot(1, 2) = normal(1);
    rot(2, 0) = tangent0(2);
    rot(2, 1) = tangent1(2);
    rot(2, 2) = normal(2);

    rotContact.block<3, 3>(indexRom * 3, 0) = rot;
    std::cout << rot << std::endl << std::endl;

    indexRom++;
  }  // for each ROMS

  polytope::vector_t x(rbReport->ROMReports.size());
  polytope::vector_t y(rbReport->ROMReports.size());
  polytope::vector_t nu(rbReport->ROMReports.size());
  for (size_t k = 0; k < rbReport->ROMReports.size(); ++k) {
    x(k) = 0.25;  // approx size of foot
    y(k) = 0.15;
    nu(k) = 0.5;
  }
  // save giwc in node structure
  // node->giwc(polytope::U_stance(rotContact,posContact,nu,x,y));

}  // computeGIWC

}  // namespace rbprm
}  // namespace hpp
