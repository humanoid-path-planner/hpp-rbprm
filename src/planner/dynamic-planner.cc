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

#include <hpp/fcl/collision_data.h>

#include <boost/tuple/tuple.hpp>
#include <hpp/centroidal-dynamics/centroidal_dynamics.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/configuration-shooter/uniform.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/kinodynamic-distance.hh>
#include <hpp/core/node.hh>
#include <hpp/core/path-projector.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/path.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/rbprm/planner/dynamic-planner.hh>
#include <hpp/rbprm/planner/parabola-path.hh>
#include <hpp/rbprm/planner/rbprm-roadmap.hh>
#include <hpp/rbprm/planner/rbprm-steering-kinodynamic.hh>
#include <hpp/rbprm/rbprm-device.hh>
#include <hpp/rbprm/rbprm-path-validation.hh>
#include <hpp/rbprm/rbprm-validation-report.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/timer.hh>

#include "hpp/rbprm/utils/algorithms.h"

namespace hpp {
namespace rbprm {
using core::BiRRTPlanner;
using core::Configuration_t;
using core::ConfigurationPtr_t;
using core::Path;
using core::PathPtr_t;
using core::Problem;
using core::Roadmap;
using core::RoadmapPtr_t;
using core::size_type;
using pinocchio::displayConfig;
using pinocchio::value_type;

typedef centroidal_dynamics::MatrixXX MatrixXX;
typedef centroidal_dynamics::Matrix6X Matrix6X;
typedef centroidal_dynamics::Vector3 Vector3;
typedef centroidal_dynamics::Matrix3 Matrix3;
typedef centroidal_dynamics::Matrix63 Matrix63;
typedef centroidal_dynamics::Vector6 Vector6;
typedef centroidal_dynamics::VectorX VectorX;

DynamicPlannerPtr_t DynamicPlanner::createWithRoadmap(
    core::ProblemConstPtr_t problem, const RoadmapPtr_t& roadmap) {
  DynamicPlanner* ptr = new DynamicPlanner(problem, roadmap);
  return DynamicPlannerPtr_t(ptr);
}

DynamicPlannerPtr_t DynamicPlanner::create(core::ProblemConstPtr_t problem) {
  DynamicPlanner* ptr = new DynamicPlanner(problem);
  return DynamicPlannerPtr_t(ptr);
}

DynamicPlanner::DynamicPlanner(core::ProblemConstPtr_t problem)
    : BiRRTPlanner(problem),
      qProj_(new core::Configuration_t(problem->robot()->configSize())),
      roadmap_(std::dynamic_pointer_cast<core::Roadmap>(
          core::RbprmRoadmap::create(problem->distance(), problem->robot()))),
      sm_(std::dynamic_pointer_cast<SteeringMethodKinodynamic>(
          problem->steeringMethod())),
      smParabola_(rbprm::SteeringMethodParabola::create(problem)),
      rbprmPathValidation_(std::dynamic_pointer_cast<RbPrmPathValidation>(
          problem->pathValidation())) {
  assert(sm_ &&
         "steering method should be a kinodynamic steering method for this "
         "solver");
  assert(
      rbprmPathValidation_ &&
      "Path validation should be a RbPrmPathValidation class for this solver");
  assert(problem->robot()->mass() > 0. &&
         "When using dynamic planner, the robot mass should be correctly "
         "defined.");
  hppDout(notice, "number of affordances objects : "
                      << problem->collisionObstacles().size());

  sizeFootX_ = problem->getParameter(std::string("DynamicPlanner/sizeFootX"))
                   .floatValue() /
               2.;
  sizeFootY_ = problem->getParameter(std::string("DynamicPlanner/sizeFootY"))
                   .floatValue() /
               2.;
  if (sizeFootX_ > 0. && sizeFootY_ > 0.)
    rectangularContact_ = 1;
  else
    rectangularContact_ = 0;

  tryJump_ =
      problem->getParameter(std::string("DynamicPlanner/tryJump")).boolValue();
  hppDout(notice, "tryJump in dynamic planner = " << tryJump_);
  mu_ = problem->getParameter(std::string("DynamicPlanner/friction"))
            .floatValue();
  hppDout(notice, "mu define in python : " << mu_);
}

DynamicPlanner::DynamicPlanner(core::ProblemConstPtr_t problem,
                               const RoadmapPtr_t& roadmap)
    : BiRRTPlanner(problem, roadmap),
      qProj_(new core::Configuration_t(problem->robot()->configSize())),
      roadmap_(std::dynamic_pointer_cast<core::Roadmap>(
          core::RbprmRoadmap::create(problem->distance(), problem->robot()))),
      sm_(std::dynamic_pointer_cast<SteeringMethodKinodynamic>(
          problem->steeringMethod())),
      smParabola_(rbprm::SteeringMethodParabola::create(problem)),
      rbprmPathValidation_(std::dynamic_pointer_cast<RbPrmPathValidation>(
          problem->pathValidation())) {
  assert(sm_ &&
         "steering method should be a kinodynamic steering method for this "
         "solver");
  assert(
      rbprmPathValidation_ &&
      "Path validation should be a RbPrmPathValidation class for this solver");
  assert(problem->robot()->mass() > 0. &&
         "When using dynamic planner, the robot mass should be correctly "
         "defined.");

  hppDout(notice, "number of affordances objects : "
                      << problem->collisionObstacles().size());
  sizeFootX_ = problem->getParameter(std::string("DynamicPlanner/sizeFootX"))
                   .floatValue() /
               2.;
  sizeFootY_ = problem->getParameter(std::string("DynamicPlanner/sizeFootY"))
                   .floatValue() /
               2.;
  if (sizeFootX_ > 0. && sizeFootY_ > 0.)
    rectangularContact_ = 1;
  else
    rectangularContact_ = 0;

  tryJump_ =
      problem->getParameter(std::string("DynamicPlanner/tryJump")).boolValue();
  hppDout(notice, "tryJump in dynamic planner = " << tryJump_);
  mu_ = problem->getParameter(std::string("DynamicPlanner/friction"))
            .floatValue();
  hppDout(notice, "mu define in python : " << mu_);
}

void DynamicPlanner::init(const DynamicPlannerWkPtr_t& weak) {
  BiRRTPlanner::init(weak);
  weakPtr_ = weak;
}

core::PathPtr_t DynamicPlanner::extendInternal(
    core::ConfigurationPtr_t& qProj_, const core::NodePtr_t& near,
    const core::ConfigurationPtr_t& target, bool reverse) {
  const core::ConstraintSetPtr_t& constraints(sm_->constraints());
  if (constraints) {
    core::ConfigProjectorPtr_t configProjector(constraints->configProjector());
    if (configProjector) {
      configProjector->projectOnKernel(*(near->configuration()), *target,
                                       *qProj_);
    } else {
      *qProj_ = *target;
    }

    if (constraints->apply(*qProj_)) {
      return reverse ? (*sm_)(*qProj_, near) : (*sm_)(near, *qProj_);
    } else {
      return PathPtr_t();
    }
  }
  return reverse ? (*sm_)(*target, near) : (*sm_)(near, *target);
}

core::PathPtr_t DynamicPlanner::extendParabola(
    const core::ConfigurationPtr_t& from,
    const core::ConfigurationPtr_t& target, bool reverse) {
  const core::SteeringMethodPtr_t& sm(problem()->steeringMethod());
  const core::ConstraintSetPtr_t& constraints(sm->constraints());
  core::PathPtr_t path;
  if (constraints) {
    core::ConfigProjectorPtr_t configProjector(constraints->configProjector());
    if (configProjector) {
      configProjector->projectOnKernel(*from, *target, *qProj_);
    } else {
      *qProj_ = *target;
    }
    if (constraints->apply(*qProj_)) {
      if (reverse)
        path = (*smParabola_)(*qProj_, *from);
      else
        path = (*smParabola_)(*from, *qProj_);
    } else {
      return core::PathPtr_t();
    }
  } else {
    if (reverse)
      path = (*smParabola_)(*target, *from);
    else
      path = (*smParabola_)(*from, *target);
  }
  return path;
}

bool DynamicPlanner::tryParabolaPath(const core::NodePtr_t& x_near,
                                     core::ConfigurationPtr_t q_last,
                                     const core::ConfigurationPtr_t& target,
                                     bool reverse, core::NodePtr_t& x_jump,
                                     core::NodePtr_t& x_new,
                                     core::PathPtr_t& kinoPath,
                                     core::PathPtr_t& paraPath) {
  bool success(false);
  core::PathValidationPtr_t pathValidation(problem()->pathValidation());
  core::PathPtr_t validPath;
  core::PathValidationReportPtr_t report;
  const size_type indexECS =
      problem()->robot()->configSize() -
      problem()->robot()->extraConfigSpace().dimension();  // ecs index
  bool kinoPathValid(false);
  hppDout(notice, "!! begin tryParabolaPath");

  // 1. compute a parabola between last configuration valid in contact, and
  // qrand (target)
  paraPath = extendParabola(q_last, target, reverse);
  if (paraPath) {
    hppDout(notice, "!! ParaPath computed");
    if (paraPath->length() >
        0) {  // only add if the full path is valid (because we can't extract a
              // subpath of a parabola path)
      hppDout(notice, "!! parabola path valid !");
      ParabolaPathPtr_t parabolaPath =
          std::dynamic_pointer_cast<ParabolaPath>(paraPath);
      core::ConfigurationPtr_t q_new(
          new core::Configuration_t(parabolaPath->end()));
      core::ConfigurationPtr_t q_jump(
          new core::Configuration_t(parabolaPath->initial()));
      // 2. update q_jump with the correct initial velocity needed for the
      // computed parabola
      // TODO : update q_jump with the right velocity from parabola
      for (size_t i = 0; i < 3; ++i) {
        (*q_jump)[indexECS + i] = parabolaPath->V0_[i];
        (*q_new)[indexECS + i] = parabolaPath->Vimp_[i];
      }

      hppDout(notice, "q_last = " << displayConfig(*q_last));
      hppDout(notice, "q_jump = " << displayConfig(*q_jump));
      hppDout(notice, "q_target = " << displayConfig(*target));
      hppDout(notice, "q_new = " << displayConfig(*q_new));

      // 3. compute a kinodynamic path between near and q_jump
      hppStartBenchmark(EXTEND);
      kinoPath = extendInternal(qProj_, x_near, q_jump, reverse);
      hppStopBenchmark(EXTEND);
      hppDisplayBenchmark(EXTEND);
      if (kinoPath) {
        hppDout(notice, "!! Kino path computed");
        kinoPathValid =
            pathValidation->validate(kinoPath, false, validPath, report);
        if (kinoPathValid) {
          hppDout(notice, "!! Kino path valid !");
          value_type t_final = validPath->timeRange().second;
          if (t_final != kinoPath->timeRange().first &&
              validPath->end() == *(q_jump)) {
            // 4. add both nodes and edges to the roadmap
            success = true;
            hppDout(notice, "add both nodes and edges to the roadmap");
            x_jump = roadmap()->addNodeAndEdge(x_near, q_jump, kinoPath);
            computeGIWC(x_jump);
            x_new = roadmap()->addNodeAndEdge(x_jump, q_new, paraPath);
            computeGIWC(x_new);
          } else {
            hppDout(notice, "!! lenght of Kino path incorrect.");
          }
        } else {
          hppDout(notice, "!! Kino path not valid.");
        }
      } else {
        hppDout(notice, "!! Kino path doesn't exist.");
      }
    } else {
      hppDout(notice, "!! Parabola path not valid.");
    }
  } else {
    hppDout(notice, "!! parabola path doesn't exist.");
  }

  return success;
}

void DynamicPlanner::oneStep() {
  hppDout(info,
          "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ new Step "
          "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  PathPtr_t validPath, path;
  core::PathValidationPtr_t pathValidation(problem()->pathValidation());
  value_type distance;
  core::NodePtr_t near, reachedNodeFromStart;
  bool startComponentConnected(false), pathValidFromStart(false),
      pathValidFromEnd(false);
  ConfigurationPtr_t q_new;
  hppStartBenchmark(SHOOT);
  ConfigurationPtr_t q_rand = configurationShooter_->shoot();
  hppDout(info, "Random configuration : " << displayConfig(*q_rand));
  hppStopBenchmark(SHOOT);
  hppDisplayBenchmark(SHOOT);

  // ######################## first , try to connect to start component
  // #################### //
  hppStartBenchmark(NEAREST);
  near = roadmap()->nearestNode(q_rand, startComponent_, distance);
  hppStopBenchmark(NEAREST);
  hppDisplayBenchmark(NEAREST);

  core::RbprmNodePtr_t castNode = static_cast<core::RbprmNodePtr_t>(near);
  if (castNode)
    hppDout(notice, "Node casted correctly");
  else
    hppDout(notice, "Impossible to cast node to rbprmNode");

  hppStartBenchmark(EXTEND);
  path = extendInternal(qProj_, near, q_rand);
  hppStopBenchmark(EXTEND);
  hppDisplayBenchmark(EXTEND);
  if (path) {
    core::PathValidationReportPtr_t report;
    pathValidFromStart =
        pathValidation->validate(path, false, validPath, report);

    // Insert new path to q_near in roadmap
    if (validPath) {
      if (validPath->timeRange().second != path->timeRange().first) {
        pathValidFromStart =
            pathValidFromStart && (validPath->end() == *q_rand);
        startComponentConnected = true;
        q_new = ConfigurationPtr_t(new Configuration_t(validPath->end()));
        reachedNodeFromStart =
            roadmap()->addNodeAndEdge(near, q_new, validPath);
        computeGIWC(reachedNodeFromStart);
        hppDout(info,
                "~~~~~~~~~~~~~~~~~~~~ New node added to start component : "
                    << displayConfig(*q_new));
      } else {
        pathValidFromStart = false;
      }
    }
  }

  hppDout(info,
          "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Try to connect end component "
          "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");

  // ######################## now try to connect qrand to end components (in
  // reverse )######################## //
  for (auto& itcc : endComponents_) {
    hppStartBenchmark(NEAREST);
    near = roadmap()->nearestNode(q_rand, itcc, distance, true);
    hppStopBenchmark(NEAREST);
    hppDisplayBenchmark(NEAREST);

    hppStartBenchmark(EXTEND);
    path = extendInternal(qProj_, near, q_rand, true);
    hppStopBenchmark(EXTEND);
    hppDisplayBenchmark(EXTEND);
    if (path) {
      core::PathValidationReportPtr_t report;
      pathValidFromEnd =
          pathValidation->validate(path, true, validPath, report);
      if (pathValidFromStart && validPath) {
        pathValidFromEnd = pathValidFromEnd && (validPath->initial() == *q_new);
      }
      if (pathValidFromStart &&
          pathValidFromEnd)  // qrand was successfully connected to both trees
      {
        // we won, a path is found
        roadmap()->addEdge(reachedNodeFromStart, near, validPath);
        hppDout(
            info,
            "~~~~~~~~~~~~~~~~~~~~ Start and goal component connected !!!!!! "
                << displayConfig(*q_new));
        hppDout(notice, "#### end of planning phase #### ");
        return;
      } else if (validPath) {
        value_type t_final = validPath->timeRange().second;
        if (t_final != path->timeRange().first) {
          ConfigurationPtr_t q_newEnd =
              ConfigurationPtr_t(new Configuration_t(validPath->initial()));
          core::NodePtr_t newNode =
              roadmap()->addNodeAndEdge(q_newEnd, near, validPath);
          computeGIWC(newNode);
          hppDout(info,
                  "~~~~~~~~~~~~~~~~~~~~~~ New node added to end component : "
                      << displayConfig(*q_newEnd));

          if (startComponentConnected) {  // now try to connect both nodes (qnew
                                          // -> qnewEnd)
            hppStartBenchmark(EXTEND);
            path =
                extendInternal(qProj_, reachedNodeFromStart, q_newEnd, false);
            hppStopBenchmark(EXTEND);
            hppDisplayBenchmark(EXTEND);
            if (path &&
                pathValidation->validate(path, false, validPath, report)) {
              if (validPath->end() == *q_newEnd) {
                roadmap()->addEdge(reachedNodeFromStart, newNode, path);
                hppDout(info,
                        "~~~~~~~~ both new nodes connected together !!!!!! "
                            << displayConfig(*q_new));
                return;
              }
            }
          }
        }
      }
    }
  }
}

void DynamicPlanner::computeGIWC(const core::NodePtr_t x, bool use_bestReport) {
  core::ValidationReportPtr_t report;
  // randomnize the collision pair, in order to get a different surface of
  // contact each time (because only the first one in collision is considered by
  // fcl and put in the report)
  rbprmPathValidation_->getValidator()
      ->randomnizeCollisionPairs();  // FIXME : remove if we compute all
                                     // collision pairs
  rbprmPathValidation_->getValidator()->computeAllContacts(true);
  hppDout(notice, "Compute GIWC, call validate for configuration : "
                      << pinocchio::displayConfig(*(x->configuration())));
  problem()->configValidations()->validate(*(x->configuration()), report);
  rbprmPathValidation_->getValidator()->computeAllContacts(false);
  if (use_bestReport) {
    core::RbprmNodePtr_t node = static_cast<core::RbprmNodePtr_t>(x);
    node->chooseBestContactSurface(
        report,
        std::dynamic_pointer_cast<pinocchio::RbPrmDevice>(problem()->robot()));
  }
  computeGIWC(x, report);
}

void DynamicPlanner::computeGIWC(const core::NodePtr_t xNode,
                                 core::ValidationReportPtr_t report) {
  core::RbprmNodePtr_t node = static_cast<core::RbprmNodePtr_t>(xNode);
  hppDout(notice, "## compute GIWC");
  core::ConfigurationPtr_t q = node->configuration();
  // fil normal information in node
  if (node) {
    hppDout(info, "~~ NODE cast correctly");
  } else {
    hppDout(error, "~~ NODE cannot be cast");
    return;
  }

  hppDout(info, "~~ q = " << displayConfig(*q));
  node->fillNodeMatrices(
      report, rectangularContact_, sizeFootX_, sizeFootY_,
      problem()->robot()->mass(), mu_,
      std::dynamic_pointer_cast<pinocchio::RbPrmDevice>(problem()->robot()));
}  // computeGIWC

// re implement virtual method, same as base class but without the symetric edge
// (goal -> start)
void DynamicPlanner::tryConnectInitAndGoals() {
  // call steering method here to build a direct conexion
  core::PathValidationPtr_t pathValidation(problem()->pathValidation());
  core::PathProjectorPtr_t pathProjector(problem()->pathProjector());
  core::PathPtr_t validPath, projPath, path, kinoPath, paraPath;
  core::NodePtr_t initNode = roadmap()->initNode();
  core::NodePtr_t x_jump;
  computeGIWC(initNode, true);
  for (auto& itn : roadmap()->goalNodes()) {
    computeGIWC(itn, true);
    core::ConfigurationPtr_t q1((initNode)->configuration());
    core::ConfigurationPtr_t q2(itn->configuration());
    assert(*q1 != *q2);
    hppStartBenchmark(EXTEND);
    path = extendInternal(qProj_, initNode, q2);
    hppStopBenchmark(EXTEND);
    hppDisplayBenchmark(EXTEND);
    hppDout(notice, "try direction path, after extendInternal");
    if (!path) continue;
    hppDout(notice, "try direction path, after continue");

    if (pathProjector) {
      if (!pathProjector->apply(path, projPath)) continue;
    } else {
      projPath = path;
    }
    if (projPath) {
      core::PathValidationReportPtr_t report;
      // roadmap ()->addEdge (initNode, itn, projPath);  // (TODO a
      // supprimer)display the path no matter if it's successful or not

      bool pathValid =
          pathValidation->validate(projPath, false, validPath, report);

      // roadmap ()->addEdge (initNode, itn, validPath);  // (TODO a
      // supprimer)display the path no matter if it's successful or not

      if (pathValid &&
          validPath->timeRange().second !=
              path->timeRange().first) {  // connection to goal config
                                          // successfull, add the edge
        roadmap()->addEdge(initNode, itn, projPath);
      } else if (validPath) {
        if (tryJump_) {
          std::vector<std::string> filter;
          core::ValidationReportPtr_t valReport;
          // check if the validation fail because of the ROM or because of the
          // trunk
          RbPrmPathValidationPtr_t rbprmPathValidation =
              std::dynamic_pointer_cast<RbPrmPathValidation>(pathValidation);
          bool successPathOperator;
          bool trunkValid = rbprmPathValidation->getValidator()->validate(
              (*projPath)(report->parameter, successPathOperator), valReport,
              filter);
          if (trunkValid &&
              successPathOperator) {  // if it failed because of the ROM, we can
                                      // try a parabola
            core::ConfigurationPtr_t q_jump(
                new core::Configuration_t(validPath->end()));
            core::NodePtr_t x_goal;
            bool parabolaSuccess =
                tryParabolaPath(initNode, q_jump, q2, false, x_jump, x_goal,
                                kinoPath, paraPath);
            hppDout(notice, "parabola success = " << parabolaSuccess);
            if (parabolaSuccess) {
              hppDout(notice, "x_goal conf = "
                                  << displayConfig(*(x_goal->configuration())));
              roadmap()->addEdge(x_jump, itn, paraPath);
            }
          } else {
            hppDout(notice, "trunk in collision");
          }
        } else if (validPath->timeRange().second !=
                   path->timeRange().first) {  // add the last valid configu
                                               // reached to the roadmap
          core::ConfigurationPtr_t q_new(
              new core::Configuration_t(validPath->end()));
          core::NodePtr_t x_new =
              roadmap()->addNodeAndEdge(initNode, q_new, validPath);
          computeGIWC(x_new);
        }
      }
    }
  }
}

core::PathVectorPtr_t DynamicPlanner::finishSolve(
    const core::PathVectorPtr_t& path) {
  /*std::cout<<"total_path_computed = "<<sm_->totalTimeComputed_<<std::endl;
  std::cout<<"total_path_validated = "<<sm_->totalTimeValidated_<<std::endl;
  std::cout<<"percentage validated path
  ="<<((sm_->totalTimeValidated_)/(sm_->totalTimeComputed_))*100.<<std::endl;
  std::cout<<"rejected_paths = "<<sm_->rejectedPath_<<std::endl;
  */
  /*
  std::ofstream myfile;
  myfile.open ("/local/dev_hpp/benchs/benchHyq_darpa.txt", std::ios::out |
  std::ios::app ); myfile<<"total_path_computed =
  "<<sm_->totalTimeComputed_<<std::endl; myfile<<"total_path_validated =
  "<<sm_->totalTimeValidated_<<std::endl; myfile<<"percentage_validated_path
  ="<<(double)sm_->totalTimeValidated_/sm_->totalTimeValidated_<<std::endl;
  myfile<<"total_direction_computed = "<<sm_->dirTotal_<<std::endl;
  myfile<<"total_direction_valid = "<<sm_->dirValid_<<std::endl;
  myfile<<"percentage_valide_direction
  ="<<(double)(((double)sm_->dirValid_)/((double)sm_->dirTotal_))<<std::endl;
  myfile<<"rejected_paths = "<<sm_->rejectedPath_<<std::endl;
  myfile<<"num_nodes = "<<roadmap()->nodes().size()<<std::endl;

  myfile.close();
  */
  return path;
}

HPP_START_PARAMETER_DECLARATION(Kinodynamic)
Problem::declareParameter(core::ParameterDescription(
    core::Parameter::FLOAT, "DynamicPlanner/sizeFootX",
    "The lenght of the feet along X axis (assuming rectangular feet).",
    core::Parameter(0.)));
Problem::declareParameter(core::ParameterDescription(
    core::Parameter::FLOAT, "DynamicPlanner/sizeFootY",
    "The lenght of the feet along Y axis (assuming rectangular feet).",
    core::Parameter(0.)));
Problem::declareParameter(core::ParameterDescription(
    core::Parameter::FLOAT, "DynamicPlanner/friction",
    "Value of the friction coefficient between the feet of the robot and the "
    "ground.",
    core::Parameter(0.5)));
Problem::declareParameter(core::ParameterDescription(
    core::Parameter::BOOL, "DynamicPlanner/tryJump",
    "If True, when a trajectory is invalid because all the rom leave "
    "the contact, a ballistic motion is tried to connect both states",
    core::Parameter(false)));
HPP_END_PARAMETER_DECLARATION(Kinodynamic)

}  // namespace rbprm
}  // namespace hpp
