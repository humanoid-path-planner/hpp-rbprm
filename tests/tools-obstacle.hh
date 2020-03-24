
// Copyright (C) 2018 LAAS-CNRS
// Author: Pierre Fernbach
//
// This file is part of the hpp-rbprm.
//
// hpp-rbprm is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// test-hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-core.  If not, see <http://www.gnu.org/licenses/>.

#ifndef TOOLSOBSTACLE_HH
#define TOOLSOBSTACLE_HH

#include <pinocchio/parsers/urdf.hpp>
#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/fwd.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/affordance/affordance-extraction.hh>
#include <hpp/affordance/operations.hh>
#include <hpp/rbprm/rbprm-shooter.hh>
#include <hpp/rbprm/rbprm-device.hh>
#include <hpp/rbprm/rbprm-path-validation.hh>
#include <hpp/rbprm/planner/dynamic-planner.hh>
#include <hpp/rbprm/planner/rbprm-steering-kinodynamic.hh>
#include <hpp/rbprm/planner/random-shortcut-dynamic.hh>
#include <hpp/rbprm/planner/oriented-path-optimizer.hh>
#include <hpp/rbprm/dynamic/dynamic-path-validation.hh>

using namespace hpp;
using namespace hpp::core;

void addAffObjects(hpp::core::ProblemSolver& problemSolver, const affordance::OperationBases_t& ops,
                   const std::vector<affordance::CollisionObjects_t>& affObjs,
                   const std::string obstacleNameNonAff) throw(std::runtime_error) {
  const std::string affSuffix = "aff";
  std::string obstacleName(obstacleNameNonAff);
  obstacleName += affSuffix;
  for (unsigned int opIdx = 0; opIdx < ops.size(); opIdx++) {
    AffordanceObjects_t objs;
    affordance::CollisionObjects_t affs = affObjs[opIdx];
    for (unsigned int objIdx = 0; objIdx < affs.size(); objIdx++) {
      std::stringstream ss;
      ss << opIdx << "_" << objIdx;
      std::string ig = obstacleName + ss.str();
      problemSolver.addObstacle(ig, *(affs[objIdx]), false, false);
      hpp::pinocchio::CollisionObjectPtr_t obj = problemSolver.obstacle(ig);
      objs.push_back(std::make_pair(ig, obj));
    }
    if (problemSolver.affordanceObjects.has(ops[opIdx]->affordance_)) {
      AffordanceObjects_t mapObjs = problemSolver.affordanceObjects.get(ops[opIdx]->affordance_);
      objs.insert(objs.begin() + objs.size(), mapObjs.begin(), mapObjs.end());
    }
    problemSolver.affordanceObjects.erase(ops[opIdx]->affordance_);
    problemSolver.affordanceObjects.add(ops[opIdx]->affordance_, objs);
  }
}

void affordanceAnalysis(ProblemSolver& problemSolver, const std::string& obstacleName,
                        const affordance::OperationBases_t& operations, std::vector<double> reduceSizes) {
  std::list<std::string> obstacles = problemSolver.obstacleNames(true, true);
  std::list<std::string>::iterator objIt = std::find(obstacles.begin(), obstacles.end(), obstacleName);
  while (reduceSizes.size() < operations.size()) reduceSizes.push_back(0.);
  if (objIt == obstacles.end()) throw std::runtime_error("No obstacle by given name found. Unable to analyse.");
  try {
    affordance::SemanticsDataPtr_t aff =
        affordance::affordanceAnalysis((problemSolver.obstacle(obstacleName)->fcl()), operations);
    std::vector<std::vector<fcl::CollisionObjectPtr_t> > affObjs =
        affordance::getReducedAffordanceObjects(aff, reduceSizes);
    // add fcl::CollisionObstacles to problemSolver
    addAffObjects(problemSolver, operations, affObjs, obstacleName);
  } catch (const std::exception& exc) {
    throw std::runtime_error(exc.what());
  }
}

affordance::OperationBases_t createOperations(hpp::core::ProblemSolver& pSolver) throw(std::runtime_error) {
  pSolver.affordanceConfigs.add("Support", vector3_t(0.3, 0.3, 0.05));
  pSolver.affordanceConfigs.add("Lean", vector3_t(0.1, 0.3, 0.05));
  pSolver.affordanceConfigs.add("Support45", vector3_t(0.1, 0.3, 0.05));
  if (!pSolver.affordanceConfigs.has("Support"))
    throw std::runtime_error("No 'Support' affordance type found Afford::createOperations ()");
  const hpp::pinocchio::vector3_t& sconf = pSolver.affordanceConfigs.get("Support");
  if (!pSolver.affordanceConfigs.has("Lean"))
    throw std::runtime_error("No 'Lean' affordance type found in Afford::createOperations ()");
  const hpp::pinocchio::vector3_t& lconf = pSolver.affordanceConfigs.get("Lean");
  if (!pSolver.affordanceConfigs.has("Support45"))
    throw std::runtime_error("No 'Support45' affordance type found in Afford::createOperations ()");
  const hpp::pinocchio::vector3_t& s45conf = pSolver.affordanceConfigs.get("Support45");
  affordance::SupportOperationPtr_t support(new affordance::SupportOperation(sconf[0], sconf[1], sconf[2]));
  affordance::LeanOperationPtr_t lean(new affordance::LeanOperation(lconf[0], lconf[1], lconf[2]));
  affordance::Support45OperationPtr_t support45(
      new affordance::Support45Operation(s45conf[0], s45conf[1], s45conf[2]));

  affordance::OperationBases_t operations;
  operations.push_back(support);
  operations.push_back(lean);
  operations.push_back(support45);

  return operations;
}

void loadObstacleWithAffordance(hpp::core::ProblemSolver& pSolver, std::string packagename, std::string filename,
                                std::string prefix, const affordance::OperationBases_t& operations,
                                std::vector<double> reduceSizes) {
  DevicePtr_t device(hpp::pinocchio::Device::create(prefix));
  if (packagename.empty())
    hpp::pinocchio::urdf::loadModelFromString(device, 0, "", "anchor", filename, "");
  else
    hpp::pinocchio::urdf::loadUrdfModel(device, "anchor", packagename, filename);
  device->controlComputation(hpp::pinocchio::JOINT_POSITION);
  pSolver.addObstacle(device, true, true);
  std::list<std::string> obstacles = pSolver.obstacleNames(true, false);
  for (std::list<std::string>::const_iterator cit = obstacles.begin(); cit != obstacles.end(); ++cit) {
    if ((*cit).find(prefix) == 0) affordanceAnalysis(pSolver, *cit, operations, reduceSizes);
  }
}

void loadObstacleWithAffordance(hpp::core::ProblemSolver& pSolver, std::string packagename, std::string filename,
                                std::string prefix) {
  affordance::OperationBases_t operations = createOperations(pSolver);
  std::vector<double> reduceSizes;
  while (reduceSizes.size() < operations.size()) reduceSizes.push_back(0.);
  return loadObstacleWithAffordance(pSolver, packagename, filename, prefix, operations, reduceSizes);
}

void loadDarpa(hpp::core::ProblemSolver& pSolver) {
  loadObstacleWithAffordance(pSolver, std::string("hpp_environments"), std::string("multicontact/darpa"),
                             std::string("planning"));
}

typedef hpp::core::Container<hpp::core::AffordanceObjects_t> affMap_t;
struct BindShooter {
  BindShooter(const std::size_t shootLimit = 10000, const std::size_t displacementLimit = 100)
      : shootLimit_(shootLimit), displacementLimit_(displacementLimit) {}

  hpp::rbprm::RbPrmShooterPtr_t create(/*const hpp::pinocchio::DevicePtr_t& robot,*/ const hpp::core::Problem& problem,
                                       const hpp::core::ProblemSolverPtr_t problemSolver) {
    hpp::core::Container<hpp::core::AffordanceObjects_t> affMap_ = problemSolver->affordanceObjects;
    affMap_ = problemSolver->affordanceObjects;
    hpp::pinocchio::RbPrmDevicePtr_t robotcast =
        boost::static_pointer_cast<hpp::pinocchio::RbPrmDevice>(problem.robot());
    if (affMap_.map.empty()) {
      throw std::runtime_error("No affordances found. Unable to create shooter object.");
    }
    rbprm::RbPrmShooterPtr_t shooter =
        hpp::rbprm::RbPrmShooter::create(robotcast, problemSolver->problem()->collisionObstacles(), affMap_,
                                         romFilter_, affFilter_, shootLimit_, displacementLimit_);
    if (!so3Bounds_.empty()) shooter->BoundSO3(so3Bounds_);
    return shooter;
  }

  hpp::core::PathValidationPtr_t createPathValidation(const hpp::pinocchio::DevicePtr_t& robot,
                                                      const hpp::pinocchio::value_type& val,
                                                      const hpp::core::ProblemSolverPtr_t problemSolver) {
    hpp::pinocchio::RbPrmDevicePtr_t robotcast = boost::static_pointer_cast<hpp::pinocchio::RbPrmDevice>(robot);
    hpp::core::Container<hpp::core::AffordanceObjects_t> affMap_ = problemSolver->affordanceObjects;
    if (affMap_.map.empty()) {
      throw std::runtime_error("No affordances found. Unable to create Path Validaton object.");
    }
    hpp::rbprm::RbPrmValidationPtr_t validation(
        hpp::rbprm::RbPrmValidation::create(robotcast, romFilter_, affFilter_, affMap_));
    hpp::rbprm::RbPrmPathValidationPtr_t collisionChecking = hpp::rbprm::RbPrmPathValidation::create(robot, val);
    collisionChecking->add(validation);
    problemSolver->problem()->configValidation(core::ConfigValidations::create());
    problemSolver->problem()->configValidations()->add(validation);
    return collisionChecking;
  }

  hpp::core::PathValidationPtr_t createDynamicPathValidation(const hpp::pinocchio::DevicePtr_t& robot,
                                                             const hpp::pinocchio::value_type& val,
                                                             const hpp::core::ProblemSolverPtr_t problemSolver) {
    hpp::pinocchio::RbPrmDevicePtr_t robotcast = boost::static_pointer_cast<hpp::pinocchio::RbPrmDevice>(robot);
    hpp::core::Container<hpp::core::AffordanceObjects_t> affMap_ = problemSolver->affordanceObjects;
    if (affMap_.map.empty()) {
      throw std::runtime_error("No affordances found. Unable to create Path Validaton object.");
    }
    hpp::rbprm::RbPrmValidationPtr_t validation(
        hpp::rbprm::RbPrmValidation::create(robotcast, romFilter_, affFilter_, affMap_));
    hpp::rbprm::DynamicPathValidationPtr_t collisionChecking = hpp::rbprm::DynamicPathValidation::create(robot, val);
    collisionChecking->add(validation);
    problemSolver->problem()->configValidation(core::ConfigValidations::create());
    problemSolver->problem()->configValidations()->add(validation);
    // build the dynamicValidation :
    double sizeFootX, sizeFootY, mass, mu;
    bool rectangularContact;
    sizeFootX = problemSolver->problem()->getParameter(std::string("DynamicPlanner/sizeFootX")).floatValue() / 2.;
    sizeFootY = problemSolver->problem()->getParameter(std::string("DynamicPlanner/sizeFootY")).floatValue() / 2.;
    if (sizeFootX > 0. && sizeFootY > 0.)
      rectangularContact = 1;
    else
      rectangularContact = 0;
    mass = robot->mass();
    mu = problemSolver->problem()->getParameter(std::string("DynamicPlanner/friction")).floatValue();
    hppDout(notice, "mu define in python : " << mu);
    DynamicValidationPtr_t dynamicVal =
        DynamicValidation::create(rectangularContact, sizeFootX, sizeFootY, mass, mu, robot);
    collisionChecking->addDynamicValidator(dynamicVal);

    return collisionChecking;
  }

  std::vector<std::string> romFilter_;
  std::map<std::string, std::vector<std::string> > affFilter_;
  std::size_t shootLimit_;
  std::size_t displacementLimit_;
  std::vector<double> so3Bounds_;
};

std::vector<double> addSo3LimitsHyQ() {
  std::vector<double> res;
  res.push_back(-0.4);
  res.push_back(0.4);
  res.push_back(-0.3);
  res.push_back(0.3);
  res.push_back(-0.3);
  res.push_back(0.3);
  return res;
}

hpp::core::ProblemSolverPtr_t configureRbprmProblemSolverForSupportLimbs(const DevicePtr_t& robot,
                                                                         BindShooter& bShooter) {
  hpp::core::ProblemSolverPtr_t ps = hpp::core::ProblemSolver::create();

  /*bind shooter init*/
  hpp::pinocchio::RbPrmDevicePtr_t robotcast = boost::static_pointer_cast<hpp::pinocchio::RbPrmDevice>(robot);
  std::vector<std::string> affNames;
  affNames.push_back(std::string("Support"));
  for (std::map<std::string, DevicePtr_t>::const_iterator cit = robotcast->robotRoms_.begin();
       cit != robotcast->robotRoms_.end(); ++cit) {
    bShooter.affFilter_.insert(std::make_pair(cit->first, affNames));
    bShooter.romFilter_.push_back(cit->first);
  }
  /*END bind shoorther init*/

  ps->robot(robot);
  ps->configurationShooters.add("RbprmShooter", boost::bind(&BindShooter::create, boost::ref(bShooter), _1, ps));
  ps->pathValidations.add("RbprmPathValidation",
                          boost::bind(&BindShooter::createPathValidation, boost::ref(bShooter), _1, _2, ps));
  ps->pathValidations.add("RbprmDynamicPathValidation",
                          boost::bind(&BindShooter::createDynamicPathValidation, boost::ref(bShooter), _1, _2, ps));
  ps->pathPlanners.add("DynamicPlanner", DynamicPlanner::createWithRoadmap);
  ps->steeringMethods.add("RBPRMKinodynamic", SteeringMethodKinodynamic::create);
  ps->pathOptimizers.add("RandomShortcutDynamic", RandomShortcutDynamic::create);
  ps->pathOptimizers.add("OrientedPathOptimizer", OrientedPathOptimizer::create);
  return ps;
}
#endif  // TOOLSOBSTACLE_HH
