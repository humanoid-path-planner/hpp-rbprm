// Copyright (C) 2019 LAAS-CNRS
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


#define BOOST_TEST_MODULE test-kinodynamic
#include <boost/test/included/unit_test.hpp>

#include <hpp/core/problem-solver.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/rbprm/rbprm-device.hh>
#include "tools-fullbody.hh"
#include "tools-obstacle.hh"
#include <hpp/pinocchio/configuration.hh>

using namespace hpp;
using namespace rbprm;


bool checkPathVector(core::PathPtr_t path){
    core::PathVectorPtr_t pv = boost::dynamic_pointer_cast<core::PathVector>(path);
    BOOST_CHECK(pv->numberPaths() > 0);
    if (pv->numberPaths() == 1)
      return true;

    size_type idAcc = path->outputSize() -3; // because acceleration may be set à 0 in init/end config of paths

    core::PathPtr_t previousPath = pv->pathAtRank(0);
    bool successPath;
    if(previousPath->initial().head(idAcc) != (*previousPath)(0.,successPath).head(idAcc)){
      std::cout<<"init config not equal to config at time 0"<<std::endl;
      std::cout<<"init config : "<<pinocchio::displayConfig(previousPath->initial() )<<std::endl;
      std::cout<<"t=0  config : "<<pinocchio::displayConfig((*previousPath)(0.,successPath))<<std::endl;
      return false;
    }
    if(previousPath->end().head(idAcc) != (*previousPath)(previousPath->length(),successPath).head(idAcc)){
      std::cout<<"end config not equal to config at time length()"<<std::endl;
      std::cout<<"end  config : "<<pinocchio::displayConfig(previousPath->end() )<<std::endl;
      std::cout<<"t=l  config : "<<pinocchio::displayConfig((*previousPath)(previousPath->length(),successPath))<<std::endl;
      return false;
    }

    core::PathPtr_t currentPath;
    for(size_t i = 1 ; i < pv->numberPaths() ; ++i){
      currentPath = pv->pathAtRank(i);
      if(previousPath->end().head(idAcc) != currentPath->initial().head(idAcc)){
        std::cout<<"previous path end not equal to current init, for id"<<i<<std::endl;
        std::cout<<"end previous : "<<pinocchio::displayConfig(previousPath->end() )<<std::endl;
        std::cout<<"init current : "<<pinocchio::displayConfig(currentPath->initial())<<std::endl;
        return false;
      }
      if(currentPath->initial().head(idAcc) != (*currentPath)(0.,successPath).head(idAcc)){
        std::cout<<"init config not equal to config at time 0 at index "<<i<<std::endl;
        std::cout<<"init config : "<<pinocchio::displayConfig(currentPath->initial() )<<std::endl;
        std::cout<<"t=0  config : "<<pinocchio::displayConfig((*currentPath)(0.,successPath))<<std::endl;
        return false;
      }
      if(currentPath->end().head(idAcc) != (*currentPath)(currentPath->length(),successPath).head(idAcc)){
        std::cout<<"end config not equal to config at time length() at index "<<i<<std::endl;
        std::cout<<"end  config : "<<pinocchio::displayConfig(currentPath->end() )<<std::endl;
        std::cout<<"t=l  config : "<<pinocchio::displayConfig((*currentPath)(currentPath->length(),successPath))<<std::endl;
        return false;
      }
      previousPath = currentPath;
    }
    return true;
}

bool checkPath(core::PathPtr_t path, double v,double dt = 0.01){
  double t = dt;
  double maxD = v*sqrt(3.)*dt;
  bool successPath;
  vector3_t a = (*path)(0.,successPath).head<3>();
  if (!successPath)
    return false;
  vector3_t b;
  double norm;
  while(t < path->length()){
    b = (*path)(t,successPath).head<3>();
    if (!successPath)
      return false;
    norm = (a-b).norm();
    if(norm > maxD)
      return false;
    t += dt;
    a = b;
  }
  return true;
}



BOOST_AUTO_TEST_SUITE( rbrrt_kinodynamic )

BOOST_AUTO_TEST_CASE (load_abstract_model) {

    hpp::pinocchio::RbPrmDevicePtr_t rbprmDevice = loadTalosLEGAbsract();
    //for(size_t i = 0 ; i < rbprmDevice->data().mass.size() ; ++i)
    //  std::cout<<"mass : "<<i<<" = "<<rbprmDevice->data().mass[i]<<std::endl;
    //BOOST_CHECK_CLOSE(rbprmDevice->mass(),90.27,1e-2); // FIXME : need to investigate and open an issue
    hpp::core::ProblemSolverPtr_t ps = hpp::core::ProblemSolver::create();
    ps->robot(rbprmDevice);
    BOOST_CHECK_CLOSE(rbprmDevice->mass(),90.27,1e-2);
    BOOST_CHECK_CLOSE(ps->robot()->mass(),90.27,1e-2);
    //for(size_t i = 0 ; i < rbprmDevice->data().mass.size() ; ++i)
    //  std::cout<<"mass : "<<i<<" = "<<rbprmDevice->data().mass[i]<<std::endl;
}


BOOST_AUTO_TEST_CASE (straight_line) {
    hpp::pinocchio::RbPrmDevicePtr_t rbprmDevice = loadTalosLEGAbsract();
    rbprmDevice->setDimensionExtraConfigSpace(6);
    BindShooter bShooter;
    std::vector<double> boundsSO3;
    boundsSO3.push_back(-1.7);
    boundsSO3.push_back(1.7);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    bShooter.so3Bounds_ = boundsSO3;
    hpp::core::ProblemSolverPtr_t  ps = configureRbprmProblemSolverForSupportLimbs(rbprmDevice, bShooter);
    hpp::core::ProblemSolver& pSolver = *ps;
    loadObstacleWithAffordance(pSolver, std::string("hpp-rbprm-corba"),
                               std::string("ground"),std::string("planning"));
    // configure planner
    pSolver.addPathOptimizer(std::string("RandomShortcutDynamic"));
    pSolver.configurationShooterType(std::string("RbprmShooter"));
    pSolver.pathValidationType(std::string("RbprmPathValidation"),0.05);
    pSolver.distanceType(std::string("Kinodynamic"));
    pSolver.steeringMethodType(std::string("RBPRMKinodynamic"));
    pSolver.pathPlannerType(std::string("DynamicPlanner"));

    // set problem parameters :
    double aMax = 0.1;
    double vMax = 0.3;
    pSolver.problem()->setParameter(std::string("Kinodynamic/velocityBound"),core::Parameter(vMax));
    pSolver.problem()->setParameter(std::string("Kinodynamic/accelerationBound"),core::Parameter(aMax));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootX"),core::Parameter(0.2));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootY"),core::Parameter(0.12));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/friction"),core::Parameter(0.5));
    pSolver.problem()->setParameter(std::string("ConfigurationShooter/sampleExtraDOF"),core::Parameter(false));

    for(size_type i = 0 ; i < 2 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-vMax;
      rbprmDevice->extraConfigSpace().upper(i)=vMax;
    }
    rbprmDevice->extraConfigSpace().lower(2)=0.;
    rbprmDevice->extraConfigSpace().upper(2)=0.;
    for(size_type i = 3 ; i < 5 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-aMax;
      rbprmDevice->extraConfigSpace().upper(i)=aMax;
    }
    rbprmDevice->extraConfigSpace().lower(5)=0.;
    rbprmDevice->extraConfigSpace().upper(5)=0.;

    // define the planning problem :
    core::Configuration_t q_init(rbprmDevice->configSize());
    q_init << 0, 0, 1.0, 0, 0, 0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    core::Configuration_t q_goal = q_init;
    q_goal(0)=1.5;

    pSolver.initConfig(ConfigurationPtr_t(new core::Configuration_t(q_init)));
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    BOOST_CHECK_CLOSE(pSolver.robot()->mass(),90.27,1e-2);
    bool success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK(checkPathVector(pSolver.paths().back()));
    BOOST_CHECK(checkPath(pSolver.paths().back(),0.5));
    BOOST_CHECK_EQUAL(pSolver.paths().size(),1);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),8.,1e-6);
    core::PathVectorPtr_t pv = boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pv->numberPaths (),1);
    pSolver.solve();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),3);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),8.,1e-6);
    BOOST_CHECK(checkPathVector(pSolver.paths().back()));
    BOOST_CHECK(checkPath(pSolver.paths().back(),0.5));
    pv = boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pv->numberPaths (),1);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),4);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),8.,1e-6);
    BOOST_CHECK(checkPathVector(pSolver.paths().back()));
    BOOST_CHECK(checkPath(pSolver.paths().back(),0.5));
    pv = boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pv->numberPaths (),1);
}


BOOST_AUTO_TEST_CASE (square_v0) {
    hpp::pinocchio::RbPrmDevicePtr_t rbprmDevice = loadTalosLEGAbsract();
    rbprmDevice->setDimensionExtraConfigSpace(6);
    BindShooter bShooter;
    std::vector<double> boundsSO3;
    boundsSO3.push_back(-1.7);
    boundsSO3.push_back(1.7);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    bShooter.so3Bounds_ = boundsSO3;
    hpp::core::ProblemSolverPtr_t  ps = configureRbprmProblemSolverForSupportLimbs(rbprmDevice, bShooter);
    hpp::core::ProblemSolver& pSolver = *ps;
    loadObstacleWithAffordance(pSolver, std::string("hpp-rbprm-corba"),
                               std::string("ground"),std::string("planning"));
    // configure planner
    pSolver.addPathOptimizer(std::string("RandomShortcutDynamic"));
    pSolver.configurationShooterType(std::string("RbprmShooter"));
    pSolver.pathValidationType(std::string("RbprmPathValidation"),0.05);
    pSolver.distanceType(std::string("Kinodynamic"));
    pSolver.steeringMethodType(std::string("RBPRMKinodynamic"));
    pSolver.pathPlannerType(std::string("DynamicPlanner"));

    // set problem parameters :
    double aMax = 0.5;
    double vMax = 1.;
    pSolver.problem()->setParameter(std::string("Kinodynamic/velocityBound"),core::Parameter(vMax));
    pSolver.problem()->setParameter(std::string("Kinodynamic/accelerationBound"),core::Parameter(aMax));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootX"),core::Parameter(0.2));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootY"),core::Parameter(0.12));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/friction"),core::Parameter(0.5));
    pSolver.problem()->setParameter(std::string("ConfigurationShooter/sampleExtraDOF"),core::Parameter(false));

    for(size_type i = 0 ; i < 2 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-vMax;
      rbprmDevice->extraConfigSpace().upper(i)=vMax;
    }
    rbprmDevice->extraConfigSpace().lower(2)=0.;
    rbprmDevice->extraConfigSpace().upper(2)=0.;
    for(size_type i = 3 ; i < 5 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-aMax;
      rbprmDevice->extraConfigSpace().upper(i)=aMax;
    }
    rbprmDevice->extraConfigSpace().lower(5)=0.;
    rbprmDevice->extraConfigSpace().upper(5)=0.;

    // define the planning problem :
    core::Configuration_t q_init(rbprmDevice->configSize());
    q_init << 0, 0, 1.0, 0, 0, 0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    core::Configuration_t q_goal = q_init;
    q_goal(0)=1.;
    q_goal(1)=1.;

    pSolver.initConfig(ConfigurationPtr_t(new core::Configuration_t(q_init)));
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    bool success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),1);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.8284271247461898,1e-10);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),2);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.8284271247461898,1e-10);
    BOOST_CHECK_EQUAL(boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back())->numberPaths (),1);

    pSolver.resetGoalConfigs();
    q_goal(0) = 0.;
    q_goal(1) = 1.;
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),3);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.8284271247461898,1e-10);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),4);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.8284271247461898,1e-10);
    BOOST_CHECK_EQUAL(boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back())->numberPaths (),1);

    pSolver.resetGoalConfigs();
    q_goal(0) = -1.;
    q_goal(1) = 1.;
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),5);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.8284271247461898,1e-10);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),6);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.8284271247461898,1e-10);
    BOOST_CHECK_EQUAL(boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back())->numberPaths (),1);

    pSolver.resetGoalConfigs();
    q_goal(0) = -1.;
    q_goal(1) = 0.;
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),7);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.8284271247461898,1e-10);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),8);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.8284271247461898,1e-10);
    BOOST_CHECK_EQUAL(boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back())->numberPaths (),1);

    pSolver.resetGoalConfigs();
    q_goal(0) = -1.5;
    q_goal(1) = -1.;
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),9);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),3.4641016151377544,1e-10);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),10);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),3.4641016151377544,1e-10);
    BOOST_CHECK_EQUAL(boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back())->numberPaths (),1);

    pSolver.resetGoalConfigs();
    q_goal(0) = 0.;
    q_goal(1) = -1;
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),11);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.8284271247461898,1e-10);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),12);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.8284271247461898,1e-10);
    BOOST_CHECK_EQUAL(boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back())->numberPaths (),1);

    pSolver.resetGoalConfigs();
    q_goal(0) = 1;
    q_goal(1) = -1;
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),13);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.8284271247461898,1e-10);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),14);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.8284271247461898,1e-10);
    BOOST_CHECK_EQUAL(boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back())->numberPaths (),1);

    pSolver.resetGoalConfigs();
    q_goal(0) = 1.3;
    q_goal(1) = -1.2;
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),15);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),3.2249030993194197,1e-10);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),16);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),3.2249030993194197,1e-10);
    BOOST_CHECK_EQUAL(boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back())->numberPaths (),1);

}


BOOST_AUTO_TEST_CASE (straight_velocity) {
    hpp::pinocchio::RbPrmDevicePtr_t rbprmDevice = loadTalosLEGAbsract();
    rbprmDevice->setDimensionExtraConfigSpace(6);
    BindShooter bShooter;
    std::vector<double> boundsSO3;
    boundsSO3.push_back(-1.7);
    boundsSO3.push_back(1.7);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    bShooter.so3Bounds_ = boundsSO3;
    hpp::core::ProblemSolverPtr_t  ps = configureRbprmProblemSolverForSupportLimbs(rbprmDevice, bShooter);
    hpp::core::ProblemSolver& pSolver = *ps;
    loadObstacleWithAffordance(pSolver, std::string("hpp-rbprm-corba"),
                               std::string("ground"),std::string("planning"));
    // configure planner
    pSolver.addPathOptimizer(std::string("RandomShortcutDynamic"));
    pSolver.configurationShooterType(std::string("RbprmShooter"));
    pSolver.pathValidationType(std::string("RbprmPathValidation"),0.05);
    pSolver.distanceType(std::string("Kinodynamic"));
    pSolver.steeringMethodType(std::string("RBPRMKinodynamic"));
    pSolver.pathPlannerType(std::string("DynamicPlanner"));

    // set problem parameters :
    double aMax = 0.5;
    double vMax = 1.5;
    pSolver.problem()->setParameter(std::string("Kinodynamic/velocityBound"),core::Parameter(vMax));
    pSolver.problem()->setParameter(std::string("Kinodynamic/accelerationBound"),core::Parameter(aMax));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootX"),core::Parameter(0.2));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootY"),core::Parameter(0.12));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/friction"),core::Parameter(0.5));
    pSolver.problem()->setParameter(std::string("ConfigurationShooter/sampleExtraDOF"),core::Parameter(false));

    for(size_type i = 0 ; i < 2 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-vMax;
      rbprmDevice->extraConfigSpace().upper(i)=vMax;
    }
    rbprmDevice->extraConfigSpace().lower(2)=0.;
    rbprmDevice->extraConfigSpace().upper(2)=0.;
    for(size_type i = 3 ; i < 5 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-aMax;
      rbprmDevice->extraConfigSpace().upper(i)=aMax;
    }
    rbprmDevice->extraConfigSpace().lower(5)=0.;
    rbprmDevice->extraConfigSpace().upper(5)=0.;

    // define the planning problem :
    core::Configuration_t q_init(rbprmDevice->configSize());
    q_init << 0, 0, 1.0, 0, 0, 0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    q_init(9) = 0.5; // init velocity along x
    core::Configuration_t q_goal = q_init;
    q_goal(0)=1.;


    pSolver.initConfig(ConfigurationPtr_t(new core::Configuration_t(q_init)));
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    bool success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),1);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),1.4641016151377546,1e-10);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),2);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),1.4641016151377546,1e-10);
    BOOST_CHECK(checkPathVector(pSolver.paths().back()));
    BOOST_CHECK(checkPath(pSolver.paths().back(),1.7));
    BOOST_CHECK_EQUAL(boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back())->numberPaths (),1);

    pSolver.resetGoalConfigs();
    q_goal(0) = 0.;
    q_goal(9) = 0.1;
    q_goal(10) = -0.2; // final velocity
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),3);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.642220510185596,1e-10);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),4);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.642220510185596,1e-10);
    BOOST_CHECK(checkPathVector(pSolver.paths().back()));
    BOOST_CHECK(checkPath(pSolver.paths().back(),1.7));
    BOOST_CHECK_EQUAL(boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back())->numberPaths (),1);

    pSolver.resetGoalConfigs();
    q_goal(0) = 1.;
    q_goal(9) = -0.3;
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),5);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.8741411087489799,1e-10);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),6);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.8741411087489799,1e-10);
    BOOST_CHECK(checkPathVector(pSolver.paths().back()));
    BOOST_CHECK(checkPath(pSolver.paths().back(),1.7));
    BOOST_CHECK_EQUAL(boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back())->numberPaths (),1);

    pSolver.resetGoalConfigs();
    q_goal(0) = -1.;
    q_goal(9) = 0.;
    q_goal(10) = 0.3;
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),7);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),4.16227766016838,1e-10);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),8);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),4.16227766016838,1e-10);
    BOOST_CHECK(checkPathVector(pSolver.paths().back()));
    BOOST_CHECK(checkPath(pSolver.paths().back(),1.7));
    BOOST_CHECK_EQUAL(boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back())->numberPaths (),1);


}



BOOST_AUTO_TEST_CASE (straight_line_amax_mu05) {
    hpp::pinocchio::RbPrmDevicePtr_t rbprmDevice = loadTalosLEGAbsract();
    rbprmDevice->setDimensionExtraConfigSpace(6);
    BindShooter bShooter;
    std::vector<double> boundsSO3;
    boundsSO3.push_back(-1.7);
    boundsSO3.push_back(1.7);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    bShooter.so3Bounds_ = boundsSO3;
    hpp::core::ProblemSolverPtr_t  ps = configureRbprmProblemSolverForSupportLimbs(rbprmDevice, bShooter);
    hpp::core::ProblemSolver& pSolver = *ps;
    loadObstacleWithAffordance(pSolver, std::string("hpp-rbprm-corba"),
                               std::string("ground"),std::string("planning"));
    // configure planner
    pSolver.addPathOptimizer(std::string("RandomShortcutDynamic"));
    pSolver.configurationShooterType(std::string("RbprmShooter"));
    pSolver.pathValidationType(std::string("RbprmPathValidation"),0.05);
    pSolver.distanceType(std::string("Kinodynamic"));
    pSolver.steeringMethodType(std::string("RBPRMKinodynamic"));
    pSolver.pathPlannerType(std::string("DynamicPlanner"));

    // set problem parameters :
    double aMax = 10.;
    double vMax = 1.;
    pSolver.problem()->setParameter(std::string("Kinodynamic/velocityBound"),core::Parameter(vMax));
    pSolver.problem()->setParameter(std::string("Kinodynamic/accelerationBound"),core::Parameter(aMax));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootX"),core::Parameter(0.2));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootY"),core::Parameter(0.12));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/friction"),core::Parameter(0.5));
    pSolver.problem()->setParameter(std::string("ConfigurationShooter/sampleExtraDOF"),core::Parameter(false));
    vector3_t p_lLeg(0., 0.0848172440888579,-1.019272022956703);
    vector3_t p_rLeg(0., -0.0848172440888579,-1.019272022956703);
    rbprmDevice->setEffectorReference("talos_lleg_rom",p_lLeg);
    rbprmDevice->setEffectorReference("talos_rleg_rom",p_rLeg);

    for(size_type i = 0 ; i < 2 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-vMax;
      rbprmDevice->extraConfigSpace().upper(i)=vMax;
    }
    rbprmDevice->extraConfigSpace().lower(2)=0.;
    rbprmDevice->extraConfigSpace().upper(2)=0.;
    for(size_type i = 3 ; i < 5 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-aMax;
      rbprmDevice->extraConfigSpace().upper(i)=aMax;
    }
    rbprmDevice->extraConfigSpace().lower(5)=0.;
    rbprmDevice->extraConfigSpace().upper(5)=0.;

    // define the planning problem :
    core::Configuration_t q_init(rbprmDevice->configSize());
    q_init << 0, 0, 1.0, 0, 0, 0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    core::Configuration_t q_goal = q_init;
    q_goal(0)=1.5;

    pSolver.initConfig(ConfigurationPtr_t(new core::Configuration_t(q_init)));
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    BOOST_CHECK_CLOSE(pSolver.robot()->mass(),90.27,1e-2);
    bool success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),1);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.5298682146914024,1e-6);
}


BOOST_AUTO_TEST_CASE (straight_line_amax_mu005) {
    hpp::pinocchio::RbPrmDevicePtr_t rbprmDevice = loadTalosLEGAbsract();
    rbprmDevice->setDimensionExtraConfigSpace(6);
    BindShooter bShooter;
    std::vector<double> boundsSO3;
    boundsSO3.push_back(-1.7);
    boundsSO3.push_back(1.7);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    bShooter.so3Bounds_ = boundsSO3;
    hpp::core::ProblemSolverPtr_t  ps = configureRbprmProblemSolverForSupportLimbs(rbprmDevice, bShooter);
    hpp::core::ProblemSolver& pSolver = *ps;
    loadObstacleWithAffordance(pSolver, std::string("hpp-rbprm-corba"),
                               std::string("ground"),std::string("planning"));
    // configure planner
    pSolver.addPathOptimizer(std::string("RandomShortcutDynamic"));
    pSolver.configurationShooterType(std::string("RbprmShooter"));
    pSolver.pathValidationType(std::string("RbprmPathValidation"),0.05);
    pSolver.distanceType(std::string("Kinodynamic"));
    pSolver.steeringMethodType(std::string("RBPRMKinodynamic"));
    pSolver.pathPlannerType(std::string("DynamicPlanner"));

    // set problem parameters :
    double aMax = 10.;
    double vMax = 1.;
    pSolver.problem()->setParameter(std::string("Kinodynamic/velocityBound"),core::Parameter(vMax));
    pSolver.problem()->setParameter(std::string("Kinodynamic/accelerationBound"),core::Parameter(aMax));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootX"),core::Parameter(0.2));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootY"),core::Parameter(0.12));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/friction"),core::Parameter(0.05));
    pSolver.problem()->setParameter(std::string("ConfigurationShooter/sampleExtraDOF"),core::Parameter(false));
    vector3_t p_lLeg(0., 0.0848172440888579,-1.019272022956703);
    vector3_t p_rLeg(0., -0.0848172440888579,-1.019272022956703);
    rbprmDevice->setEffectorReference("talos_lleg_rom",p_lLeg);
    rbprmDevice->setEffectorReference("talos_rleg_rom",p_rLeg);

    for(size_type i = 0 ; i < 2 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-vMax;
      rbprmDevice->extraConfigSpace().upper(i)=vMax;
    }
    rbprmDevice->extraConfigSpace().lower(2)=0.;
    rbprmDevice->extraConfigSpace().upper(2)=0.;
    for(size_type i = 3 ; i < 5 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-aMax;
      rbprmDevice->extraConfigSpace().upper(i)=aMax;
    }
    rbprmDevice->extraConfigSpace().lower(5)=0.;
    rbprmDevice->extraConfigSpace().upper(5)=0.;

    // define the planning problem :
    core::Configuration_t q_init(rbprmDevice->configSize());
    q_init << 0, 0, 1.0, 0, 0, 0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    core::Configuration_t q_goal = q_init;
    q_goal(0)=1.5;

    pSolver.initConfig(ConfigurationPtr_t(new core::Configuration_t(q_init)));
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    BOOST_CHECK_CLOSE(pSolver.robot()->mass(),90.27,1e-2);
    bool success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),1);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),3.5337445642816432,1e-6);
}

BOOST_AUTO_TEST_CASE (straight_line_amax_mu001) {
    hpp::pinocchio::RbPrmDevicePtr_t rbprmDevice = loadTalosLEGAbsract();
    rbprmDevice->setDimensionExtraConfigSpace(6);
    BindShooter bShooter;
    std::vector<double> boundsSO3;
    boundsSO3.push_back(-1.7);
    boundsSO3.push_back(1.7);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    bShooter.so3Bounds_ = boundsSO3;
    hpp::core::ProblemSolverPtr_t  ps = configureRbprmProblemSolverForSupportLimbs(rbprmDevice, bShooter);
    hpp::core::ProblemSolver& pSolver = *ps;
    loadObstacleWithAffordance(pSolver, std::string("hpp-rbprm-corba"),
                               std::string("ground"),std::string("planning"));
    // configure planner
    pSolver.addPathOptimizer(std::string("RandomShortcutDynamic"));
    pSolver.configurationShooterType(std::string("RbprmShooter"));
    pSolver.pathValidationType(std::string("RbprmPathValidation"),0.05);
    pSolver.distanceType(std::string("Kinodynamic"));
    pSolver.steeringMethodType(std::string("RBPRMKinodynamic"));
    pSolver.pathPlannerType(std::string("DynamicPlanner"));

    // set problem parameters :
    double aMax = 10.;
    double vMax = 1.;
    pSolver.problem()->setParameter(std::string("Kinodynamic/velocityBound"),core::Parameter(vMax));
    pSolver.problem()->setParameter(std::string("Kinodynamic/accelerationBound"),core::Parameter(aMax));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootX"),core::Parameter(0.2));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootY"),core::Parameter(0.12));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/friction"),core::Parameter(0.01));
    pSolver.problem()->setParameter(std::string("ConfigurationShooter/sampleExtraDOF"),core::Parameter(false));
    vector3_t p_lLeg(0., 0.0848172440888579,-1.019272022956703);
    vector3_t p_rLeg(0., -0.0848172440888579,-1.019272022956703);
    rbprmDevice->setEffectorReference("talos_lleg_rom",p_lLeg);
    rbprmDevice->setEffectorReference("talos_rleg_rom",p_rLeg);

    for(size_type i = 0 ; i < 2 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-vMax;
      rbprmDevice->extraConfigSpace().upper(i)=vMax;
    }
    rbprmDevice->extraConfigSpace().lower(2)=0.;
    rbprmDevice->extraConfigSpace().upper(2)=0.;
    for(size_type i = 3 ; i < 5 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-aMax;
      rbprmDevice->extraConfigSpace().upper(i)=aMax;
    }
    rbprmDevice->extraConfigSpace().lower(5)=0.;
    rbprmDevice->extraConfigSpace().upper(5)=0.;

    // define the planning problem :
    core::Configuration_t q_init(rbprmDevice->configSize());
    q_init << 0, 0, 1.0, 0, 0, 0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    core::Configuration_t q_goal = q_init;
    q_goal(0)=1.5;

    pSolver.initConfig(ConfigurationPtr_t(new core::Configuration_t(q_init)));
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    BOOST_CHECK_CLOSE(pSolver.robot()->mass(),90.27,1e-2);
    bool success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),1);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),8.253186224036595,1e-6);
}

BOOST_AUTO_TEST_CASE (straight_line_amax_mu5) {
    hpp::pinocchio::RbPrmDevicePtr_t rbprmDevice = loadTalosLEGAbsract();
    rbprmDevice->setDimensionExtraConfigSpace(6);
    BindShooter bShooter;
    std::vector<double> boundsSO3;
    boundsSO3.push_back(-1.7);
    boundsSO3.push_back(1.7);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    bShooter.so3Bounds_ = boundsSO3;
    hpp::core::ProblemSolverPtr_t  ps = configureRbprmProblemSolverForSupportLimbs(rbprmDevice, bShooter);
    hpp::core::ProblemSolver& pSolver = *ps;
    loadObstacleWithAffordance(pSolver, std::string("hpp-rbprm-corba"),
                               std::string("ground"),std::string("planning"));
    // configure planner
    pSolver.addPathOptimizer(std::string("RandomShortcutDynamic"));
    pSolver.configurationShooterType(std::string("RbprmShooter"));
    pSolver.pathValidationType(std::string("RbprmPathValidation"),0.05);
    pSolver.distanceType(std::string("Kinodynamic"));
    pSolver.steeringMethodType(std::string("RBPRMKinodynamic"));
    pSolver.pathPlannerType(std::string("DynamicPlanner"));

    // set problem parameters :
    double aMax = 10.;
    double vMax = 1.;
    pSolver.problem()->setParameter(std::string("Kinodynamic/velocityBound"),core::Parameter(vMax));
    pSolver.problem()->setParameter(std::string("Kinodynamic/accelerationBound"),core::Parameter(aMax));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootX"),core::Parameter(0.2));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootY"),core::Parameter(0.12));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/friction"),core::Parameter(5.));
    pSolver.problem()->setParameter(std::string("ConfigurationShooter/sampleExtraDOF"),core::Parameter(false));
    vector3_t p_lLeg(0., 0.0848172440888579,-1.019272022956703);
    vector3_t p_rLeg(0., -0.0848172440888579,-1.019272022956703);
    rbprmDevice->setEffectorReference("talos_lleg_rom",p_lLeg);
    rbprmDevice->setEffectorReference("talos_rleg_rom",p_rLeg);

    for(size_type i = 0 ; i < 2 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-vMax;
      rbprmDevice->extraConfigSpace().upper(i)=vMax;
    }
    rbprmDevice->extraConfigSpace().lower(2)=0.;
    rbprmDevice->extraConfigSpace().upper(2)=0.;
    for(size_type i = 3 ; i < 5 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-aMax;
      rbprmDevice->extraConfigSpace().upper(i)=aMax;
    }
    rbprmDevice->extraConfigSpace().lower(5)=0.;
    rbprmDevice->extraConfigSpace().upper(5)=0.;

    // define the planning problem :
    core::Configuration_t q_init(rbprmDevice->configSize());
    q_init << 0, 0, 1.0, 0, 0, 0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    core::Configuration_t q_goal = q_init;
    q_goal(0)=1.5;

    pSolver.initConfig(ConfigurationPtr_t(new core::Configuration_t(q_init)));
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    BOOST_CHECK_CLOSE(pSolver.robot()->mass(),90.27,1e-2);
    bool success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),1);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),2.5298683485627125,1e-6);
}

BOOST_AUTO_TEST_CASE (straight_line_amax_feetChange) {
    hpp::pinocchio::RbPrmDevicePtr_t rbprmDevice = loadTalosLEGAbsract();
    rbprmDevice->setDimensionExtraConfigSpace(6);
    BindShooter bShooter;
    std::vector<double> boundsSO3;
    boundsSO3.push_back(-1.7);
    boundsSO3.push_back(1.7);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    bShooter.so3Bounds_ = boundsSO3;
    hpp::core::ProblemSolverPtr_t  ps = configureRbprmProblemSolverForSupportLimbs(rbprmDevice, bShooter);
    hpp::core::ProblemSolver& pSolver = *ps;
    loadObstacleWithAffordance(pSolver, std::string("hpp-rbprm-corba"),
                               std::string("ground"),std::string("planning"));
    // configure planner
    pSolver.addPathOptimizer(std::string("RandomShortcutDynamic"));
    pSolver.configurationShooterType(std::string("RbprmShooter"));
    pSolver.pathValidationType(std::string("RbprmPathValidation"),0.05);
    pSolver.distanceType(std::string("Kinodynamic"));
    pSolver.steeringMethodType(std::string("RBPRMKinodynamic"));
    pSolver.pathPlannerType(std::string("DynamicPlanner"));

    // set problem parameters :
    double aMax = 10.;
    double vMax = 1.;
    pSolver.problem()->setParameter(std::string("Kinodynamic/velocityBound"),core::Parameter(vMax));
    pSolver.problem()->setParameter(std::string("Kinodynamic/accelerationBound"),core::Parameter(aMax));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootX"),core::Parameter(0.05));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootY"),core::Parameter(0.05));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/friction"),core::Parameter(0.5));
    pSolver.problem()->setParameter(std::string("ConfigurationShooter/sampleExtraDOF"),core::Parameter(false));
    vector3_t p_lLeg(0., 0.0848172440888579,-1.019272022956703);
    vector3_t p_rLeg(0., -0.0848172440888579,-1.019272022956703);
    rbprmDevice->setEffectorReference("talos_lleg_rom",p_lLeg);
    rbprmDevice->setEffectorReference("talos_rleg_rom",p_rLeg);

    for(size_type i = 0 ; i < 2 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-vMax;
      rbprmDevice->extraConfigSpace().upper(i)=vMax;
    }
    rbprmDevice->extraConfigSpace().lower(2)=0.;
    rbprmDevice->extraConfigSpace().upper(2)=0.;
    for(size_type i = 3 ; i < 5 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-aMax;
      rbprmDevice->extraConfigSpace().upper(i)=aMax;
    }
    rbprmDevice->extraConfigSpace().lower(5)=0.;
    rbprmDevice->extraConfigSpace().upper(5)=0.;

    // define the planning problem :
    core::Configuration_t q_init(rbprmDevice->configSize());
    q_init << 0, 0, 1.0, 0, 0, 0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    core::Configuration_t q_goal = q_init;
    q_goal(0)=1.5;

    pSolver.initConfig(ConfigurationPtr_t(new core::Configuration_t(q_init)));
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    BOOST_CHECK_CLOSE(pSolver.robot()->mass(),90.27,1e-2);
    bool success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),1);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),5.0502312883507487,1e-6);
}


BOOST_AUTO_TEST_CASE (nav_bauzil) {
    std::cout<<"start nav_bauzil test case, this may take a couple of minutes ..."<<std::endl;
  // this test case may take up to a minute to execute. Usually after ~5 minutes it should be considered as a failure.
    hpp::pinocchio::RbPrmDevicePtr_t rbprmDevice = loadTalosLEGAbsract();
    rbprmDevice->setDimensionExtraConfigSpace(6);
    BindShooter bShooter;
    std::vector<double> boundsSO3;
    boundsSO3.push_back(-4);
    boundsSO3.push_back(4);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    bShooter.so3Bounds_ = boundsSO3;
    hpp::core::ProblemSolverPtr_t  ps = configureRbprmProblemSolverForSupportLimbs(rbprmDevice, bShooter);
    hpp::core::ProblemSolver& pSolver = *ps;
    loadObstacleWithAffordance(pSolver, std::string("hpp-rbprm-corba"),
                               std::string("floor_bauzil"),std::string("planning"));
    // configure planner
    pSolver.addPathOptimizer(std::string("RandomShortcutDynamic"));
    pSolver.configurationShooterType(std::string("RbprmShooter"));
    pSolver.pathValidationType(std::string("RbprmPathValidation"),0.05);
    pSolver.distanceType(std::string("Kinodynamic"));
    pSolver.steeringMethodType(std::string("RBPRMKinodynamic"));
    pSolver.pathPlannerType(std::string("DynamicPlanner"));

    // set problem parameters :
    double aMax = 0.1;
    double vMax = 0.3;
    pSolver.problem()->setParameter(std::string("Kinodynamic/velocityBound"),core::Parameter(vMax));
    pSolver.problem()->setParameter(std::string("Kinodynamic/accelerationBound"),core::Parameter(aMax));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootX"),core::Parameter(0.2));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootY"),core::Parameter(0.12));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/friction"),core::Parameter(0.5));
    pSolver.problem()->setParameter(std::string("ConfigurationShooter/sampleExtraDOF"),core::Parameter(false));
    pSolver.problem()->setParameter(std::string("PathOptimization/RandomShortcut/NumberOfLoops"),core::Parameter((core::size_type)50));
    vector3_t p_lLeg(0., 0.0848172440888579,-1.019272022956703);
    vector3_t p_rLeg(0., -0.0848172440888579,-1.019272022956703);
    rbprmDevice->setEffectorReference("talos_lleg_rom",p_lLeg);
    rbprmDevice->setEffectorReference("talos_rleg_rom",p_rLeg);


    for(size_type i = 0 ; i < 2 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-vMax;
      rbprmDevice->extraConfigSpace().upper(i)=vMax;
    }
    rbprmDevice->extraConfigSpace().lower(2)=0.;
    rbprmDevice->extraConfigSpace().upper(2)=0.;
    for(size_type i = 3 ; i < 5 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-aMax;
      rbprmDevice->extraConfigSpace().upper(i)=aMax;
    }
    rbprmDevice->extraConfigSpace().lower(5)=0.;
    rbprmDevice->extraConfigSpace().upper(5)=0.;

    // define the planning problem :
    core::Configuration_t q_init(rbprmDevice->configSize());
    q_init << -0.9, 1.5, 0.98, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.07, 0, 0, 0.0, 0.0, 0.0;
    core::Configuration_t q_goal(rbprmDevice->configSize());
    q_goal << 2, 2.6, 0.98, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1, 0, 0, 0.0, 0.0, 0.0;

    pSolver.initConfig(ConfigurationPtr_t(new core::Configuration_t(q_init)));
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    BOOST_CHECK_CLOSE(pSolver.robot()->mass(),90.27,1e-2);

    pSolver.solve();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),2);
    std::cout<<"Solve complete, start optimization. This may take few minutes ..."<<std::endl;
    BOOST_CHECK(checkPathVector(pSolver.paths().back()));
    BOOST_CHECK(checkPath(pSolver.paths().back(),0.5));
    for(size_t i = 0 ; i < 10 ; ++i){
      pSolver.optimizePath(pSolver.paths().back());
      BOOST_CHECK_EQUAL(pSolver.paths().size(),3+i);
      BOOST_CHECK(checkPathVector(pSolver.paths().back()));
      BOOST_CHECK(checkPath(pSolver.paths().back(),0.5));
      std::cout<<"("<<i+1<<"/10)  "<<std::flush;
    }
    std::cout<<std::endl;

}


BOOST_AUTO_TEST_CASE (nav_bauzil_oriented) {
    std::cout<<"start nav_bauzil_oriented test case, this may take a couple of minutes ..."<<std::endl;
  // this test case may take up to a minute to execute. Usually after ~5 minutes it should be considered as a failure.
    hpp::pinocchio::RbPrmDevicePtr_t rbprmDevice = loadTalosLEGAbsract();
    rbprmDevice->setDimensionExtraConfigSpace(6);
    BindShooter bShooter;
    std::vector<double> boundsSO3;
    boundsSO3.push_back(-4);
    boundsSO3.push_back(4);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    bShooter.so3Bounds_ = boundsSO3;
    hpp::core::ProblemSolverPtr_t  ps = configureRbprmProblemSolverForSupportLimbs(rbprmDevice, bShooter);
    hpp::core::ProblemSolver& pSolver = *ps;
    loadObstacleWithAffordance(pSolver, std::string("hpp-rbprm-corba"),
                               std::string("floor_bauzil"),std::string("planning"));
    // configure planner
    pSolver.addPathOptimizer(std::string("RandomShortcutDynamic"));
    pSolver.configurationShooterType(std::string("RbprmShooter"));
    pSolver.pathValidationType(std::string("RbprmPathValidation"),0.05);
    pSolver.distanceType(std::string("Kinodynamic"));
    pSolver.steeringMethodType(std::string("RBPRMKinodynamic"));
    pSolver.pathPlannerType(std::string("DynamicPlanner"));

    // set problem parameters :
    double aMax = 0.1;
    double vMax = 0.3;
    pSolver.problem()->setParameter(std::string("Kinodynamic/velocityBound"),core::Parameter(vMax));
    pSolver.problem()->setParameter(std::string("Kinodynamic/accelerationBound"),core::Parameter(aMax));
    pSolver.problem()->setParameter(std::string("Kinodynamic/forceOrientation"),core::Parameter(true));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootX"),core::Parameter(0.2));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootY"),core::Parameter(0.12));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/friction"),core::Parameter(0.5));
    pSolver.problem()->setParameter(std::string("ConfigurationShooter/sampleExtraDOF"),core::Parameter(false));
    pSolver.problem()->setParameter(std::string("PathOptimization/RandomShortcut/NumberOfLoops"),core::Parameter((core::size_type)50));
    vector3_t p_lLeg(0., 0.0848172440888579,-1.019272022956703);
    vector3_t p_rLeg(0., -0.0848172440888579,-1.019272022956703);
    rbprmDevice->setEffectorReference("talos_lleg_rom",p_lLeg);
    rbprmDevice->setEffectorReference("talos_rleg_rom",p_rLeg);


    for(size_type i = 0 ; i < 2 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-vMax;
      rbprmDevice->extraConfigSpace().upper(i)=vMax;
    }
    rbprmDevice->extraConfigSpace().lower(2)=0.;
    rbprmDevice->extraConfigSpace().upper(2)=0.;
    for(size_type i = 3 ; i < 5 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-aMax;
      rbprmDevice->extraConfigSpace().upper(i)=aMax;
    }
    rbprmDevice->extraConfigSpace().lower(5)=0.;
    rbprmDevice->extraConfigSpace().upper(5)=0.;

    // define the planning problem :
    core::Configuration_t q_init(rbprmDevice->configSize());
    q_init << -0.9, 1.5, 0.98, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.07, 0, 0, 0.0, 0.0, 0.0;
    core::Configuration_t q_goal(rbprmDevice->configSize());
    q_goal << 2, 2.6, 0.98, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1, 0, 0, 0.0, 0.0, 0.0;

    pSolver.initConfig(ConfigurationPtr_t(new core::Configuration_t(q_init)));
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    BOOST_CHECK_CLOSE(pSolver.robot()->mass(),90.27,1e-2);

    pSolver.solve();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),2);
    std::cout<<"Solve complete, start optimization. This may take few minutes ..."<<std::endl;
    BOOST_CHECK(checkPathVector(pSolver.paths().back()));
    BOOST_CHECK(checkPath(pSolver.paths().back(),0.5));
    for(size_t i = 0 ; i < 10 ; ++i){
      pSolver.optimizePath(pSolver.paths().back());
      BOOST_CHECK_EQUAL(pSolver.paths().size(),3+i);
      BOOST_CHECK(checkPathVector(pSolver.paths().back()));
      BOOST_CHECK(checkPath(pSolver.paths().back(),0.5));
      std::cout<<"("<<i+1<<"/10)  "<<std::flush;
    }
    std::cout<<std::endl;

}


BOOST_AUTO_TEST_CASE (nav_bauzil_oriented_kino) {
    std::cout<<"start nav_bauzil_oriented_kino test case, this may take a couple of minutes ..."<<std::endl;
  // this test case may take up to a minute to execute. Usually after ~5 minutes it should be considered as a failure.
    hpp::pinocchio::RbPrmDevicePtr_t rbprmDevice = loadTalosLEGAbsract();
    rbprmDevice->setDimensionExtraConfigSpace(6);
    BindShooter bShooter;
    std::vector<double> boundsSO3;
    boundsSO3.push_back(-4);
    boundsSO3.push_back(4);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    bShooter.so3Bounds_ = boundsSO3;
    hpp::core::ProblemSolverPtr_t  ps = configureRbprmProblemSolverForSupportLimbs(rbprmDevice, bShooter);
    hpp::core::ProblemSolver& pSolver = *ps;
    loadObstacleWithAffordance(pSolver, std::string("hpp-rbprm-corba"),
                               std::string("floor_bauzil"),std::string("planning"));
    // configure planner
    pSolver.addPathOptimizer(std::string("RandomShortcutDynamic"));
    pSolver.configurationShooterType(std::string("RbprmShooter"));
    pSolver.pathValidationType(std::string("RbprmPathValidation"),0.05);
    pSolver.distanceType(std::string("Kinodynamic"));
    pSolver.steeringMethodType(std::string("RBPRMKinodynamic"));
    pSolver.pathPlannerType(std::string("DynamicPlanner"));

    // set problem parameters :
    double aMax = 0.1;
    double vMax = 0.3;
    pSolver.problem()->setParameter(std::string("Kinodynamic/velocityBound"),core::Parameter(vMax));
    pSolver.problem()->setParameter(std::string("Kinodynamic/accelerationBound"),core::Parameter(aMax));
    pSolver.problem()->setParameter(std::string("Kinodynamic/forceOrientation"),core::Parameter(true));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootX"),core::Parameter(0.2));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootY"),core::Parameter(0.12));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/friction"),core::Parameter(0.5));
    pSolver.problem()->setParameter(std::string("ConfigurationShooter/sampleExtraDOF"),core::Parameter(true));
    pSolver.problem()->setParameter(std::string("PathOptimization/RandomShortcut/NumberOfLoops"),core::Parameter((core::size_type)50));
    vector3_t p_lLeg(0., 0.0848172440888579,-1.019272022956703);
    vector3_t p_rLeg(0., -0.0848172440888579,-1.019272022956703);
    rbprmDevice->setEffectorReference("talos_lleg_rom",p_lLeg);
    rbprmDevice->setEffectorReference("talos_rleg_rom",p_rLeg);


    for(size_type i = 0 ; i < 2 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-vMax;
      rbprmDevice->extraConfigSpace().upper(i)=vMax;
    }
    rbprmDevice->extraConfigSpace().lower(2)=0.;
    rbprmDevice->extraConfigSpace().upper(2)=0.;
    for(size_type i = 3 ; i < 5 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-aMax;
      rbprmDevice->extraConfigSpace().upper(i)=aMax;
    }
    rbprmDevice->extraConfigSpace().lower(5)=0.;
    rbprmDevice->extraConfigSpace().upper(5)=0.;

    // define the planning problem :
    core::Configuration_t q_init(rbprmDevice->configSize());
    q_init << -0.9, 1.5, 0.98, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.07, 0, 0, 0.0, 0.0, 0.0;
    core::Configuration_t q_goal(rbprmDevice->configSize());
    q_goal << 2, 2.6, 0.98, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1, 0, 0, 0.0, 0.0, 0.0;

    pSolver.initConfig(ConfigurationPtr_t(new core::Configuration_t(q_init)));
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    BOOST_CHECK_CLOSE(pSolver.robot()->mass(),90.27,1e-2);

    pSolver.solve();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),2);
    std::cout<<"Solve complete, start optimization. This may take few minutes ..."<<std::endl;
    BOOST_CHECK(checkPathVector(pSolver.paths().back()));
    BOOST_CHECK(checkPath(pSolver.paths().back(),0.5));
    for(size_t i = 0 ; i < 10 ; ++i){
      pSolver.optimizePath(pSolver.paths().back());
      BOOST_CHECK_EQUAL(pSolver.paths().size(),3+i);
      BOOST_CHECK(checkPathVector(pSolver.paths().back()));
      BOOST_CHECK(checkPath(pSolver.paths().back(),0.5));
      std::cout<<"("<<i+1<<"/10)  "<<std::flush;
    }
    std::cout<<std::endl;
}



// too slow to be added in the test suite ...
BOOST_AUTO_TEST_CASE (nav_bauzil_hard) {
    std::cout<<"start nav_bauzil_hard test case, this may take several minutes ..."<<std::endl;
  // this test case may take up to 5 minute to execute. Usually after ~10 minutes it should be considered as a failure.
    hpp::pinocchio::RbPrmDevicePtr_t rbprmDevice = loadTalosLEGAbsract();
    rbprmDevice->rootJoint()->lowerBound(0, -2.3);
    rbprmDevice->rootJoint()->lowerBound(1, -1.5);
    rbprmDevice->rootJoint()->lowerBound(2, 0.98);
    rbprmDevice->rootJoint()->upperBound(0,  4.6);
    rbprmDevice->rootJoint()->upperBound(1,  3.3);
    rbprmDevice->rootJoint()->upperBound(2,  0.98);
    rbprmDevice->setDimensionExtraConfigSpace(6);
    BindShooter bShooter;
    std::vector<double> boundsSO3;
    boundsSO3.push_back(-4);
    boundsSO3.push_back(4);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    boundsSO3.push_back(-0.1);
    boundsSO3.push_back(0.1);
    bShooter.so3Bounds_ = boundsSO3;
    hpp::core::ProblemSolverPtr_t  ps = configureRbprmProblemSolverForSupportLimbs(rbprmDevice, bShooter);
    hpp::core::ProblemSolver& pSolver = *ps;
    loadObstacleWithAffordance(pSolver, std::string("hpp-rbprm-corba"),
                               std::string("floor_bauzil"),std::string("planning"));
    // configure planner
    pSolver.addPathOptimizer(std::string("RandomShortcutDynamic"));
    pSolver.configurationShooterType(std::string("RbprmShooter"));
    pSolver.pathValidationType(std::string("RbprmPathValidation"),0.05);
    pSolver.distanceType(std::string("Kinodynamic"));
    pSolver.steeringMethodType(std::string("RBPRMKinodynamic"));
    pSolver.pathPlannerType(std::string("DynamicPlanner"));

    // set problem parameters :
    double aMax = 0.1;
    double vMax = 0.3;
    pSolver.problem()->setParameter(std::string("Kinodynamic/velocityBound"),core::Parameter(vMax));
    pSolver.problem()->setParameter(std::string("Kinodynamic/accelerationBound"),core::Parameter(aMax));
    pSolver.problem()->setParameter(std::string("Kinodynamic/forceOrientation"),core::Parameter(true));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootX"),core::Parameter(0.2));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/sizeFootY"),core::Parameter(0.12));
    pSolver.problem()->setParameter(std::string("DynamicPlanner/friction"),core::Parameter(0.5));
    pSolver.problem()->setParameter(std::string("ConfigurationShooter/sampleExtraDOF"),core::Parameter(false));
    pSolver.problem()->setParameter(std::string("PathOptimization/RandomShortcut/NumberOfLoops"),core::Parameter((core::size_type)10));
    vector3_t p_lLeg(0., 0.0848172440888579,-1.019272022956703);
    vector3_t p_rLeg(0., -0.0848172440888579,-1.019272022956703);
    rbprmDevice->setEffectorReference("talos_lleg_rom",p_lLeg);
    rbprmDevice->setEffectorReference("talos_rleg_rom",p_rLeg);


    for(size_type i = 0 ; i < 2 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-vMax;
      rbprmDevice->extraConfigSpace().upper(i)=vMax;
    }
    rbprmDevice->extraConfigSpace().lower(2)=0.;
    rbprmDevice->extraConfigSpace().upper(2)=0.;
    for(size_type i = 3 ; i < 5 ; ++i){
      rbprmDevice->extraConfigSpace().lower(i)=-aMax;
      rbprmDevice->extraConfigSpace().upper(i)=aMax;
    }
    rbprmDevice->extraConfigSpace().lower(5)=0.;
    rbprmDevice->extraConfigSpace().upper(5)=0.;

    // define the planning problem :
    core::Configuration_t q_init(rbprmDevice->configSize());
    q_init << -0.7, 2., 0.98, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.07, 0, 0, 0.0, 0.0, 0.0;
    core::Configuration_t q_goal(rbprmDevice->configSize());
    q_goal << 0., -1., 0.98, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1, 0, 0, 0.0, 0.0, 0.0;

    pSolver.initConfig(ConfigurationPtr_t(new core::Configuration_t(q_init)));
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    BOOST_CHECK_CLOSE(pSolver.robot()->mass(),90.27,1e-2);

    pSolver.solve();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),2);
    std::cout<<"Solve complete, start optimization. This may take few minutes ..."<<std::endl;
    BOOST_CHECK(checkPathVector(pSolver.paths().back()));
    BOOST_CHECK(checkPath(pSolver.paths().back(),0.5));
    for(size_t i = 0 ; i < 10 ; ++i){
      pSolver.optimizePath(pSolver.paths().back());
      BOOST_CHECK_EQUAL(pSolver.paths().size(),3+i);
      BOOST_CHECK(checkPathVector(pSolver.paths().back()));
      BOOST_CHECK(checkPath(pSolver.paths().back(),0.5));
      std::cout<<"("<<i+1<<"/10)  "<<std::flush;
    }
    std::cout<<std::endl;
}


BOOST_AUTO_TEST_SUITE_END()
