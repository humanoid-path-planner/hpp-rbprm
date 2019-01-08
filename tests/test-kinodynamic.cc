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


using namespace hpp;
using namespace rbprm;




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
    BOOST_CHECK_EQUAL(pSolver.paths().size(),1);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),8.,1e-6);
    core::PathVectorPtr_t pv = boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pv->numberPaths (),1);
    pSolver.solve();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),3);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),8.,1e-6);
    pv = boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pv->numberPaths (),1);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),4);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),8.,1e-6);
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
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),3.3635856610148585,1e-10);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),2);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),3.3635856610148585,1e-10);
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
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),3.3635856610148585,1e-10);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),6);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),3.3635856610148585,1e-10);
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
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),3.7976578442318836,1e-10);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),10);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),3.7976578442318836,1e-10);
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
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),3.3635856610148585,1e-10);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),14);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),3.3635856610148585,1e-10);
    BOOST_CHECK_EQUAL(boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back())->numberPaths (),1);

    pSolver.resetGoalConfigs();
    q_goal(0) = 1.3;
    q_goal(1) = -1.2;
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    success = pSolver.prepareSolveStepByStep();
    BOOST_CHECK(success);
    pSolver.finishSolveStepByStep();
    BOOST_CHECK_EQUAL(pSolver.paths().size(),15);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),3.7621064326203357,1e-10);
    pSolver.optimizePath(pSolver.paths().back());
    BOOST_CHECK_EQUAL(pSolver.paths().size(),16);
    BOOST_CHECK_CLOSE(pSolver.paths().back()->length(),3.7621064326203357,1e-10);
    BOOST_CHECK_EQUAL(boost::dynamic_pointer_cast<core::PathVector>(pSolver.paths().back())->numberPaths (),1);

}


BOOST_AUTO_TEST_SUITE_END()
