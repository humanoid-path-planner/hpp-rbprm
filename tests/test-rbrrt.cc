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


#define BOOST_TEST_MODULE test-rbrrt
#include <boost/test/included/unit_test.hpp>

#include <hpp/core/problem-solver.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/rbprm/rbprm-device.hh>
#include "tools-fullbody.hh"
#include "tools-obstacle.hh"


using namespace hpp;
using namespace rbprm;




BOOST_AUTO_TEST_SUITE( rbrrt_quasiStatic )

BOOST_AUTO_TEST_CASE (load_abstract_model) {

    hpp::pinocchio::RbPrmDevicePtr_t rbprmDevice = loadHyQAbsract();
}


BOOST_AUTO_TEST_CASE (plan_path) {
    hpp::pinocchio::RbPrmDevicePtr_t rbprmDevice = loadHyQAbsract();
    BindShooter bShooter;
    bShooter.so3Bounds_ = addSo3LimitsHyQ();
    hpp::core::ProblemSolverPtr_t  ps = configureRbprmProblemSolverForSupportLimbs(rbprmDevice, bShooter);
    hpp::core::ProblemSolver& pSolver = *ps;
    loadDarpa(pSolver);

    // configure planner
    pSolver.addPathOptimizer(std::string("RandomShortcut"));
    pSolver.configurationShooterType(std::string("RbprmShooter"));
    pSolver.pathValidationType(std::string("RbprmPathValidation"),0.05);

    core::Configuration_t q_init(rbprmDevice->configSize());
    q_init << -2, 0, 0.63, 0.0, 0.0, 0.0, 1.0;

    core::Configuration_t q_goal = q_init;
    q_goal(0)=3;

    pSolver.initConfig(ConfigurationPtr_t(new core::Configuration_t(q_init)));
    pSolver.addGoalConfig(ConfigurationPtr_t(new core::Configuration_t(q_goal)));
    pSolver.solve();
    pSolver.optimizePath(pSolver.paths().back());
    PathVectorPtr_t resPath = pSolver.paths().back();
    Configuration_t q;
    // TODO CHECK COLLISION CONSTRAINTS
}


BOOST_AUTO_TEST_SUITE_END()

