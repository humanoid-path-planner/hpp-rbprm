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
#include <hpp/rbprm/interpolation/rbprm-path-interpolation.hh>
#include "tools-fullbody.hh"
#include "tools-obstacle.hh"


using namespace hpp;
using namespace rbprm;




BOOST_AUTO_TEST_SUITE( rbrrt_quasiStatic )

BOOST_AUTO_TEST_CASE (load_abstract_model) {

    hpp::pinocchio::RbPrmDevicePtr_t rbprmDevice = loadHyQAbsract();
}


hpp::core::ProblemSolverPtr_t planDarpa(BindShooter& bShooter)
{
    hpp::pinocchio::RbPrmDevicePtr_t rbprmDevice = loadHyQAbsract();
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
    for(int i =0; i< 10; ++i)
    {
        pSolver.optimizePath(pSolver.paths().back());
    }
    pSolver.optimizePath(pSolver.paths().back());
    return ps;
}


/*BOOST_AUTO_TEST_CASE (plan_path) {
    BindShooter bShooter;
    PathVectorPtr_t resPath = planDarpa(bShooter)->paths().back();
    // TODO CHECK COLLISION CONSTRAINTS
}*/


BOOST_AUTO_TEST_CASE (interpolate_path) {
    BindShooter bShooter;
    hpp::core::ProblemSolverPtr_t ps = planDarpa(bShooter);
    PathVectorPtr_t resPath =ps->paths().back();

    RbPrmFullBodyPtr_t fullBody = loadHyQ();

    Configuration_t q = fullBody->device_->currentConfiguration(); q[2] +=0.02;
    rbprm::State startState, endState;
    const std::string rLegId("rfleg");
    const std::string lLegId("lfleg");
    const std::string rhLegId("rhleg");
    const std::string lhLegId("lhleg");

    std::vector<std::string> allLimbs;
    allLimbs.push_back(rLegId);
    allLimbs.push_back(lhLegId);
    allLimbs.push_back(lLegId);
    allLimbs.push_back(rhLegId);

    startState = createState(fullBody, q,  allLimbs);
    q[0]=3;
    endState   = createState(fullBody, q, allLimbs);


    hpp::rbprm::interpolation::RbPrmInterpolationPtr_t interpolator =
                rbprm::interpolation::RbPrmInterpolation::create(fullBody,startState,endState,resPath,false,true);

    rbprm::T_StateFrame frams = interpolator->Interpolate(ps->affordanceObjects, bShooter.affFilter_,
                 0.01, 8, false);
    BOOST_CHECK(frams.back().second.configuration_[0] > 2.8);
}


BOOST_AUTO_TEST_SUITE_END()

