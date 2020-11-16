// Copyright (C) 2014 LAAS-CNRS
// Author: Steve Tonneau
//
// This file is part of the hpp-rbprm.
//
// hpp-core is free software: you can redistribute it and/or modify
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

//#include "test-tools.hh"

#define BOOST_TEST_MODULE test - limb - rrt
#include <hpp/rbprm/interpolation/rbprm-path-interpolation.hh>
#include <hpp/rbprm/interpolation/com-trajectory.hh>
#include <hpp/rbprm/interpolation/com-rrt.hh>
#include <Eigen/Geometry>
#include "tools-fullbody.hh"

#include <boost/test/included/unit_test.hpp>

using namespace hpp;
using namespace rbprm;
using core::Configuration_t;

const std::string rf_joint_name("leg_right_sole_fix_joint");
const std::string lf_joint_name("leg_left_sole_fix_joint");

BOOST_AUTO_TEST_SUITE(test_limb_rrt)

BOOST_AUTO_TEST_CASE(limbRRTflat) {
  RbPrmFullBodyPtr_t fullBody = loadTalos();
  hpp::core::ProblemSolverPtr_t ps = hpp::core::ProblemSolver::create();
  ps->robot(fullBody->device_);
  size_t configSize = fullBody->device_->configSize();
  Configuration_t q_init(configSize), q_goal(configSize);

  // two double support configuration on a flat floor, with the right foot moving forward
  q_init << -0.008329563887569362, 0.06620576007513382, 0.9930942566819294, 0.00038549640050368163,
      -8.145032439571079e-05, 7.703828639384208e-06, 0.9999999223495074, 2.3026656108884685e-06, -0.10867424742157307,
      -0.494756338156251, 1.0133591753493731, -0.5184389757611502, 0.10619525478737052, 2.3479772260881875e-06,
      -0.10894869854222643, -0.498994740662798, 1.022629131517737, -0.5234705245029162, 0.10646970591173445,
      7.741718104277579e-05, 0.0067860706341258265, 0.25877492217533615, 0.17368331978387042, 0.019949119349010752,
      -0.5547794316225498, -0.0013729464130708222, 0.009192352838156734, 0.09930480694564815, -0.004919981826720007,
      -0.2559990172027164, -0.16445110685094266, 0.019342285054779335, -0.49466745713075566, -0.0014774857749049518,
      0.009626279378258808, 0.10062722805604071, -0.005090515088527805, -0.00015176578141020918,
      0.00010792272041378931, 0, 0, 0, 0, 0, 0;
  q_goal << 0.021392492459073587, 0.05095293446787785, 0.9993885697183194, -0.00014218304338031964,
      -5.785918096714148e-05, -2.601588863094496e-05, 0.9999999878797353, 6.149474192480515e-05, -0.08168186661587325,
      -0.4347510047888222, 0.9866440623796208, -0.5517769620513717, 0.08025823634089192, 6.151039617785657e-05,
      -0.08186211194649087, -0.633014712720102, 0.8980893504694407, -0.26495854107692185, 0.08043848168554622,
      -7.192772889107505e-05, 0.006735418616846882, 0.2581867223496433, 0.1726631234593809, 0.005819393952467138,
      -0.5175489557152446, -0.00012355213481863927, 0.0024250046416533643, 0.09983111589088325, -0.005009776217247435,
      -0.258000081287665, -0.17328110086690235, 0.004116413934575488, -0.5112450909659445, -0.00020419620913673253,
      0.0016736569413459677, 0.09957814693818354, -0.00499155448845885, -0.0006293152753588235,
      -2.2792373347219125e-05, 0, 0, 0, 0, 0, 0;
  State s_init = createState(fullBody, q_init);
  State s_goal = createState(fullBody, q_goal);

  // build a CoM reference Path in the problem solver
  // first define a list of c, dc, ddc :
  T_Configuration c_t, dc_t, ddc_t;
  c_t.push_back(Vector3(-0.007335037762674138, 0.056386684822780066, 0.8548761161600128));
  c_t.push_back(Vector3(-0.0068282315994240075, 0.05880884353484932, 0.8553261738101325));
  c_t.push_back(Vector3(-0.006166712140232212, 0.0609516642750017, 0.8557810761994065));
  c_t.push_back(Vector3(-0.005334470979775361, 0.06277872826730958, 0.8562396306568699));
  c_t.push_back(Vector3(-0.0043143610925355375, 0.06425340627157496, 0.8567006469424537));
  c_t.push_back(Vector3(-0.003087982471589512, 0.06533883413153226, 0.8571629364532667));
  c_t.push_back(Vector3(-0.0016355594107369067, 0.06599788818749985, 0.8576253114310225));
  c_t.push_back(Vector3(6.419138846776049e-05, 0.06619316052180603, 0.8580865841700384));
  c_t.push_back(Vector3(0.0020342027643879884, 0.06588693401692583, 0.8585455662252698));
  c_t.push_back(Vector3(0.004299210044314844, 0.06504115720864033, 0.8590010676198598));
  c_t.push_back(Vector3(0.006885916014741039, 0.0636174189171785, 0.8594518960516682));
  c_t.push_back(Vector3(0.0098231690128288, 0.06157692263962375, 0.8598968560982675));
  c_t.push_back(Vector3(0.013142155326066588, 0.05888046068748006, 0.8603347484199007));
  c_t.push_back(Vector3(0.016876607181162673, 0.05548838805530804, 0.8607643689599564));
  c_t.push_back(Vector3(0.021063027700722402, 0.051360596017010034, 0.8611845081427736));
  c_t.push_back(Vector3(0.02574093440691642, 0.04643655974634927, 0.8615937133499726));
  c_t.push_back(Vector3(0.031160741135083655, 0.04035013641071752, 0.8619827292958794));

  dc_t.push_back(Vector3(0.004002077423403626, 0.026090015253394872, 0.0044567812068005115));
  dc_t.push_back(Vector3(0.005435676129913748, 0.02356858910290817, 0.004514194745513037));
  dc_t.push_back(Vector3(0.0070214921041798824, 0.02068440331545588, 0.004559658436728889));
  dc_t.push_back(Vector3(0.008770101877294134, 0.01743553495255048, 0.004593202557663893));
  dc_t.push_back(Vector3(0.010693167726252274, 0.01381981796749292, 0.004614849437938533));
  dc_t.push_back(Vector3(0.012803515454947801, 0.009834841739830702, 0.004624613474331654));
  dc_t.push_back(Vector3(0.01511521993526881, 0.005477949463049019, 0.004622501140311318));
  dc_t.push_back(Vector3(0.017643698978823105, 0.0007462363716895011, 0.0046085109902880604));
  dc_t.push_back(Vector3(0.020405816165390153, -0.004363452195850822, 0.0045826336585769985));
  dc_t.push_back(Vector3(0.023419993313970346, -0.009854522900821883, 0.004544851853057419));
  dc_t.push_back(Vector3(0.026706333346691903, -0.0157306366759034, 0.004495140343514042));
  dc_t.push_back(Vector3(0.03028675436541839, -0.02199571116728329, 0.0044334659446364465));
  dc_t.push_back(Vector3(0.0341851358366661, -0.02865392335087786, 0.004359787493632031));
  dc_t.push_back(Vector3(0.03842747786592224, -0.03570971233435052, 0.004274055822285332));
  dc_t.push_back(Vector3(0.04304207466852871, -0.04316778244974884, 0.004176213721910038));
  dc_t.push_back(Vector3(0.04805970704538201, -0.051431625515619264, 0.00406146148090171));

  ddc_t.push_back(Vector3(0.013254873206397372, -0.022504259983497088, 0.0006639920340443563));
  ddc_t.push_back(Vector3(0.014704118285834341, -0.0261189460959399, 0.0005442167026611065));
  ddc_t.push_back(Vector3(0.016251431848449092, -0.029751047009470705, 0.0004248038695311637));
  ddc_t.push_back(Vector3(0.017907133632619684, -0.033402983338637546, 0.0003056739171463166));
  ddc_t.push_back(Vector3(0.019682266266665416, -0.03707718967509238, 0.00018674741284035665));
  ddc_t.push_back(Vector3(0.02158866891722941, -0.04077611559198769, 6.794505860134234e-05));
  ddc_t.push_back(Vector3(0.02363905625021711, -0.04450222717883896, -5.081236143957349e-05));
  ddc_t.push_back(Vector3(0.02584710323098434, -0.048258008661537344, -0.0001696040338337));
  ddc_t.push_back(Vector3(0.028227536329503075, -0.05204596405164602, -0.00028850916875201166));
  ddc_t.push_back(Vector3(0.030796231739217347, -0.05586861881360452, -0.00040760705304296665));
  ddc_t.push_back(Vector3(0.03357032126568897, -0.05972852155376997, -0.0005269771034628092));
  ddc_t.push_back(Vector3(0.03656830659459445, -0.06362824572629266, -0.000646698920167138));
  ddc_t.push_back(Vector3(0.03981018271450337, -0.06757039139342141, -0.0007668523407436574));
  ddc_t.push_back(Vector3(0.043317571383828524, -0.07155758713223562, -0.0008875174965792404));
  ddc_t.push_back(Vector3(0.047113866126768794, -0.07559249359936515, -0.001008774893639288));
  ddc_t.push_back(Vector3(0.05122445891168781, -0.08764816999729377, -0.001225393708891645));

  const double dt = 0.1;
  const double duration = 1.6;
  // build a path in the problem solver from this list of points and derivatives
  core::PathVectorPtr_t res = core::PathVector::create(3, 3);
  CIT_Configuration cit = c_t.begin();
  ++cit;
  CIT_Configuration cdit = dc_t.begin();
  CIT_Configuration cddit = ddc_t.begin();
  for (; cit != c_t.end(); ++cit, ++cdit, ++cddit) {
    res->appendPath(interpolation::ComTrajectory::create(*(cit - 1), *cit, *cdit, *cddit, dt));
  }
  size_t com_path_id = ps->addPath(res);
  core::PathPtr_t com_path = ps->paths()[com_path_id];
  BOOST_CHECK_CLOSE(com_path->length(), duration, 0.001);  // check duration of the com trajectory created

  // solve limb rrt :
  core::PathPtr_t eff_path = interpolation::comRRT(fullBody, ps, com_path, s_init, s_goal, 5, true);

  // test the resulting path
  BOOST_CHECK_CLOSE(eff_path->length(), 1., 1.);  // results of limb-rrt should always be defined on [0;1]
  bool success;
  double t_id = eff_path->operator()(0, success)[configSize];
  BOOST_CHECK(success);
  BOOST_CHECK_EQUAL(t_id, 0.);
  t_id = eff_path->operator()(eff_path->length(), success)[configSize];
  BOOST_CHECK(success);
  BOOST_CHECK_CLOSE(t_id, 1., 1e-6);

  // check that path begin and end at the correct configurations
  Configuration_t p0(eff_path->operator()(0, success).head(configSize));
  BOOST_CHECK(success);
  BOOST_CHECK(p0.isApprox(q_init));
  fullBody->device_->currentConfiguration(p0);
  fullBody->device_->computeForwardKinematics();
  Transform3f rf_pose = fullBody->device_->getFrameByName(rf_joint_name).currentTransformation();
  BOOST_CHECK(rf_pose.translation().isApprox(Vector3(-0.00884695, -0.0851828, 0.00345772), 1e-3));

  Configuration_t p1(eff_path->operator()(eff_path->length(), success).head(configSize));
  BOOST_CHECK(success);
  BOOST_CHECK(p1.isApprox(q_goal));
  fullBody->device_->currentConfiguration(p1);
  fullBody->device_->computeForwardKinematics();
  rf_pose = fullBody->device_->getFrameByName(rf_joint_name).currentTransformation();
  BOOST_CHECK(rf_pose.translation().isApprox(Vector3(0.141153, -0.0851828, 0.00345772), 1e-3));

  // check that the left foot position is constant during all the motion:
  double t = 0.;
  while (t <= eff_path->length()) {
    fullBody->device_->currentConfiguration(eff_path->operator()(t, success).head(configSize));
    BOOST_CHECK(success);
    fullBody->device_->computeForwardKinematics();
    Transform3f lf_pose = fullBody->device_->getFrameByName(lf_joint_name).currentTransformation();
    BOOST_CHECK(lf_pose.translation().isApprox(Vector3(-0.00884695, 0.0848172, 0.00199798), 1e-2));
    t += 0.01;
  }

  // check if CoM reference is followed:
  fullBody->device_->controlComputation(hpp::pinocchio::COMPUTE_ALL);
  Configuration_t com(3), com_ref(3);
  Configuration_t q(configSize + 1);
  t = 0.;
  while (t <= eff_path->length()) {
    q = eff_path->operator()(t, success);
    BOOST_CHECK(success);
    t_id = q[configSize];
    fullBody->device_->currentConfiguration(q.head(configSize));
    fullBody->device_->computeForwardKinematics();
    com = fullBody->device_->positionCenterOfMass();
    com_ref = com_path->operator()(t_id* duration, success);
    BOOST_CHECK(success);
    // std::cout<<" t   = "<< t <<std::endl;
    // std::cout<<" t id= "<< t_id <<std::endl;
    // std::cout<<" com     = "<<com.transpose()<<std::endl;
    // std::cout<<" com ref = "<<com_ref.transpose()<<std::endl;
    // FIXME, see https://github.com/humanoid-path-planner/hpp-rbprm/issues/46
    // BOOST_CHECK(com.isApprox(com_ref, 1e-2));
    t += 0.05;
  }
}

BOOST_AUTO_TEST_SUITE_END()
