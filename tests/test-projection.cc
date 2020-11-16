#define BOOST_TEST_MODULE test - projection
#include <pinocchio/fwd.hpp>
#include <boost/test/included/unit_test.hpp>

#include "tools-fullbody.hh"
#include <hpp/pinocchio/configuration.hh>
#include <hpp/rbprm/projection/projection.hh>
#include <hpp/core/fwd.hh>
using namespace hpp;
using namespace rbprm;
using core::value_type;

BOOST_AUTO_TEST_SUITE(test_projection)

BOOST_AUTO_TEST_CASE(projectToComPositionHyQ) {
  RbPrmFullBodyPtr_t fullBody = loadHyQ();

  core::Configuration_t q_ref(fullBody->device_->configSize());
  q_ref << 0.0, 0.0, 0.6838277139631803, 0.0, 0.0, 0.0, 1.0, 0.14279812395541294, 0.934392553166556,
      -0.9968239786882757, -0.06521258938340457, -0.8831796268418511, 1.150049183494211, -0.06927610020154493,
      0.9507443168724581, -0.8739975339028809, 0.03995660287873871, -0.9577096766517215, 0.93846028213260710;
  const std::string rLegId("rfleg");
  const std::string lLegId("lfleg");
  const std::string rhLegId("rhleg");
  const std::string lhLegId("lhleg");

  std::vector<std::string> allLimbs;
  allLimbs.push_back(rLegId);
  allLimbs.push_back(lLegId);
  allLimbs.push_back(rhLegId);
  allLimbs.push_back(lhLegId);

  State s_init = createState(fullBody, q_ref, allLimbs);
  BOOST_CHECK_EQUAL(s_init.nbContacts, 4);
  fullBody->device_->currentConfiguration(q_ref);
  hpp::pinocchio::Computation_t newflag = static_cast<hpp::pinocchio::Computation_t>(
      hpp::pinocchio::JOINT_POSITION | hpp::pinocchio::JACOBIAN | hpp::pinocchio::COM);
  fullBody->device_->controlComputation(newflag);
  fullBody->device_->computeForwardKinematics();
  fcl::Vec3f com_init = fullBody->device_->positionCenterOfMass();
  // com position with reference configuration given
  BOOST_CHECK_CLOSE(com_init[0], 0.038820542472487805, 1e-6);
  BOOST_CHECK_CLOSE(com_init[1], 0.013905920702454319, 1e-6);
  BOOST_CHECK_CLOSE(com_init[2], 0.64079567046650698, 1e-6);

  fcl::Vec3f com_goal = com_init;
  // set an easy com goal position, close to the reference one
  com_goal[0] = 0.;
  com_goal[1] = -0.1;
  com_goal[2] = 0.64;

  // check if the projection is successfull and the new com is correct
  rbprm::projection::ProjectionReport rep = rbprm::projection::projectToComPosition(fullBody, com_goal, s_init);
  BOOST_CHECK(rep.success_);
  BOOST_CHECK_EQUAL(rep.result_.nbContacts, 4);

  fullBody->device_->currentConfiguration(rep.result_.configuration_);
  fullBody->device_->computeForwardKinematics();
  fcl::Vec3f com_pino = fullBody->device_->positionCenterOfMass();
  for (size_t i = 0; i < 3; ++i)
    BOOST_CHECK_SMALL(com_pino[i] - com_goal[i],
                      1e-4);  // precision should be the same as the error treshold of the config projector

  // check if contact position are preserved :
  for (std::vector<std::string>::const_iterator cit = allLimbs.begin(); cit != allLimbs.end(); ++cit) {
    rbprm::RbPrmLimbPtr_t limb = fullBody->GetLimbs().at(*cit);
    const std::string& limbName = *cit;
    const fcl::Vec3f position = limb->effector_.currentTransformation().translation();
    for (size_t i = 0; i < 3; ++i) BOOST_CHECK_SMALL(position[i] - s_init.contactPositions_[limbName][i], 1e-4);
  }

  // set a com position a little farther
  com_goal[0] = 0.1;
  com_goal[1] = -0.05;
  com_goal[2] = 0.66;
  // check if the projection is successfull and the new com is correct
  rep = rbprm::projection::projectToComPosition(fullBody, com_goal, s_init);
  BOOST_CHECK(rep.success_);
  BOOST_CHECK_EQUAL(rep.result_.nbContacts, 4);

  fullBody->device_->currentConfiguration(rep.result_.configuration_);
  fullBody->device_->computeForwardKinematics();
  com_pino = fullBody->device_->positionCenterOfMass();
  for (size_t i = 0; i < 3; ++i)
    BOOST_CHECK_SMALL(com_pino[i] - com_goal[i],
                      1e-4);  // precision should be the same as the error treshold of the config projector

  // check if contact position are preserved :
  for (std::vector<std::string>::const_iterator cit = allLimbs.begin(); cit != allLimbs.end(); ++cit) {
    rbprm::RbPrmLimbPtr_t limb = fullBody->GetLimbs().at(*cit);
    const std::string& limbName = *cit;
    const fcl::Vec3f position = limb->effector_.currentTransformation().translation();
    for (size_t i = 0; i < 3; ++i) BOOST_CHECK_SMALL(position[i] - s_init.contactPositions_[limbName][i], 1e-4);
  }

  srand((unsigned int)(time(NULL)));
  for (size_t k = 0; k < 100; ++k) {
    // randomly sample CoM position close to the reference one :
    com_goal[0] = -0.3 + 0.6 * (value_type(rand()) / value_type(RAND_MAX));
    com_goal[1] = -0.3 + 0.6 * (value_type(rand()) / value_type(RAND_MAX));
    com_goal[2] = 0.62 + 0.06 * (value_type(rand()) / value_type(RAND_MAX));
    // check if the projection is successfull and the new com is correct
    rep = rbprm::projection::projectToComPosition(fullBody, com_goal, s_init);
    if (rep.success_) {
      BOOST_CHECK_EQUAL(rep.result_.nbContacts, 4);

      fullBody->device_->currentConfiguration(rep.result_.configuration_);
      fullBody->device_->computeForwardKinematics();
      com_pino = fullBody->device_->positionCenterOfMass();
      for (size_t i = 0; i < 3; ++i)
        BOOST_CHECK_SMALL(com_pino[i] - com_goal[i],
                          1e-4);  // precision should be the same as the error treshold of the config projector

      // check if contact position are preserved :
      for (std::vector<std::string>::const_iterator cit = allLimbs.begin(); cit != allLimbs.end(); ++cit) {
        rbprm::RbPrmLimbPtr_t limb = fullBody->GetLimbs().at(*cit);
        const std::string& limbName = *cit;
        const fcl::Vec3f position = limb->effector_.currentTransformation().translation();
        for (size_t i = 0; i < 3; ++i) BOOST_CHECK_SMALL(position[i] - s_init.contactPositions_[limbName][i], 1e-4);
      }
    }
  }
}

/*
BOOST_AUTO_TEST_CASE (projectToComPositionSimpleHumanoid) {

  RbPrmFullBodyPtr_t fullBody = loadSimpleHumanoid();
  const std::string rLegId("rleg");
  const std::string lLegId("lleg");

  std::vector<std::string> allLimbs;
  allLimbs.push_back(rLegId);
  allLimbs.push_back(lLegId);

  core::Configuration_t q_ref(fullBody->device_->neutralConfiguration());
  std::cout<<"q ref = "<<q_ref<<std::endl;
  State s_init = createState(fullBody,q_ref,allLimbs);
  BOOST_CHECK_EQUAL(s_init.nbContacts,2);
  fullBody->device_->currentConfiguration(q_ref);
  pinocchio::Computation_t newflag = static_cast <pinocchio::Computation_t> (pinocchio::JOINT_POSITION |
pinocchio::JACOBIAN | pinocchio::COM); fullBody->device_->controlComputation (newflag);
  fullBody->device_->computeForwardKinematics();
  fcl::Vec3f com_init  = fullBody->device_->positionCenterOfMass();
  std::cout<<"com_init = "<<com_init<<std::endl;
  fcl::Vec3f com_pino,com_goal;
  rbprm::projection::ProjectionReport rep;
  srand((unsigned int)(time(NULL)));
  for(size_t k = 0 ; k < 100 ; ++k){
    //randomly sample CoM position close to the reference one :
    com_goal[0] = com_init[0]-0.2 + 0.4*(value_type(rand()) / value_type(RAND_MAX));
    com_goal[1] = com_init[1]-0.2 + 0.4*(value_type(rand()) / value_type(RAND_MAX));
    com_goal[2] = com_init[2]-0.1 + 0.2*(value_type(rand()) / value_type(RAND_MAX));
    // check if the projection is successfull and the new com is correct
    rep = rbprm::projection::projectToComPosition(fullBody,com_goal,s_init);
    if(rep.success_){
      std::cout<<"com goal : "<<com_goal<<std::endl;
      BOOST_CHECK_EQUAL(rep.result_.nbContacts,2);

      fullBody->device_->currentConfiguration(rep.result_.configuration_);
      fullBody->device_->computeForwardKinematics();
      com_pino  = fullBody->device_->positionCenterOfMass();
       std::cout<<"com pino : "<<com_pino<<std::endl;
      for(size_t i = 0 ; i < 3 ; ++i)
        BOOST_CHECK_SMALL(com_pino[i]-com_goal[i],1e-4); // precision should be the same as the error treshold of the
config projector


      // check if contact position are preserved :
      for(std::vector<std::string>::const_iterator cit = allLimbs.begin(); cit != allLimbs.end(); ++cit)
       {
        rbprm::RbPrmLimbPtr_t limb = fullBody->GetLimbs().at(*cit);
        const std::string& limbName = *cit;
        const fcl::Vec3f position = limb->effector_.currentTransformation().translation();
        for(size_t i = 0 ; i < 3 ; ++i)
          BOOST_CHECK_SMALL(position[i] - s_init.contactPositions_[limbName][i],1e-4);
      }
    }
  }
}

*/

BOOST_AUTO_TEST_SUITE_END()
