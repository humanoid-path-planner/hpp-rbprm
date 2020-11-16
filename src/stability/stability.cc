//
// Copyright (c) 2014 CNRS
// Authors: Steve Tonneau (steve.tonneau@laas.fr)
//
// This file is part of hpp-rbprm.
// hpp-rbprm is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-rbprm is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/rbprm/stability/stability.hh>
#include <hpp/rbprm/stability/support.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/center-of-mass-computation.hh>
#include <hpp/rbprm/tools.hh>

#include <Eigen/Dense>

#include <vector>
#include <map>
#include <string>

#ifdef PROFILE
#include "hpp/rbprm/rbprm-profiler.hh"
#endif

using namespace hpp;
using namespace hpp::core;
using namespace hpp::pinocchio;
using namespace hpp::rbprm;
using namespace centroidal_dynamics;

namespace hpp {
namespace rbprm {
namespace stability {

void computeRectangleContact(const std::string& name, const RbPrmLimbPtr_t limb, const State& state, Ref_matrix43 p,
                             double lx = 0, double ly = 0) {
  hppDout(notice, "Compute rectangular contact : ");
  if (lx == 0) lx = limb->x_;
  if (ly == 0) ly = limb->y_;
  const fcl::Vec3f& position = state.contactPositions_.at(name);
  hppDout(notice, "Position of center : " << position.transpose());
  // create rotation matrix from normal
  Eigen::Matrix3d R;
  p << lx, ly, 0, lx, -ly, 0, -lx, -ly, 0, -lx, ly, 0;
  if (limb->contactType_ == _3_DOF) {
    // create rotation matrix from normal
    const fcl::Vec3f& normal = state.contactNormals_.at(name);
    const fcl::Vec3f z_current = limb->effector_.currentTransformation().rotation() * limb->normal_;
    const fcl::Matrix3f alignRotation = tools::GetRotationMatrix(z_current, normal);
    const fcl::Matrix3f rotation = alignRotation * limb->effector_.currentTransformation().rotation();
    const fcl::Vec3f offset = rotation * limb->offset_;
    Eigen::Vector3d z, x, y;
    for (int i = 0; i < 3; ++i) z[i] = normal[i];
    x = z.cross(Eigen::Vector3d(0, -1, 0));
    if (x.norm() < 10e-6) {
      y = z.cross(fcl::Vec3f(1, 0, 0));
      y.normalize();
      x = y.cross(z);
    } else {
      x.normalize();
      y = z.cross(x);
    }
    R.block<3, 1>(0, 0) = x;
    R.block<3, 1>(0, 1) = y;
    R.block<3, 1>(0, 2) = z;

    for (std::size_t i = 0; i < 4; ++i) {
      p.row(i) = position + (R * (p.row(i).transpose())) + offset;
    }
  } else {
    fcl::Vec3f z_axis(0, 0, 1);
    fcl::Matrix3f rotationLocal = tools::GetRotationMatrix(z_axis, limb->normal_).inverse();
    fcl::Transform3f roWorld;
    roWorld.setRotation(state.contactRotation_.at(name));
    roWorld.setTranslation(position);
    for (std::size_t i = 0; i < 4; ++i) {
      fcl::Vec3f pLocal = rotationLocal * (p.row(i).transpose()) + limb->offset_;
      p.row(i) = (roWorld * pLocal).getTranslation();
      hppDout(notice, "position : " << p.row(i));
    }
  }
}

Vector3 computePointContact(const std::string& name, const RbPrmLimbPtr_t limb, const State& state) {
  const fcl::Vec3f& position = state.contactPositions_.at(name);
  // create rotation matrix from normal
  const fcl::Vec3f& normal = state.contactNormals_.at(name);
  const fcl::Vec3f z_current = limb->effector_.currentTransformation().rotation() * limb->normal_;
  const fcl::Matrix3f alignRotation = tools::GetRotationMatrix(z_current, normal);
  const fcl::Matrix3f rotation = alignRotation * limb->effector_.currentTransformation().rotation();
  const fcl::Vec3f offset = rotation * limb->offset_;
  return position + offset;
}

Equilibrium initLibrary(const RbPrmFullBodyPtr_t fullbody) {
  return Equilibrium(fullbody->device_->name(), fullbody->device_->mass(), 4, SOLVER_LP_QPOASES, true, 10, false);
}

std::size_t numContactPoints(const RbPrmLimbPtr_t& limb) {
  if (limb->contactType_ == _3_DOF)
    return 1;
  else
    return 4;
}

const std::vector<std::size_t> numContactPoints(const T_Limb& limbs, const std::vector<std::string>& contacts,
                                                std::size_t& totalNumContacts) {
  std::size_t n;
  std::vector<std::size_t> res;
  for (std::vector<std::string>::const_iterator cit = contacts.begin(); cit != contacts.end(); ++cit) {
    n = numContactPoints(limbs.at(*cit));
    totalNumContacts += n;
    res.push_back(n);
  }
  return res;
}

centroidal_dynamics::Vector3 setupLibrary(const RbPrmFullBodyPtr_t fullbody, State& state, Equilibrium& sEq,
                                          EquilibriumAlgorithm& alg, core::value_type friction, const double feetX,
                                          const double feetY) throw(std::runtime_error) {
  friction = fullbody->getFriction();
  hppDout(notice, "Setup centroidal dynamic lib, friction = " << friction);
  const rbprm::T_Limb& limbs = fullbody->GetLimbs();
  hpp::pinocchio::ConfigurationIn_t save = fullbody->device_->currentConfiguration();
  std::vector<std::string> contacts;
  std::vector<std::string> graspscontacts;
  for (std::map<std::string, fcl::Vec3f>::const_iterator cit = state.contactPositions_.begin();
       cit != state.contactPositions_.end(); ++cit) {
    if (limbs.at(cit->first)->grasps_)
      graspscontacts.push_back(cit->first);
    else
      contacts.push_back(cit->first);
  }
  fullbody->device_->currentConfiguration(state.configuration_);
  fullbody->device_->computeForwardKinematics();
  std::size_t nbContactPoints(0);
  std::vector<std::size_t> contactPointsInc = numContactPoints(limbs, contacts, nbContactPoints);
  std::vector<std::size_t> contactGraspPointsInc = numContactPoints(limbs, graspscontacts, nbContactPoints);
  centroidal_dynamics::MatrixX3 normals(nbContactPoints, 3);
  centroidal_dynamics::MatrixX3 positions(nbContactPoints, 3);
  std::size_t currentIndex(0), c(0);
  for (std::vector<std::size_t>::const_iterator cit = contactPointsInc.begin(); cit != contactPointsInc.end();
       ++cit, ++c) {
    const RbPrmLimbPtr_t limb = limbs.at(contacts[c]);
    const fcl::Vec3f& n = state.contactNormals_.at(contacts[c]);
    Vector3 normal(n[0], n[1], n[2]);
    normal.normalize();
    const std::size_t& inc = *cit;
    if (inc > 1)
      computeRectangleContact(contacts[c], limb, state, positions.middleRows<4>(currentIndex), feetX, feetY);
    else
      positions.middleRows<1>(currentIndex, inc) = computePointContact(contacts[c], limb, state);
    for (std::size_t i = 0; i < inc; ++i) {
      normals.middleRows<1>(currentIndex + i) = normal;
    }
    currentIndex += inc;
  }
  int graspIndex = -1;
  if (graspscontacts.size() > 0) {
    c = 0;
    graspIndex = (int)currentIndex;
    for (std::vector<std::size_t>::const_iterator cit = contactGraspPointsInc.begin();
         cit != contactGraspPointsInc.end(); ++cit, ++c) {
      const RbPrmLimbPtr_t limb = limbs.at(graspscontacts[c]);
      const fcl::Vec3f& n = state.contactNormals_.at(graspscontacts[c]);
      Vector3 normal(n[0], n[1], n[2]);
      const std::size_t& inc = *cit;
      if (inc > 1)
        computeRectangleContact(graspscontacts[c], limb, state, positions.middleRows<4>(currentIndex));
      else
        positions.middleRows<1>(currentIndex, inc) = computePointContact(graspscontacts[c], limb, state);
      for (std::size_t i = 0; i < inc; ++i) {
        normals.middleRows<1>(currentIndex + i) = normal;
      }
      currentIndex += inc;
    }
  }
  centroidal_dynamics::Vector3 com;
  /*pinocchio::CenterOfMassComputationPtr_t comcptr = pinocchio::CenterOfMassComputation::create(fullbody->device_);
  comcptr->add(fullbody->device_->getJointByName("romeo/base_joint_xyz"));
  comcptr->computeMass();
  comcptr->compute();
  const fcl::Vec3f comfcl = comcptr->com();*/
  const fcl::Vec3f comfcl = fullbody->device_->positionCenterOfMass();
  for (int i = 0; i < 3; ++i) com(i) = comfcl[i];
  fullbody->device_->currentConfiguration(save);
  if (graspIndex > -1 && alg != EQUILIBRIUM_ALGORITHM_PP) {
    alg = EQUILIBRIUM_ALGORITHM_PP;
  }
  hppDout(notice, "Setup cone contacts : ");
  hppDout(notice, "position : \n" << positions);
  hppDout(notice, "normal : \n" << normals);
  bool success = sEq.setNewContacts(positions, normals, friction, alg);
  if (!success) throw std::runtime_error("Error in centroidal-dynamic lib while computing new contacts");
  return com;
}

std::pair<MatrixXX, VectorX> ComputeCentroidalCone(const RbPrmFullBodyPtr_t fullbody, State& state,
                                                   const hpp::core::value_type friction) {
  std::pair<MatrixXX, VectorX> res;
  MatrixXX& H = res.first;
  VectorX& h = res.second;
#ifdef PROFILE
  RbPrmProfiler& watch = getRbPrmProfiler();
  watch.start("test balance");
#endif
  Equilibrium staticEquilibrium(initLibrary(fullbody));
  centroidal_dynamics::EquilibriumAlgorithm alg = EQUILIBRIUM_ALGORITHM_PP;
  setupLibrary(fullbody, state, staticEquilibrium, alg, friction);
#ifdef PROFILE
  watch.stop("test balance");
#endif
  LP_status status = LP_STATUS_OPTIMAL;
  if (status != LP_STATUS_OPTIMAL) {
    std::cout << "error " << std::endl;
  } else {
    status = staticEquilibrium.getPolytopeInequalities(H, h);
    if (status != LP_STATUS_OPTIMAL) {
      std::cout << "error " << std::endl;
      H = Eigen::MatrixXd::Zero(6, 6);
      h = Eigen::MatrixXd::Zero(6, 1);
    }
  }
  return res;
}

double IsStable(const RbPrmFullBodyPtr_t fullbody, State& state, fcl::Vec3f acc, fcl::Vec3f com,
                const centroidal_dynamics::EquilibriumAlgorithm algorithm) {
#ifdef PROFILE
  RbPrmProfiler& watch = getRbPrmProfiler();
  watch.start("test balance");
#endif
  centroidal_dynamics::EquilibriumAlgorithm alg = algorithm;
  // centroidal_dynamics::EquilibriumAlgorithm alg= centroidal_dynamics::EQUILIBRIUM_ALGORITHM_PP;
  if (fullbody->device_->extraConfigSpace().dimension() >= 6) {
    if (acc.norm() == 0) {
      hppDout(notice, "isStable ? called with acc = 0");
      hppDout(notice, "configuration in state = " << pinocchio::displayConfig(state.configuration_));
      core::size_type configSize = fullbody->device_->configSize() - fullbody->device_->extraConfigSpace().dimension();
      acc = state.configuration_.segment<3>(configSize + 3);
      hppDout(notice, "new acceleration = " << acc);
    }
  }
  Equilibrium staticEquilibrium(initLibrary(fullbody));
  centroidal_dynamics::Vector3 comComputed = setupLibrary(fullbody, state, staticEquilibrium, alg);
  if (!com.isZero()) {
    hppDout(notice, "isStable : a CoM was given as parameter, use this one. : " << com);
    comComputed = centroidal_dynamics::Vector3(com);
  }
  double res;
  LP_status status;
  if (alg == EQUILIBRIUM_ALGORITHM_PP) {
    hppDout(notice, "isStable Called with STATIC_EQUILIBRIUM_ALGORITHM_PP");
    bool isStable(false);
    if (fullbody->staticStability()) {
      status = staticEquilibrium.checkRobustEquilibrium(comComputed, isStable);
    } else {
      status = staticEquilibrium.checkRobustEquilibrium(comComputed, acc, isStable);
    }
    res =
        isStable ? std::numeric_limits<double>::max() : -1.;  // FIXME robustness not implemented with PP algorithm ...
  } else                                                      // STATIC_EQUILIBRIUM_ALGORITHM_DLP
  {
    if (fullbody->staticStability()) {
      status = staticEquilibrium.computeEquilibriumRobustness(comComputed, res);
      hppDout(notice, "isStable Called with staticStability");
    } else {
      status = staticEquilibrium.computeEquilibriumRobustness(comComputed, acc, res);
      hppDout(notice, "isStable : config = " << pinocchio::displayConfig(state.configuration_));
      hppDout(notice, "isStable : COM = " << comComputed.transpose());
      hppDout(notice, "isStable : acc = " << acc);
    }
  }
#ifdef PROFILE
  watch.stop("test balance");
#endif
  if (status != LP_STATUS_OPTIMAL) {
    if (status == LP_STATUS_UNBOUNDED) hppDout(notice, "isStable : lp unbounded");
    if (status == LP_STATUS_INFEASIBLE || status == LP_STATUS_UNBOUNDED) {
      // return 1.1; // completely arbitrary: TODO
      // hppDout(notice,"isStable LP infeasible");
      return -1.1;  // completely arbitrary: TODO
    }
    // hppDout(notice,"isStable error in LP");
    return -std::numeric_limits<double>::max();
  }
  hppDout(notice, "isStable LP successfully solved : robustness = " << res);
  /*    centroidal_dynamics::MatrixXX Hrow; VectorX h;
      staticEquilibrium.getPolytopeInequalities(Hrow,h);
      MatrixXX H = -Hrow;
      H.rowwise().normalize();
      int dimH = (int)(H.rows());
      hppDout(notice,"Dim H rows : "<<dimH<<" ; col : "<<H.cols());
      hppDout(notice,"H = "<<H);
      hppDout(notice,"h = "<<h);*/
  return res;
}
}  // namespace stability
}  // namespace rbprm
}  // namespace hpp
