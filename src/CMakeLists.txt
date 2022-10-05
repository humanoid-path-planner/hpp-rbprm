#
# Copyright (c) 2010, 2011 CNRS Authors: Florent Lamiraux
#
# This file is part of hpp-core hpp-core is free software: you can redistribute
# it and/or modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of the License,
# or (at your option) any later version.
#
# hpp-core is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE.  See the GNU General Lesser Public License for more
# details.  You should have received a copy of the GNU Lesser General Public
# License along with hpp-core  If not, see <http://www.gnu.org/licenses/>.

set(LIBRARY_NAME ${PROJECT_NAME})

set(${LIBRARY_NAME}_SOURCES
    rbprm-shooter.cc
    rbprm-validation.cc
    rbprm-path-validation.cc
    rbprm-rom-validation.cc
    rbprm-device.cc
    rbprm-limb.cc
    interpolation/interpolation-constraints.cc
    interpolation/effector-rrt.cc
    interpolation/bezier-path.cc
    interpolation/rbprm-path-interpolation.cc
    interpolation/time-constraint-shooter.cc
    interpolation/limb-rrt-shooter.cc
    interpolation/com-rrt-shooter.cc
    projection/projection.cc
    contact_generation/contact_generation.cc
    contact_generation/kinematics_constraints.cc
    contact_generation/algorithm.cc
    contact_generation/reachability.cc
    interpolation/limb-rrt.cc
    interpolation/com-rrt.cc
    interpolation/time-constraint-path-validation.cc
    interpolation/time-constraint-path.cc
    interpolation/com-trajectory.cc
    interpolation/polynom-trajectory.cc
    rbprm-fullbody.cc
    rbprm-state.cc
    sampling/sample.cc
    sampling/analysis.cc
    sampling/heuristic-tools.cc
    sampling/heuristic.cc
    sampling/sample-db.cc
    tools.cc
    stability/stability.cc
    stability/support.cc
    utils/stop-watch.cc
    utils/algorithms.cc
    rbprm-profiler.cc
    # planner/parabola-planner.cc
    planner/steering-method-parabola.cc
    planner/parabola-path.cc
    planner/timed-parabola-path.cc
    planner/dynamic-planner.cc
    planner/rbprm-steering-kinodynamic.cc
    planner/rbprm-node.cc
    dynamic/dynamic-validation.cc
    dynamic/dynamic-path-validation.cc
    planner/random-shortcut-dynamic.cc
    planner/oriented-path-optimizer.cc)

add_library(${LIBRARY_NAME} SHARED ${${LIBRARY_NAME}_SOURCES})

target_link_libraries(${LIBRARY_NAME} hpp-centroidal-dynamics
                      hpp-bezier-com-traj)
target_include_directories(${LIBRARY_NAME} PUBLIC ${CDD_INCLUDE_DIRS})

pkg_config_use_dependency(${LIBRARY_NAME} hpp-core)
pkg_config_use_dependency(${LIBRARY_NAME} hpp-util)
pkg_config_use_dependency(${LIBRARY_NAME} hpp-statistics)
pkg_config_use_dependency(${LIBRARY_NAME} hpp-constraints)
pkg_config_use_dependency(${LIBRARY_NAME} hpp-fcl)
pkg_config_use_dependency(${LIBRARY_NAME} hpp-pinocchio)
pkg_config_use_dependency(${LIBRARY_NAME} ndcurves)
pkg_config_use_dependency(${LIBRARY_NAME} pinocchio)

install(TARGETS ${LIBRARY_NAME} DESTINATION lib)
