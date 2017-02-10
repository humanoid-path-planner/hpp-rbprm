#
# Copyright (c) 2010, 2011 CNRS
# Authors: Florent Lamiraux
#
#
# This file is part of hpp-core
# hpp-core is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-core is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-core  If not, see
# <http://www.gnu.org/licenses/>.

SET(LIBRARY_NAME ${PROJECT_NAME})

SET(${LIBRARY_NAME}_SOURCES
	rbprm-shooter.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/rbprm-shooter.hh
	rbprm-validation.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/rbprm-validation.hh
        rbprm-rom-validation.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/rbprm-rom-validation.hh
	rbprm-device.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/rbprm-device.hh
	rbprm-limb.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/rbprm-limb.hh
        ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/interpolation/time-dependant.hh
        ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/interpolation/interpolation-constraints.hh
        ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/interpolation/time-constraint-helper.hh
        ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/interpolation/time-constraint-helper.inl
        ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/interpolation/time-constraint-utils.hh
        interpolation/effector-rrt.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/interpolation/spline/effector-rrt.hh
        interpolation/rbprm-path-interpolation.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/interpolation/rbprm-path-interpolation.hh
        interpolation/time-constraint-shooter.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/interpolation/time-constraint-shooter.hh
        interpolation/limb-rrt-shooter.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/interpolation/limb-rrt-shooter.hh
        interpolation/com-rrt-shooter.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/interpolation/com-rrt-shooter.hh
				projection/projection.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/projection/projection.hh
				contact_generation/contact_generation.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/contact_generation/contact_generation.hh
        ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/interpolation/limb-rrt.hh
        interpolation/limb-rrt.cc
        ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/interpolation/com-rrt.hh
        interpolation/com-rrt.cc
        interpolation/time-constraint-path-validation.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/interpolation/time-constraint-path-validation.hh
        ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/interpolation/time-constraint-steering.hh
        interpolation/time-constraint-path.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/interpolation/time-constraint-path.hh
        interpolation/com-trajectory.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/interpolation/com-trajectory.hh
        rbprm-fullbody.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/rbprm-fullbody.hh
        rbprm-state.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/rbprm-state.hh
        sampling/sample.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/sampling/sample.hh
        sampling/analysis.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/sampling/analysis.hh
        sampling/heuristic.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/sampling/heuristic.hh
        sampling/sample-db.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/sampling/sample-db.hh
        tools.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/tools.hh
        stability/stability.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/stability/stability.hh
        stability/support.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/stability/support.hh
        ik-solver.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/ik-solver.hh
        utils/stop-watch.cc ${PROJECT_SOURCE_DIR}/include/utils/stop-watch.hh
        rbprm-profiler.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/rbprm-profiler.hh
        ${PROJECT_SOURCE_DIR}/include/spline/curve_abc.h
        ${PROJECT_SOURCE_DIR}/include/spline/bezier_curve.h
        ${PROJECT_SOURCE_DIR}/include/spline/exact_cubic.h
        ${PROJECT_SOURCE_DIR}/include/spline/cubic_function.h
        ${PROJECT_SOURCE_DIR}/include/spline/MathDefs.h
  )

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
ADD_LIBRARY(${LIBRARY_NAME}
	SHARED
  ${${LIBRARY_NAME}_SOURCES}
  )

TARGET_LINK_LIBRARIES(${LIBRARY_NAME} robust-equilibrium-lib)

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-util)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-statistics)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-constraints)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-fcl)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-model)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
