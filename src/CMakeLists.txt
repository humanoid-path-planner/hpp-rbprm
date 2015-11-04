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
	rbprm-path-interpolation.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/rbprm-path-interpolation.hh
	rbprm-fullbody.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/rbprm-fullbody.hh
	${PROJECT_SOURCE_DIR}/include/hpp/rbprm/rbprm-state.hh
	sampling/sample.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/sampling/sample.hh
	sampling/sample-container.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/sampling/sample-container.hh
        sampling/heuristic.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/sampling/heuristic.hh
        tools.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/tools.hh
	stability/stability.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/stability/stability.hh
	stability/support.cc ${PROJECT_SOURCE_DIR}/include/hpp/rbprm/stability/support.hh
  )

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
ADD_LIBRARY(${LIBRARY_NAME}
	SHARED
  ${${LIBRARY_NAME}_SOURCES}
  )

TARGET_LINK_LIBRARIES(${LIBRARY_NAME} robust-equilibrium-lib polytope)

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-util)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-statistics)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-constraints)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-fcl)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-model)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} polytope)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
