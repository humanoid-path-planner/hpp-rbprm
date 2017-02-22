#
# Copyright (c) 2010, 2011 CNRS
# Authors: Steve Tonneau
#
#
# This file is part of hpp-rbprm
# hpp-rbprm is free software: you can redistribute it
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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME hpp-rbprm)
SET(PROJECT_DESCRIPTION
   "Implementation of RB-PRM planner using hpp."
 )
 SET(PROJECT_URL "")


# Set to 1 for profiling
 # add_definitions(-DPROFILE)

# Where to compile shared objects
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

 SETUP_PROJECT()
 ADD_REQUIRED_DEPENDENCY("hpp-core >= 3")
 ADD_REQUIRED_DEPENDENCY("hpp-util >= 3")
 ADD_REQUIRED_DEPENDENCY("hpp-statistics")
 ADD_REQUIRED_DEPENDENCY("hpp-fcl")
 ADD_REQUIRED_DEPENDENCY("hpp-constraints >= 3")
 ADD_REQUIRED_DEPENDENCY("hpp-model >= 3")
 ADD_REQUIRED_DEPENDENCY("hpp-model-urdf >= 3")

FIND_PACKAGE( OpenMP REQUIRED)
find_package(Boost)
find_package(Boost 1.54.0 COMPONENTS filesystem system REQUIRED)

if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# remove flag that makes all warnings into errors

MESSAGE( STATUS "CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS} )

# Declare Headers
SET(${PROJECT_NAME}_HEADERS
    include/hpp/rbprm/rbprm-device.hh
    include/hpp/rbprm/rbprm-fullbody.hh
    include/hpp/rbprm/rbprm-limb.hh
		include/hpp/rbprm/projection/projection.hh
		include/hpp/rbprm/contact_generation/contact_generation.hh
		include/hpp/rbprm/contact_generation/algorithm.hh
    include/hpp/rbprm/interpolation/rbprm-path-interpolation.hh
    include/hpp/rbprm/interpolation/time-constraint-helper.hh
    include/hpp/rbprm/interpolation/time-constraint-helper.inl
    include/hpp/rbprm/interpolation/time-constraint-steering.hh
    include/hpp/rbprm/interpolation/limb-rrt.hh
    include/hpp/rbprm/interpolation/com-rrt.hh
    include/hpp/rbprm/interpolation/com-rrt-shooter.hh
    include/hpp/rbprm/interpolation/time-constraint-shooter.hh
    include/hpp/rbprm/interpolation/limb-rrt-shooter.hh
    include/hpp/rbprm/interpolation/time-constraint-utils.hh
    include/hpp/rbprm/interpolation/time-constraint-path-validation.hh
    include/hpp/rbprm/interpolation/time-constraint-path.hh
    include/hpp/rbprm/interpolation/com-trajectory.hh
    include/hpp/rbprm/interpolation/time-dependant.hh
    include/hpp/rbprm/interpolation/interpolation-constraints.hh
    include/hpp/rbprm/interpolation/spline/effector-rrt.hh
    include/hpp/rbprm/rbprm-shooter.hh
    include/hpp/rbprm/rbprm-state.hh
    include/hpp/rbprm/rbprm-validation.hh
    include/hpp/rbprm/rbprm-validation-report.hh
    include/hpp/rbprm/rbprm-path-validation.hh
    include/hpp/rbprm/rbprm-rom-validation.hh
    include/hpp/rbprm/sampling/sample.hh
    include/hpp/rbprm/sampling/sample-db.hh
    include/hpp/rbprm/sampling/heuristic.hh
    include/hpp/rbprm/sampling/analysis.hh
    include/hpp/rbprm/stability/stability.hh
    include/hpp/rbprm/stability/support.hh
    include/hpp/rbprm/tools.hh
    include/hpp/rbprm/ik-solver.hh
    include/hpp/rbprm/rbprm-profiler.hh
    include/utils/Stdafx.hh
    include/utils/stop-watch.hh
    include/spline/curve_abc.h
    include/spline/bezier_curve.h
    include/spline/exact_cubic.h
    include/spline/cubic_function.h
    include/spline/MathDefs.h
#    include/hpp/rbprm/planner/parabola-planner.hh
    include/hpp/rbprm/planner/dynamic-planner.hh
    include/hpp/rbprm/planner/steering-dynamic.hh
    include/hpp/rbprm/planner/parabola-path.hh
    include/hpp/rbprm/planner/timed-parabola-path.hh
    include/hpp/rbprm/planner/steering-method-parabola.hh
    include/hpp/rbprm/planner/rbprm-node.hh
    include/hpp/rbprm/planner/rbprm-roadmap.hh
    include/hpp/rbprm/planner/rbprm-steering-kinodynamic.hh
    include/utils/algorithms.h
  )

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()
# Activate hpp-util logging if requested
SET (HPP_BENCHMARK FALSE CACHE BOOL "trigger hpp-util benchmark output")
IF (HPP_BENCHMARK)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_ENABLE_BENCHMARK")
ENDIF()

ADD_SUBDIRECTORY(src)
#ADD_SUBDIRECTORY(tools)
ADD_SUBDIRECTORY(tests)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
