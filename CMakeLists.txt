#
# Copyright (c) 2010, 2011, 2020 CNRS Authors: Steve Tonneau, Guilhem Saurel
#
# This file is part of hpp-rbprm hpp-rbprm is free software: you can
# redistribute it and/or modify it under the terms of the GNU Lesser General
# Public License as published by the Free Software Foundation, either version 3
# of the License, or (at your option) any later version.
#
# hpp-core is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE.  See the GNU General Lesser Public License for more
# details.  You should have received a copy of the GNU Lesser General Public
# License along with hpp-core  If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME hpp-rbprm)
set(PROJECT_DESCRIPTION "Implementation of RB-PRM planner using hpp.")

set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR TRUE)

include(cmake/hpp.cmake)

compute_project_args(PROJECT_ARGS LANGUAGES CXX C)
project(${PROJECT_NAME} ${PROJECT_ARGS})

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/find-external/CDD")
find_package(CDD REQUIRED)
find_package(OpenMP REQUIRED)

add_project_dependency("hpp-core" REQUIRED)
add_project_dependency("hpp-bezier-com-traj" REQUIRED)
add_project_dependency("hpp-affordance" REQUIRED)
add_project_dependency("hpp-environments" REQUIRED)
add_project_dependency("anymal-rbprm" REQUIRED)
add_project_dependency("hyq-rbprm" REQUIRED)
add_project_dependency("simple-humanoid-rbprm" REQUIRED)
add_project_dependency("talos-rbprm" REQUIRED)
add_project_dependency("example-robot-data" REQUIRED)
add_project_dependency("ndcurves" REQUIRED)

if(OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS
      "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

message(STATUS "CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS})

add_required_dependency("octomap >= 1.6")
if(OCTOMAP_INCLUDE_DIRS AND OCTOMAP_LIBRARY_DIRS)
  include_directories(${OCTOMAP_INCLUDE_DIRS})
  link_directories(${OCTOMAP_LIBRARY_DIRS})
  string(REPLACE "." ";" VERSION_LIST ${OCTOMAP_VERSION})
  list(GET VERSION_LIST 0 OCTOMAP_MAJOR_VERSION)
  list(GET VERSION_LIST 1 OCTOMAP_MINOR_VERSION)
  list(GET VERSION_LIST 2 OCTOMAP_PATCH_VERSION)
  add_definitions(
    -DOCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION}
    -DOCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION}
    -DOCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION} -DHPP_FCL_HAVE_OCTOMAP=1)
  message(STATUS "FCL uses Octomap" ${OCTOMAP_MINOR_VERSION})
else()
  message(STATUS "FCL does not use Octomap")
endif()

set(${PROJECT_NAME}_HEADERS
    include/hpp/rbprm/rbprm-device.hh
    include/hpp/rbprm/rbprm-fullbody.hh
    include/hpp/rbprm/rbprm-limb.hh
    include/hpp/rbprm/reports.hh
    include/hpp/rbprm/rbprm-shooter.hh
    include/hpp/rbprm/rbprm-state.hh
    include/hpp/rbprm/rbprm-validation.hh
    include/hpp/rbprm/rbprm-validation-report.hh
    include/hpp/rbprm/rbprm-path-validation.hh
    include/hpp/rbprm/rbprm-rom-validation.hh
    include/hpp/rbprm/tools.hh
    include/hpp/rbprm/rbprm-profiler.hh
    include/hpp/rbprm/contact_generation/algorithm.hh
    include/hpp/rbprm/contact_generation/contact_generation.hh
    include/hpp/rbprm/contact_generation/reachability.hh
    include/hpp/rbprm/contact_generation/kinematics_constraints.hh
    include/hpp/rbprm/dynamic/dynamic-validation.hh
    include/hpp/rbprm/dynamic/dynamic-path-validation.hh
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
    include/hpp/rbprm/interpolation/polynom-trajectory.hh
    include/hpp/rbprm/interpolation/time-dependant.hh
    include/hpp/rbprm/interpolation/interpolation-constraints.hh
    include/hpp/rbprm/interpolation/spline/effector-rrt.hh
    include/hpp/rbprm/interpolation/spline/bezier-path.hh
    # include/hpp/rbprm/planner/parabola-planner.hh
    include/hpp/rbprm/planner/dynamic-planner.hh
    include/hpp/rbprm/planner/parabola-path.hh
    include/hpp/rbprm/planner/timed-parabola-path.hh
    include/hpp/rbprm/planner/steering-method-parabola.hh
    include/hpp/rbprm/planner/rbprm-node.hh
    include/hpp/rbprm/planner/rbprm-roadmap.hh
    include/hpp/rbprm/planner/rbprm-steering-kinodynamic.hh
    include/hpp/rbprm/planner/random-shortcut-dynamic.hh
    include/hpp/rbprm/planner/oriented-path-optimizer.hh
    include/hpp/rbprm/projection/projection.hh
    include/hpp/rbprm/sampling/sample.hh
    include/hpp/rbprm/sampling/sample-db.hh
    include/hpp/rbprm/sampling/heuristic-tools.hh
    include/hpp/rbprm/sampling/heuristic.hh
    include/hpp/rbprm/sampling/analysis.hh
    include/hpp/rbprm/stability/stability.hh
    include/hpp/rbprm/stability/support.hh
    include/hpp/rbprm/utils/Stdafx.hh
    include/hpp/rbprm/utils/stop-watch.hh
    include/hpp/rbprm/utils/algorithms.h)

set(${PROJECT_NAME}_SOURCES
    src/rbprm-shooter.cc
    src/rbprm-validation.cc
    src/rbprm-path-validation.cc
    src/rbprm-rom-validation.cc
    src/rbprm-device.cc
    src/rbprm-limb.cc
    src/interpolation/interpolation-constraints.cc
    src/interpolation/effector-rrt.cc
    src/interpolation/bezier-path.cc
    src/interpolation/rbprm-path-interpolation.cc
    src/interpolation/time-constraint-shooter.cc
    src/interpolation/limb-rrt-shooter.cc
    src/interpolation/com-rrt-shooter.cc
    src/projection/projection.cc
    src/contact_generation/contact_generation.cc
    src/contact_generation/kinematics_constraints.cc
    src/contact_generation/algorithm.cc
    src/contact_generation/reachability.cc
    src/interpolation/limb-rrt.cc
    src/interpolation/com-rrt.cc
    src/interpolation/time-constraint-path-validation.cc
    src/interpolation/time-constraint-path.cc
    src/interpolation/com-trajectory.cc
    src/interpolation/polynom-trajectory.cc
    src/rbprm-fullbody.cc
    src/rbprm-state.cc
    src/sampling/sample.cc
    src/sampling/analysis.cc
    src/sampling/heuristic-tools.cc
    src/sampling/heuristic.cc
    src/sampling/sample-db.cc
    src/tools.cc
    src/stability/stability.cc
    src/stability/support.cc
    src/utils/stop-watch.cc
    src/utils/algorithms.cc
    src/rbprm-profiler.cc
    # src/planner/parabola-planner.cc
    src/planner/steering-method-parabola.cc
    src/planner/parabola-path.cc
    src/planner/timed-parabola-path.cc
    src/planner/dynamic-planner.cc
    src/planner/rbprm-steering-kinodynamic.cc
    src/planner/rbprm-node.cc
    src/dynamic/dynamic-validation.cc
    src/dynamic/dynamic-path-validation.cc
    src/planner/random-shortcut-dynamic.cc
    src/planner/oriented-path-optimizer.cc)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} PUBLIC ${CDD_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} hpp-bezier-com-traj::hpp-bezier-com-traj
                      hpp-affordance::hpp-affordance hpp-core::hpp-core)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

# ADD_SUBDIRECTORY(tools)
add_subdirectory(tests)

pkg_config_append_libs(${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
