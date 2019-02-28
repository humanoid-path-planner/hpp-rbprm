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
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/test.cmake)

SET(PROJECT_NAME hpp-rbprm)
SET(PROJECT_DESCRIPTION
  "Implementation of RB-PRM planner using hpp."
  )

SETUP_HPP_PROJECT()

SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/find-external/CDD")
FIND_PACKAGE(CDD REQUIRED)

ADD_REQUIRED_DEPENDENCY("hpp-core >= 4.2")
ADD_REQUIRED_DEPENDENCY("hpp-util >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-statistics")
ADD_REQUIRED_DEPENDENCY("hpp-fcl")
ADD_REQUIRED_DEPENDENCY("hpp-constraints >= 4.1")
ADD_REQUIRED_DEPENDENCY("hpp-pinocchio >= 4.2")
ADD_REQUIRED_DEPENDENCY("pinocchio")
ADD_REQUIRED_DEPENDENCY("hpp-bezier-com-traj")
ADD_REQUIRED_DEPENDENCY("hpp-centroidal-dynamics")
ADD_REQUIRED_DEPENDENCY("hpp-affordance")
ADD_REQUIRED_DEPENDENCY("hpp-spline")

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

add_required_dependency("octomap >= 1.6")
if (OCTOMAP_INCLUDE_DIRS AND OCTOMAP_LIBRARY_DIRS)
  include_directories(${OCTOMAP_INCLUDE_DIRS})
  link_directories(${OCTOMAP_LIBRARY_DIRS})
  string(REPLACE "." ";" VERSION_LIST ${OCTOMAP_VERSION})
  list(GET VERSION_LIST 0 OCTOMAP_MAJOR_VERSION)
  list(GET VERSION_LIST 1 OCTOMAP_MINOR_VERSION)
  list(GET VERSION_LIST 2 OCTOMAP_PATCH_VERSION)
  add_definitions (-DOCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION}
    -DOCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION}
    -DOCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION} -DHPP_FCL_HAVE_OCTOMAP=1)
  message(STATUS "FCL uses Octomap" ${OCTOMAP_MINOR_VERSION})
else()
  message(STATUS "FCL does not use Octomap")
endif()

ADD_SUBDIRECTORY(include/hpp/rbprm)
ADD_SUBDIRECTORY(src)
#ADD_SUBDIRECTORY(tools)
ADD_SUBDIRECTORY(tests)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

SETUP_PROJECT_FINALIZE()
