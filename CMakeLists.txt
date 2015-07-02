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
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME hpp-rbprm)
SET(PROJECT_DESCRIPTION
   "Implementation of RB-PRM planner using hpp."
 )
 SET(PROJECT_URL "")

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

# Declare Headers
SET(${PROJECT_NAME}_HEADERS
	include/hpp/rbprm/rbprm-device.hh
  include/hpp/rbprm/rbprm-shooter.hh
  include/hpp/rbprm/rbprm-validation.hh
	include/hpp/rbprm/sampling/sample.hh
	include/hpp/rbprm/sampling/sample-container.hh
  )

ADD_SUBDIRECTORY(src)
#ADD_SUBDIRECTORY(tools)
ADD_SUBDIRECTORY(tests)


SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
