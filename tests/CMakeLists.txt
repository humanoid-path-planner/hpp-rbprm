# Copyright 2012-2014, 2020 CNRS-LAAS
#
# Author: Mathieu Geisert, Guilhem Saurel
#
# This file is part of hpp-core
# hpp-model-urdf is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# hpp-core is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with hpp-core  If not, see <http://www.gnu.org/licenses/>.

SET(${PROJECT_NAME}_TESTS
  #device
  #rbprm-shooter
  #sampling
  #fullbody
  #interpolate
  #contact-gen
  reachability
  rbrrt
  projection
  kinodynamic
  limb-rrt
  )


FOREACH(NAME ${${PROJECT_NAME}_TESTS})
  ADD_UNIT_TEST(${NAME} "test-${NAME}")
  TARGET_LINK_LIBRARIES(${NAME} ${PROJECT_NAME})
ENDFOREACH(NAME ${${PROJECT_NAME}_TESTS})
