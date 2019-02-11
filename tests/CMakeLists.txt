# Copyright 2012, 2013, 2014 CNRS-LAAS
#
# Author: Mathieu Geisert
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
  #reachability
  rbrrt
  projection
  kinodynamic
  )

FOREACH(NAME ${${PROJECT_NAME}_TESTS})
  ADD_UNIT_TEST(${NAME} "test-${NAME}")

  PKG_CONFIG_USE_DEPENDENCY(${NAME} hpp-core)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} hpp-fcl)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} hpp-affordance)
  TARGET_LINK_LIBRARIES(${NAME} hpp-affordance)

  # Link against Boost and project library.
  TARGET_LINK_LIBRARIES(${NAME}
    ${Boost_LIBRARIES}
    ${PROJECT_NAME}
    hpp-centroidal-dynamics
    hpp-bezier-com-traj
    )
ENDFOREACH(NAME ${${PROJECT_NAME}_TESTS})
