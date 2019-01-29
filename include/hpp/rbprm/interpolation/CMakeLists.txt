SET(${PROJECT_NAME}_INTERPOLATION_HEADERS
  rbprm-path-interpolation.hh
  time-constraint-helper.hh
  time-constraint-helper.inl
  time-constraint-steering.hh
  limb-rrt.hh
  com-rrt.hh
  com-rrt-shooter.hh
  time-constraint-shooter.hh
  limb-rrt-shooter.hh
  time-constraint-utils.hh
  time-constraint-path-validation.hh
  time-constraint-path.hh
  com-trajectory.hh
  polynom-trajectory.hh
  time-dependant.hh
  interpolation-constraints.hh
  )

INSTALL(FILES
  ${${PROJECT_NAME}_INTERPOLATION_HEADERS}
  DESTINATION include/hpp/rbprm/interpolation
  )

ADD_SUBDIRECTORY(spline)
