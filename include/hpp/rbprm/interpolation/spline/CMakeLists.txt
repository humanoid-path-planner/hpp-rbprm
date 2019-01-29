SET(${PROJECT_NAME}_INTERPOLATION_SPLINE_HEADERS
  effector-rrt.hh
  bezier-path.hh
  )

INSTALL(FILES
  ${${PROJECT_NAME}_INTERPOLATION_SPLINE_HEADERS}
  DESTINATION include/hpp/rbprm/interpolation/spline
  )
