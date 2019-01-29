SET(${PROJECT_NAME}_PLANNER_HEADERS
  #parabola-planner.hh
  dynamic-planner.hh
  parabola-path.hh
  timed-parabola-path.hh
  steering-method-parabola.hh
  rbprm-node.hh
  rbprm-roadmap.hh
  rbprm-steering-kinodynamic.hh
  random-shortcut-dynamic.hh
  oriented-path-optimizer.hh
  )

INSTALL(FILES
  ${${PROJECT_NAME}_PLANNER_HEADERS}
  DESTINATION include/hpp/rbprm/planner
  )
