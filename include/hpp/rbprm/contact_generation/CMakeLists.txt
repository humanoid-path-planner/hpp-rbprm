SET(${PROJECT_NAME}_CONTACT_HEADERS
  algorithm.hh
  contact_generation.hh
  reachability.hh
  kinematics_constraints.hh
  )

INSTALL(FILES
  ${${PROJECT_NAME}_CONTACT_HEADERS}
  DESTINATION include/hpp/rbprm/contact_generation
  )
