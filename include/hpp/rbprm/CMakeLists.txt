
SET(${PROJECT_NAME}_HEADERS
  rbprm-device.hh
  rbprm-fullbody.hh
  rbprm-limb.hh
  reports.hh
  rbprm-shooter.hh
  rbprm-state.hh
  rbprm-validation.hh
  rbprm-validation-report.hh
  rbprm-path-validation.hh
  rbprm-rom-validation.hh
  tools.hh
  rbprm-profiler.hh
  )

INSTALL(FILES
  ${${PROJECT_NAME}_HEADERS}
  DESTINATION include/hpp/rbprm
  )

ADD_SUBDIRECTORY(contact_generation)
ADD_SUBDIRECTORY(interpolation)
ADD_SUBDIRECTORY(projection)
ADD_SUBDIRECTORY(stability)
ADD_SUBDIRECTORY(utils)
ADD_SUBDIRECTORY(dynamic)
ADD_SUBDIRECTORY(planner)
ADD_SUBDIRECTORY(sampling)
