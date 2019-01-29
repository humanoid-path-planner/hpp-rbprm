SET(${PROJECT_NAME}_DYNAMIC_HEADERS
  dynamic-validation.hh
  dynamic-path-validation.hh
  )

INSTALL(FILES
  ${${PROJECT_NAME}_DYNAMIC_HEADERS}
  DESTINATION include/hpp/rbprm/dynamic
  )
