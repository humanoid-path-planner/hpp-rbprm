SET(${PROJECT_NAME}_STABILITY_HEADERS
  stability.hh
  support.hh
  )

INSTALL(FILES
  ${${PROJECT_NAME}_STABILITY_HEADERS}
  DESTINATION include/hpp/rbprm/stability
  )
