SET(${PROJECT_NAME}_SAMPLING_HEADERS
  sample.hh
  sample-db.hh
  heuristic-tools.hh
  heuristic.hh
  analysis.hh
  )


INSTALL(FILES
  ${${PROJECT_NAME}_SAMPLING_HEADERS}
  DESTINATION include/hpp/rbprm/sampling
  )
