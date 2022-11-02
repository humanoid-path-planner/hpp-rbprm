set(${PROJECT_NAME}_SAMPLING_HEADERS sample.hh sample-db.hh heuristic-tools.hh
                                     heuristic.hh analysis.hh)

install(FILES ${${PROJECT_NAME}_SAMPLING_HEADERS}
        DESTINATION include/hpp/rbprm/sampling)
