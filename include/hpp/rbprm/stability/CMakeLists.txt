set(${PROJECT_NAME}_STABILITY_HEADERS stability.hh support.hh)

install(FILES ${${PROJECT_NAME}_STABILITY_HEADERS}
        DESTINATION include/hpp/rbprm/stability)
