set(${PROJECT_NAME}_UTILS_HEADERS Stdafx.hh stop-watch.hh algorithms.h)

install(FILES ${${PROJECT_NAME}_UTILS_HEADERS}
        DESTINATION include/hpp/rbprm/utils)
