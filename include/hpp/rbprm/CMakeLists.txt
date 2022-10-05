set(${PROJECT_NAME}_HEADERS
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
    rbprm-profiler.hh)

install(FILES ${${PROJECT_NAME}_HEADERS} DESTINATION include/hpp/rbprm)

add_subdirectory(contact_generation)
add_subdirectory(interpolation)
add_subdirectory(projection)
add_subdirectory(stability)
add_subdirectory(utils)
add_subdirectory(dynamic)
add_subdirectory(planner)
add_subdirectory(sampling)
