 CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
 INCLUDE(cmake/base.cmake)

 SET(PROJECT_NAME hpp-rbprm)
 SET(PROJECT_DESCRIPTION
   "Implementation of RB-PRM planner using hpp."
 )
 SET(PROJECT_URL "")

 SETUP_PROJECT()
 ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 3")

 # Create and install executable running the corba server
 ADD_EXECUTABLE (hpp-rbprm-server
   src/rbprm-main.cc
 )
 # Link executable with hpp-corbaserver library
 PKG_CONFIG_USE_DEPENDENCY (hpp-rbprm-server hpp-corbaserver)
 # Install executable
 INSTALL (TARGETS hpp-rbprm-server DESTINATION ${CMAKE_INSTALL_BINDIR})
 SETUP_PROJECT_FINALIZE()
