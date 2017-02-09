#  Humanoid Path Planner - RBPRM module

Copyright 2015 LAAS-CNRS

Author: Steve Tonneau

##Description
HPP - RBPRM is a library written for the software Humanoid Path Planner [(link)](http://projects.laas.fr/gepetto/index.php/Software/Hpp).
It implements the acyclic contact planner presented in two papers [(link)](http://stevetonneau.fr/files/publications/isrr15/isrr15.html):

"An efficient acyclic contact planner for multiped robots" (submitted to IJRR).


"A Reachability-based planner for sequences of acyclic contacts in cluttered environments" (presented at ISRR 15).

We recommend reading the IJRR submission before going using RB-PRM.

The planner has applications in both robotics and computer graphics applications (click on the pictures to watch videos):

[![IMAGE ALT TEXT](http://img.youtube.com/vi/K3ivZe0AS68/0.jpg)](http://www.youtube.com/watch?v=K3ivZe0AS68 "An efficient acyclic contact planner for multiped robots")
[![IMAGE ALT TEXT](http://img.youtube.com/vi/YjL-DBQgXwk/0.jpg)](http://www.youtube.com/watch?v=YjL-DBQgXwk "https://www.youtube.com/watch?v=YjL-DBQgXwk")
[![IMAGE ALT TEXT](http://img.youtube.com/vi/NhvL8jWlka0/0.jpg)](http://www.youtube.com/watch?v=NhvL8jWlka0 "Character contact re-positioning under large environment deformation")

##Installation on ubuntu-14.04 64 bit with ros-indigo

To install HPP-RBPRM: 

  1. install HPP 
	- see https://github.com/humanoid-path-planner/hpp-doc
	- IMPORTANT: you should use the devel branch of the project for the software to work correctly

  2. install hpp-affordance, and hpp-affordance corba, comprising terrain analysis tools
	- see https://github.com/humanoid-path-planner/hpp-affordance

  3. install robust-equilibrium-lib, a library for quickly asserting static equilibrium
	- see https://github.com/andreadelprete/robust-equilibrium-lib

 
  4. Use CMake to install the library. For instance:

				mkdir $HPP_RBPRM_DIR/build
				cd $HPP_RBPRM_DIR/build
				cd cmake ..
				make install

  5. Optionally, install the python bindings for python, and example scripts (HPP-RBPRM-CORBA)
	- see https://github.com/stonneau/hpp-rbprm-corba
  

##Documentation

  Open $DEVEL_DIR/install/share/doc/hpp-rbprm/doxygen-html/index.html in a web brower and you
  will have access to the code documentation.
