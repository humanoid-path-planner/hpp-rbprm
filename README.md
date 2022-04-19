#  Humanoid Path Planner - RBPRM module

[![Pipeline status](https://gitlab.laas.fr/humanoid-path-planner/hpp-rbprm/badges/master/pipeline.svg)](https://gitlab.laas.fr/humanoid-path-planner/hpp-rbprm/commits/master)
[![Coverage report](https://gitlab.laas.fr/humanoid-path-planner/hpp-rbprm/badges/master/coverage.svg?job=doc-coverage)](https://gepettoweb.laas.fr/doc/humanoid-path-planner/hpp-rbprm/master/coverage/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/humanoid-path-planner/hpp-rbprm/master.svg)](https://results.pre-commit.ci/latest/github/humanoid-path-planner/hpp-rbprm)

Copyright 2015-2020 LAAS-CNRS

Authors: Steve Tonneau, Pierre Fernbach

## Description
HPP - RBPRM is a library written for the software Humanoid Path Planner [(link)](http://projects.laas.fr/gepetto/index.php/Software/Hpp).
It implements the acyclic contact planner presented in two papers [(link)](http://stevetonneau.fr/files/publications/isrr15/isrr15.html):

"An efficient acyclic contact planner for multiped robots" (submitted to IJRR).

"A Reachability-based planner for sequences of acyclic contacts in cluttered environments" (presented at ISRR 15).

We recommend reading the IJRR submission before going using RB-PRM.

The planner has applications in both robotics and computer graphics applications (click on the pictures to watch videos):

[![IMAGE ALT TEXT](http://img.youtube.com/vi/K3ivZe0AS68/0.jpg)](http://www.youtube.com/watch?v=K3ivZe0AS68 "An efficient acyclic contact planner for multiped robots")
[![IMAGE ALT TEXT](http://img.youtube.com/vi/YjL-DBQgXwk/0.jpg)](http://www.youtube.com/watch?v=YjL-DBQgXwk "https://www.youtube.com/watch?v=YjL-DBQgXwk")
[![IMAGE ALT TEXT](http://img.youtube.com/vi/NhvL8jWlka0/0.jpg)](http://www.youtube.com/watch?v=NhvL8jWlka0 "Character contact re-positioning under large environment deformation")


## Installation

### From binary

This package is available as binary in [robotpkg/wip](http://robotpkg.openrobots.org/robotpkg-wip.html). Set up your system to add robotpkg/wip repository, then run:

```
apt-get install robotpkg-pyXX-hpp-rbprm
```
Replace pyXX with your python version.


### From sources

Follow the instructions [here](http://humanoid-path-planner.github.io/hpp-doc/download.html). After step 7 run:
```
make rbprm.install
```

## Documentation

  Open $DEVEL_HPP_DIR/install/share/doc/hpp-rbprm/doxygen-html/index.html in a web brower and you
  will have access to the code documentation.
