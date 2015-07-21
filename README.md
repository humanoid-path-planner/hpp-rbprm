Implementation of RB-PRM planner using hpp


TODO:

Assumes 6 DOF abstraction

Handle not only position but also rotation in effector contact point offset.
(especially when initializing default start and goal states in corba server)

Tests for fullbody contact generation

Tests for interpolation path

Implement RBPRM objects for HRP2,
and an automatic generation program.

Implement and test stability criterion.

Uniformize use of Vec3f vs Eigen

avoid always taking collision objects into parameters

Adding documentation.
