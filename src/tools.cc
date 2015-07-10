// Copyright (c) 2014, LAAS-CNRS
// Authors: Steve Tonneau (steve.tonneau@laas.fr)
//
// This file is part of hpp-rbprm.
// hpp-rbprm is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-rbprm is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-rbprm. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/rbprm/tools.hh>

namespace hpp {
  namespace tools {
    fcl::Transform3f GetRotationMatrix(const fcl::Vec3f& from, const fcl::Vec3f& to)
    {
        fcl::Vec3f u, v, uXv, a;
        u = from; u.normalize();
        v = to  ; v.normalize();
        uXv = u.cross(v);
        numeric sinTheta = uXv.norm();
        if (sinTheta == 0) // angle is 0
        {
            result = Matrix3::Identity();
        }
        else
        {
            numeric cosTheta = u.dot(v);
            a = uXv / sinTheta;

            Matrix3 I = Matrix3::Identity();

            Matrix3 Iaaa = Matrix3::Zero();

            Iaaa(0,1) = - a(2); Iaaa(0,2) =  a(1); //  0  -z   y
            Iaaa(1,0) =   a(2); Iaaa(1,2) = -a(0); //  z   0  -x
            Iaaa(2,0) = - a(1); Iaaa(2,1) =  a(0); // -y   x   0

            result = I * cosTheta + sinTheta * Iaaa + (1 - cosTheta) * (a*a.transpose());
        }
    }
  } // model
} //hpp
