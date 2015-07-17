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
#include <Eigen/Geometry>

namespace hpp {
  namespace tools {
    Eigen::Matrix3d GetRotationMatrix(const Eigen::Vector3d &from, const Eigen::Vector3d &to)
    {
        Eigen::Vector3d u, v, uXv;
        Eigen::Vector3d  a;
        u = from; u.normalize();
        v = to  ; v.normalize();
        uXv = u.cross(v);
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        double sinTheta = uXv.norm();
        if (sinTheta < std::numeric_limits<double>::epsilon()) // angle is 0
        {
            return I;
        }
        else
        {
            double cosTheta = u.dot(v);
            a = uXv / sinTheta;
            Eigen::Matrix3d Iaaa;
            Iaaa(0,1) = 0     ; Iaaa(0,1) = -a[2]; Iaaa(0,2) =  a[1]; //  0  -z   y
            Iaaa(1,0) =  a[2] ; Iaaa(1,0) = 0    ; Iaaa(1,2) = -a[0]; //  z   0  -x
            Iaaa(2,0) = -a[1] ; Iaaa(2,1) = a[0] ; Iaaa(2,2) =  0; // -y   x   0

            return I * cosTheta + sinTheta * Iaaa + (1 - cosTheta) * (a*a.transpose());
        }
    }
  } // model
} //hpp
