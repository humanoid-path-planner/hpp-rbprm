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
#include <iostream>
#include <fstream>
#include <hpp/model/joint.hh>

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

    // TODO directly compute transformation, this is terrible
    fcl::Matrix3f GetRotationMatrix(const fcl::Vec3f& from, const fcl::Vec3f& to)
    {
        fcl::Matrix3f result;
        Eigen::Matrix3d resEigen = Eigen::Matrix3d::Identity();
        Eigen::Vector3d u, v, uXv;
        Eigen::Vector3d  a;
        for(int i =0; i<3; ++i)
        {
            u[i] = from[i];
            v[i] = to[i];
        }
        u.normalize();
        v.normalize();
        uXv = u.cross(v);
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        double sinTheta = uXv.norm();
        if (sinTheta < std::numeric_limits<double>::epsilon()) // angle is 0
        {
            resEigen = I; // identity
        }
        else
        {
            double cosTheta = u.dot(v);
            a = uXv / sinTheta;
            Eigen::Matrix3d Iaaa;
            Iaaa(0,1) = 0     ; Iaaa(0,1) = -a[2]; Iaaa(0,2) =  a[1]; //  0  -z   y
            Iaaa(1,0) =  a[2] ; Iaaa(1,0) = 0    ; Iaaa(1,2) = -a[0]; //  z   0  -x
            Iaaa(2,0) = -a[1] ; Iaaa(2,1) = a[0] ; Iaaa(2,2) =  0; // -y   x   0
            resEigen = I * cosTheta + sinTheta * Iaaa + (1 - cosTheta) * (a*a.transpose());
        }
        for(int i = 0; i<3;++i)
        {
            for(int j = 0; j<3;++j)
            {
                result(i,j) = resEigen(i,j);
            }
        }
        return result;
    }

    namespace io
    {
    double StrToD (const std::string &str)
    {
        std::istringstream ss(str);
        double result;
        return ss >> result ? result : 0;
    }

    int StrToI (const std::string &str)
    {
        std::istringstream ss(str);
        double result;
        return ss >> result ? (int)result : 0;
    }

    double StrToD (std::ifstream& input)
    {
        std::string line;
        getline(input, line);
        return StrToD(line);
    }

    int StrToI (std::ifstream& input)
    {
        std::string line;
        getline(input, line);
        return StrToI(line);
    }

    std::vector<std::string> splitString(const std::string& s, const char sep)
    {
       //Eclate une chane au niveau de ses ;.
       std::vector<std::string> ret;
       std::string s1="";
       for(unsigned int i=0;i<s.size();i++)
       {
           if(s[i]==sep||i==s.size()-1)
           {
               if(i==s.size()-1)
                   s1+=s[i];
               if(s1!="") ret.push_back(s1);
               s1="";
           }
           else
               s1+=s[i];
       }
       return ret;
    }

    void writeMatrix(const Eigen::MatrixXd& mat, std::ostream& output)
    {
        output << mat.rows()<<";"<<mat.cols()<<std::endl;
        for(int i =0; i< mat.rows(); ++i)
        {
            for(int j =0; j< mat.cols(); ++j)
            {
                output << mat(i,j)<<";";
            }
        }
    }

    Eigen::MatrixXd readMatrix(std::ifstream& myfile)
    {
        std::string line;
        return readMatrix(myfile,line);
    }

    Eigen::MatrixXd readMatrix(std::ifstream& myfile, std::string& line)
    {
        getline(myfile, line); //id
        std::vector<std::string> dim = splitString(line,';');
        int rows=StrToI(dim[0]);
        int cols=StrToI(dim[1]);
        Eigen::MatrixXd res(rows, cols);
        getline(myfile, line); //id
        std::vector<std::string> data = splitString(line,';');
        std::vector<std::string>::const_iterator current = data.begin();
        for(int i =0; i< rows; ++i)
        {
            for(int j =0; j< cols; ++j)
            {
                res(i,j) = StrToD(*current);
                ++current;
            }
        }
        return res;
    }


    void writeVecFCL(const fcl::Vec3f& vec, std::ostream& output)
    {
        for(int i =0; i< 3; ++i)
        {
            output << vec[i]<<";";
        }
    }

    fcl::Vec3f readVecFCL(std::ifstream& myfile)
    {
        std::string line;
        return readVecFCL(myfile,line);
    }

    fcl::Vec3f readVecFCL(std::ifstream& myfile, std::string& line)
    {
        fcl::Vec3f res;
        getline(myfile, line); //id
        std::vector<std::string> dim = splitString(line,';');
        for(int i =0; i< 3; ++i)
        {
            res[i] = StrToD(dim[i]);
        }
        return res;
    }

    void writeRotMatrixFCL(const fcl::Matrix3f& mat, std::ostream& output)
    {
        for(int i =0; i< 3; ++i)
        {
            for(int j =0; j< 3; ++j)
            {
                output << mat(i,j)<<";";
            }
        }
    }

    fcl::Matrix3f readRotMatrixFCL(std::ifstream& myfile)
    {
        std::string line;
        return readRotMatrixFCL(myfile,line);
    }

    fcl::Matrix3f readRotMatrixFCL(std::ifstream& myfile, std::string& line)
    {
        fcl::Matrix3f res;
        getline(myfile, line); //id
        std::vector<std::string> data = splitString(line,';');
        std::vector<std::string>::const_iterator current = data.begin();
        for(int i =0; i< 3; ++i)
        {
            for(int j =0; j< 3; ++j)
            {
                res(i,j) = StrToD(*current);
                ++current;
            }
        }
        return res;
    }

    } // io
  } // tools
} //hpp
