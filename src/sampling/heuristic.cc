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

#include <hpp/rbprm/sampling/heuristic.hh>

#include <Eigen/Eigen>

using namespace hpp;
using namespace hpp::model;
using namespace hpp::rbprm;
using namespace hpp::rbprm::sampling;

namespace
{
double EFORTHeuristic(const sampling::Sample* sample,
                      const Eigen::Vector3d& direction, const Eigen::Vector3d& normal)
{
    double EFORT = -direction.transpose() * sample->jacobianProduct_.block<3,3>(0,0) * (-direction);
    return EFORT * direction.dot(normal);
}

double ManipulabilityHeuristic(const sampling::Sample* sample,
                      const Eigen::Vector3d& direction, const Eigen::Vector3d& normal)
{
    double EFORT = -direction.transpose() * sample->jacobianProduct_.block<3,3>(0,0) * (-direction);
    return EFORT * (sample->manipulability_ + (direction.dot(normal)));
}
}

HeuristicFactory::HeuristicFactory()
{
    heuristics_.insert(std::make_pair("EFORT", &EFORTHeuristic));
    heuristics_.insert(std::make_pair("manipulability", &ManipulabilityHeuristic));
}

HeuristicFactory::~HeuristicFactory(){}

bool HeuristicFactory::AddHeuristic(const std::string& name, const heuristic func)
{
    if(heuristics_.find(name) != heuristics_.end())
        return false;
    heuristics_.insert(std::make_pair(name,func));
    return true;
}
