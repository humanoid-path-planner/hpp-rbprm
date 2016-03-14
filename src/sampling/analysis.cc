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

#include <hpp/rbprm/sampling/analysis.hh>
#include <time.h>

#include <Eigen/Eigen>

using namespace hpp;
using namespace hpp::model;
using namespace hpp::rbprm;
using namespace hpp::rbprm::sampling;

namespace
{
    double manipulability(const SampleDB& /*sampleDB*/, const sampling::Sample& sample)
    {
        double det = sample.jacobianProduct_.determinant();
        return det > 0 ? sqrt(det) : 0;
    }

    double isotropy(const SampleDB& /*sampleDB*/, const sampling::Sample& sample)
    {
        double det = sample.jacobianProduct_.determinant();
        return det > 0 ? sqrt(det) : 0;
    }
}

AnalysisFactory::AnalysisFactory()
{
    evaluate_.insert(std::make_pair("manipulability", &manipulability));/*
    evaluate_.insert(std::make_pair("EFORT", &EFORTHeuristic));
    evaluate_.insert(std::make_pair("EFORT_Normal", &EFORTNormalHeuristic));
    evaluate_.insert(std::make_pair("manipulability", &ManipulabilityHeuristic));
    evaluate_.insert(std::make_pair("random", &RandomHeuristic));
    evaluate_.insert(std::make_pair("forward", &ForwardHeuristic));
    evaluate_.insert(std::make_pair("backward", &BackwardHeuristic));*/
}

AnalysisFactory::~AnalysisFactory(){}

bool AnalysisFactory::AddAnalysis(const std::string& name, const evaluate func)
{
    if(evaluate_.find(name) != evaluate_.end())
        return false;
    evaluate_.insert(std::make_pair(name,func));
    return true;
}
