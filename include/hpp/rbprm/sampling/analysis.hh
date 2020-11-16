//
// Copyright (c) 2014 CNRS
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
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_ANALYSIS_HH
#define HPP_ANALYSIS_HH

#include <hpp/rbprm/sampling/sample.hh>
#include <hpp/rbprm/sampling/sample-db.hh>
#include <hpp/rbprm/rbprm-fullbody.hh>

#include <map>

namespace hpp {

namespace rbprm {
namespace sampling {

struct HPP_RBPRM_DLLAPI AnalysisFactory {
  AnalysisFactory(hpp::rbprm::RbPrmFullBodyPtr_t device);
  ~AnalysisFactory();

  bool AddAnalysis(const std::string& name, const evaluate func);
  T_evaluate evaluate_;
  rbprm::RbPrmFullBodyPtr_t device_;
};
}  // namespace sampling
}  // namespace rbprm
}  // namespace hpp
#endif  // HPP_ANALYSIS_HH
