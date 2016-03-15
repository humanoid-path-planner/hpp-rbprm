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

#ifndef HPP_HEURISTIC_HH
# define HPP_HEURISTIC_HH

#include <hpp/rbprm/sampling/sample.hh>

#include <map>


namespace hpp {

  namespace rbprm {
  namespace sampling{


  /// Defines a heuristic method to sort samples
  /// the higher the score, the better the sample
  /// in presented joint
  /// \param sample sample candidate
  /// \param direction overall direction of motion
  /// \param normal contact surface normal relatively to the candidate
  typedef double (*heuristic) (const sampling::Sample& sample,
                               const Eigen::Vector3d& direction, const Eigen::Vector3d& normal);

  /// Defines a set of existing heuristics for biasing the sample candidate selection
  ///
  /// This class defines two heuristics by default. "EFORT" and "manipulability".
  struct HPP_RBPRM_DLLAPI HeuristicFactory
  {
       HeuristicFactory();
      ~HeuristicFactory();

       bool AddHeuristic(const std::string& name, const heuristic func);
       std::map<std::string, const heuristic> heuristics_;
  };

  } // namespace sampling
} // namespace rbprm
} // namespace hpp
#endif // HPP_HEURISTIC_HH
