//
// Copyright (c) 2017 CNRS
// Authors: Pierre Fernbach
//
// This file is part of hpp-rbprm
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_RBPRM_RANDOM_SHORTCUT_DYNAMIC_HH
#define HPP_RBPRM_RANDOM_SHORTCUT_DYNAMIC_HH

#include <hpp/core/path-optimization/random-shortcut.hh>
#include <hpp/rbprm/planner/rbprm-steering-kinodynamic.hh>
#include <hpp/rbprm/rbprm-path-validation.hh>

namespace hpp {
namespace rbprm {
/// \addtogroup path_optimization
/// \{

/// Random shortcut dynamic
///
/// Path optimizer that iteratively samples random configurations along a
/// path and that tries to connect these configurations by a call to
/// the steering method.
///
/// Extend the original algorithm from hpp-core in order to take compute
/// the dynamic constraints (friction cones) and verify the dynamic feasability
///
/// \note The optimizer assumes that the input path is a vector of optimal
///       paths for the distance function.

// forward declaration
HPP_PREDEF_CLASS(RandomShortcutDynamic);
// Planner objects are manipulated only via shared pointers
typedef std::shared_ptr<RandomShortcutDynamic> RandomShortcutDynamicPtr_t;

class RandomShortcutDynamic : public core::pathOptimization::RandomShortcut {
 public:
  /// Return shared pointer to new object.
  static RandomShortcutDynamicPtr_t create(core::ProblemConstPtr_t problem);

  /// Optimize path
  virtual core::PathVectorPtr_t optimize(const core::PathVectorPtr_t& path);

 protected:
  RandomShortcutDynamic(core::ProblemConstPtr_t problem);

  core::PathPtr_t steer(core::ConfigurationIn_t q1,
                        core::ConfigurationIn_t q2) const;

 private:
  const SteeringMethodKinodynamicPtr_t sm_;
  const RbPrmPathValidationPtr_t rbprmPathValidation_;
  double sizeFootX_, sizeFootY_;
  bool rectangularContact_;
  bool tryJump_;
  double mu_;
};  // class RandomShortcut
/// \}
}  // namespace rbprm
}  // namespace hpp

#endif  // HPP_RBPRM_RANDOM_SHORTCUT_DYNAMIC_HH
