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

#ifndef HPP_RBPRM_PATH_INTERPOLATION_HH
# define HPP_RBPRM_PATH_INTERPOLATION_HH

# include <hpp/rbprm/config.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>
# include <hpp/core/path-vector.hh>
# include <hpp/model/device.hh>

# include <vector>

namespace hpp {
  namespace rbprm {
    HPP_PREDEF_CLASS(RbPrmInterpolation);

    /// Representation of a robot limb.
    /// Contains a SampleContainer used for computing contact candidates
    ///
    class RbPrmInterpolation;
    typedef boost::shared_ptr <RbPrmInterpolation> RbPrmInterpolationPtr_t;

    class HPP_RBPRM_DLLAPI RbPrmInterpolation
    {
    public:
        static RbPrmInterpolationPtr_t create (const core::PathVectorConstPtr_t path, const RbPrmFullBodyPtr_t robot, const State& start, const State& end);

    public:
        ~RbPrmInterpolation();

        std::vector<State> Interpolate(const model::ObjectVector_t &collisionObjects, const double timeStep = 0.01);
        std::vector<State> Interpolate(const std::vector<core::ConfigurationIn_t>& configurations, const model::ObjectVector_t &collisionObjects);

    public:
        const core::PathVectorConstPtr_t path_;
        const State start_;
        const State end_;

    private:
        RbPrmFullBodyPtr_t robot_;

    protected:
      RbPrmInterpolation (const core::PathVectorConstPtr_t path, const RbPrmFullBodyPtr_t robot,const State& start, const State& end);

      ///
      /// \brief Initialization.
      ///
      void init (const RbPrmInterpolationWkPtr_t& weakPtr);

    private:
      RbPrmInterpolationWkPtr_t weakPtr_;
    }; // class RbPrmLimb
  } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_PATH_INTERPOLATION_HH
