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

#ifndef HPP_RBPRM_SHOOTER_HH
# define HPP_RBPRM_SHOOTER_HH

# include <hpp/rbprm/config.hh>
# include <hpp/rbprm/rbprm-device.hh>
# include <hpp/rbprm/rbprm-validation.hh>
# include <hpp/model/joint.hh>
# include <hpp/model/joint-configuration.hh>
# include <hpp/core/configuration-shooter.hh>

# include <vector>

namespace hpp {
    namespace rbprm {

    struct TrianglePoints
    {
        fcl::Vec3f p1, p2, p3;
    };

/// \addtogroup configuration_sampling
/// \{
    typedef std::vector<fcl::CollisionObjectConstPtr_t> T_CollisionObject;
/// Samples configuration which respect the reachability condition
    class HPP_RBPRM_DLLAPI RbPrmShooter : public core::ConfigurationShooter{
    ///
    public:
    /// Note that translation joints have to be bounded.
    RbPrmShooter (const model::RbPrmDevicePtr_t& robot,
                  const T_CollisionObject &geometries,
                  rbprm::RbPrmValidationPtr_t& validator,
                  const std::size_t shootLimit = 10000,
                  const std::size_t displacementLimit = 100);

    virtual core::ConfigurationPtr_t shoot () const;

    public:
        typedef std::pair<fcl::Vec3f, TrianglePoints> T_TriangleNormal;

    public:
        const std::size_t shootLimit_;
        const std::size_t displacementLimit_;

    private:
        void InitWeightedTriangles(const T_CollisionObject &geometries);
        const T_TriangleNormal& RandomPointIntriangle() const;
        const T_TriangleNormal& WeightedTriangle() const;

    private:
        std::vector<double> weights_;
        std::vector<T_TriangleNormal> triangles_;
        const model::RbPrmDevicePtr_t& robot_;
        rbprm::RbPrmValidationPtr_t& validator_;
    }; // class RbprmShooter
/// \}
    } // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_SHOOTER_HH

