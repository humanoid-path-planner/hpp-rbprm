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
    HPP_PREDEF_CLASS (RbPrmShooter);
    typedef boost::shared_ptr <RbPrmShooter>
    RbPrmShooterPtr_t;

/// \addtogroup configuration_sampling
/// \{
/// Samples configuration which respect the reachability condition
    class HPP_RBPRM_DLLAPI RbPrmShooter : public core::ConfigurationShooter{
    ///
    public:
        static HPP_RBPRM_DLLAPI RbPrmShooterPtr_t create (const model::RbPrmDevicePtr_t& robot,
                                         const core::ObjectVector_t &geometries,
                                         const std::vector<std::string>& filter = std::vector<std::string>(),
                                         const std::map<std::string, rbprm::NormalFilter>& normalFilters = std::map<std::string, rbprm::NormalFilter>(),
                                         const std::size_t shootLimit = 10000,
                                         const std::size_t displacementLimit = 100);
    virtual core::ConfigurationPtr_t shoot () const;

    public:
        typedef std::pair<fcl::Vec3f, TrianglePoints> T_TriangleNormal;

    public:
        const std::size_t shootLimit_;
        const std::size_t displacementLimit_;
        const std::vector<std::string> filter_;


    protected:
    /// Note that translation joints have to be bounded.
    RbPrmShooter (const model::RbPrmDevicePtr_t& robot,
                  const core::ObjectVector_t &geometries,
                  const std::vector<std::string>& filter,
                  const std::map<std::string, rbprm::NormalFilter>& normalFilters,
                  const std::size_t shootLimit = 10000,
                  const std::size_t displacementLimit = 100);

    void init (const RbPrmShooterPtr_t& self);

    private:
        void InitWeightedTriangles(const model::ObjectVector_t &geometries);
        const T_TriangleNormal& RandomPointIntriangle() const;
        const T_TriangleNormal& WeightedTriangle() const;

    private:
        std::vector<double> weights_;
        std::vector<T_TriangleNormal> triangles_;
        const model::RbPrmDevicePtr_t robot_;
        rbprm::RbPrmValidationPtr_t validator_;
        RbPrmShooterWkPtr_t weak_;
    }; // class RbprmShooter
/// \}
    } // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_SHOOTER_HH

