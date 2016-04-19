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
        /// Creates an instance of RbPrmShooter for reachability based planning.
        ///
        /// \param robot the Device describing the trunk approximation, that must remain collision free
        /// \param geometries The mesh describing the environment. WARNING:A current prerequisite of RB-PRM
        /// is that the mesh is composed of triangles. The normals of the triangles are computed using
        /// the blender convention.
        /// \param filter Specifies constraints on the range of motion of the limbs of the robot for a configuration to
        /// be valid. If filter is empty, no constraints are defined. Otherwise, a configuration can only be valid the all the
        /// ROMS specified in the vector acre colliding with the environment
        /// \param normalFilters Specifies constraints on the accepted normal orientation (in world coordinates) for the reachability condition to be valid.
        /// for instance If a normal filter is set to a normal of  z=(0,0,1) and a range of 0.9, only surfaces with a normal that
        /// has a dot product with z greater or equal than 0.9 will be considered for validating the ROM collision.
        /// Otherwise no constraints are considered and any colliding surface will validate the condition.
        /// \param shootLimit the maximum number of trials spent in trying to generate a valid configuration before failing.
        /// \param displacementLimit maximum number of local displacements allowed for a shot configuration to try to verify
        /// the reachability condition.
        /// \return a pointer to an instance of RbPrmShooter
        static HPP_RBPRM_DLLAPI RbPrmShooterPtr_t create (const model::RbPrmDevicePtr_t& robot,
                                         const core::ObjectVector_t &geometries,
																				 const std::map<std::string, std::vector<model::CollisionObjectPtr_t> > affordances,
                                         const std::vector<std::string>& filter = std::vector<std::string>(),
                                         const std::map<std::string, std::vector<std::string> >& affFilters = std::map<std::string, std::vector<std::string> >(),
                                         const std::size_t shootLimit = 10000,
                                         const std::size_t displacementLimit = 100);
    virtual core::ConfigurationPtr_t shoot () const;


    public:
        /// Sets limits on robot orientation, described according to Euler's ZYX rotation order
        ///
        /// \param limitszyx 6D vector with the lower and upperBound for each rotation axis in sequence
        /// expressed in gradients
        /// [z_inf, z_sup, y_inf, y_sup, x_inf, x_sup]
        void BoundSO3(const std::vector<double>& limitszyx);

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
									const std::map<std::string, std::vector<model::CollisionObjectPtr_t> > &affordances,
                  const std::vector<std::string>& filter,
                  const std::map<std::string, std::vector<std::string> >& affFilters,
                  const std::size_t shootLimit = 10000,
                  const std::size_t displacementLimit = 100);

    void init (const RbPrmShooterPtr_t& self);

    private:
        void InitWeightedTriangles
					(const std::map<std::string, std::vector<model::CollisionObjectPtr_t> > &affordances);
        const T_TriangleNormal& RandomPointIntriangle(const std::string &affordance) const;
        const T_TriangleNormal& WeightedTriangle(const std::string &affordance) const;

    private:
        std::map<std::string, std::vector<double> > affWeights_;
        std::map<std::string, std::vector<T_TriangleNormal> >affTris_;
        const model::RbPrmDevicePtr_t robot_;
        rbprm::RbPrmValidationPtr_t validator_;
        RbPrmShooterWkPtr_t weak_;
        model::DevicePtr_t eulerSo3_;
    }; // class RbprmShooter
/// \}
    } // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_SHOOTER_HH

