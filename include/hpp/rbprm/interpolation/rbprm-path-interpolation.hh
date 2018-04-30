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
    typedef std::vector<model::vector_t,Eigen::aligned_allocator<model::vector_t> > T_Configuration;
    typedef T_Configuration::const_iterator CIT_Configuration;
    namespace interpolation {
    HPP_PREDEF_CLASS(RbPrmInterpolation);

    /// Interpolation class for transforming a path computed by RB-PRM into
    /// a discrete sequence of balanced contact configurations.
    ///
    class RbPrmInterpolation;
    typedef boost::shared_ptr <RbPrmInterpolation> RbPrmInterpolationPtr_t;

    class HPP_RBPRM_DLLAPI RbPrmInterpolation
    {
    public:
        /// Creates a smart pointer to the Interpolation class
        ///
        /// \param path the path returned by RB-PRM computation
        /// \param robot the FullBody instance considered for extending the part
        /// \param start the start full body configuration of the problem
        /// \param end the end full body configuration of the problem
        /// \return a pointer to the created RbPrmInterpolation instance
        static RbPrmInterpolationPtr_t create (const RbPrmFullBodyPtr_t robot, const State& start, const State& end,
                                               const core::PathVectorConstPtr_t path = core::PathVectorConstPtr_t(), const bool testReachability = true, const bool quasiStatic = false);

    public:
        ~RbPrmInterpolation();

        /// Transforms the path computed by RB-PRM into
        /// a discrete sequence of balanced contact configurations.
        ///
        /// \param affordances the set of 3D objects to consider for contact creation.
        /// \param affFilters a vector of strings determining which affordance
        ///  types are to be used in generating contacts for each limb.
        /// \param timeStep the discretization step of the path.
        /// \param robustnessTreshold minimum value of the static equilibrium robustness criterion required to accept the configuration (0 by default).
        /// \return a pointer to the created RbPrmInterpolation instance
        rbprm::T_StateFrame Interpolate(const affMap_t& affordances, const std::map<std::string, std::vector<std::string> >& affFilters,
                                        const double timeStep = 0.01, const double robustnessTreshold=0.,
                                        const bool filterStates = false);

        /// Transforms a discrete sequence of configurations into
        /// a discrete sequence of balanced contact configurations.
        ///
        /// \param affordances the set of 3D objects to consider for contact creation.
        /// \param affFilters a vector of strings determining which affordance
        ///  types are to be used in generating contacts for each limb.
        /// \param configs
		/// \param timeStep the discretization step of the path.
		/// \param initValue initial time value associated to the first configuration        
        /// \param robustnessTreshold minimum value of the static equilibrium robustness criterion required to accept the configuration (0 by default).
       /// \return The time parametrized list of states according to the reference path
        rbprm::T_StateFrame Interpolate(const affMap_t& affordances, const std::map<std::string, std::vector<std::string> >& affFilters,
                                        const T_Configuration& configs, const double robustnessTreshold=0.,
                                        const model::value_type timeStep = 1., const model::value_type initValue = 0.,
                                        const bool filterStates = false);

        core::Configuration_t configPosition(core::ConfigurationIn_t previous, const core::PathVectorConstPtr_t path, double i);


        ///
        /// \brief addGoalConfig add goal configuration (end_ state) at the end of a states list. Modify the last state (or add intermediate states) in the list to assure that there is only one contact variation between each states.
        /// \param states
        /// \return the input list with the last states modified and the goal state added
        ///
        rbprm::T_StateFrame addGoalConfig(const rbprm::T_StateFrame& states);

    public:
        const core::PathVectorConstPtr_t path_;
        const State start_;
        const State end_;
        bool testReachability_; // decide if we use the reachability criterion during interpolation
        bool quasiStatic_; // decide if we use the criterion only in quasi-static


    private:
        RbPrmFullBodyPtr_t robot_;

        T_StateFrame FilterStates(const T_StateFrame& originStates, const bool deep);
        T_StateFrame FilterStatesRec( const T_StateFrame& originStates);
        T_StateFrame tryReplaceStates( const T_StateFrame& originStates);
        void tryReplaceStates(const CIT_StateFrame& from, const CIT_StateFrame to, T_StateFrame& res);
        T_StateFrame trySkipStates( const T_StateFrame& originStates);
        void trySkipStates(const CIT_StateFrame& from, const CIT_StateFrame to, T_StateFrame& res);
        void FilterRepositioning(const CIT_StateFrame& from, const CIT_StateFrame to, T_StateFrame& res);
        T_StateFrame FilterRepositioning(const T_StateFrame& originStates);
        void FilterBreakCreate(const CIT_StateFrame& from, const CIT_StateFrame to, T_StateFrame& res);
        T_StateFrame FilterBreakCreate(const T_StateFrame& originStates);
        StateFrame findBestRepositionState(T_StateFrame candidates,std::vector<std::string> limbsNames);
        bool testReachability(const State& s0, const State& s1);


    protected:
      RbPrmInterpolation (const core::PathVectorConstPtr_t path, const RbPrmFullBodyPtr_t robot,const State& start, const State& end, const bool testReachability = true, const bool quasiStatic = false);

      ///
      /// \brief Initialization.
      ///
      void init (const RbPrmInterpolationWkPtr_t& weakPtr);

    private:
      RbPrmInterpolationWkPtr_t weakPtr_;
    }; // class RbPrmLimb

    /// Remove redundant State in terms of contacts, defined as follows:
    /// - The same effector is repositioned two or more times in a row, while
    /// and the root trajectory is approximatively linear all along
    /// - An effector is broken, and the same effector is replaced in the following state
    /// \param originStates original state list
    /// \param deep if false, only matching configurations in sequence are removed
    /// \return A list of key states filtered

    } // namespace interpolation
  } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_PATH_INTERPOLATION_HH
