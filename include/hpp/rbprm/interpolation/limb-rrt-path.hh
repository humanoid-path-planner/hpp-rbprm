//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-core
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

#ifndef HPP_RBPRM_LIMBRRT_PATH_HH
# define HPP_RBPRM_LIMBRRT_PATH_HH

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>
# include <hpp/core/path.hh>
# include <hpp/rbprm/interpolation/limb-rrt-path.hh>

namespace hpp {
namespace rbprm {
namespace interpolation {
    HPP_PREDEF_CLASS (LimbRRTPath);
    typedef boost::shared_ptr <LimbRRTPath>
    LimbRRTPathPtr_t;
    /// Linear interpolation between two configurations
    ///
    /// Degrees of freedom are interpolated depending on the type of
    /// \link hpp::model::Joint joint \endlink
    /// they parameterize:
    ///   \li linear interpolation for translation joints, bounded rotation
    ///       joints, and translation part of freeflyer joints,
    ///   \li angular interpolation for unbounded rotation joints,
    ///   \li constant angular velocity for SO(3) part of freeflyer joints.
    class HPP_CORE_DLLAPI LimbRRTPath : public core::Path
    {
    public:
      typedef Path parent_t;
      /// Destructor
      virtual ~LimbRRTPath () throw () {}

      /// Create instance and return shared pointer
      /// \param device Robot corresponding to configurations
      /// \param init, end Start and end configurations of the path
      /// \param length Distance between the configurations.
      static LimbRRTPathPtr_t create (const core::DevicePtr_t& device,
                                      core::ConfigurationIn_t init,
                                      core::ConfigurationIn_t end,
                                      core::value_type length,
                                      const std::size_t pathDofRank)
      {
    LimbRRTPath* ptr = new LimbRRTPath (device, init, end, length, pathDofRank);
    LimbRRTPathPtr_t shPtr (ptr);
    ptr->init (shPtr);
        ptr->checkPath ();
    return shPtr;
      }

      /// Create instance and return shared pointer
      /// \param device Robot corresponding to configurations
      /// \param init, end Start and end configurations of the path
      /// \param length Distance between the configurations.
      /// \param constraints the path is subject to
      static LimbRRTPathPtr_t create (const core::DevicePtr_t& device,
                                      core::ConfigurationIn_t init,
                                      core::ConfigurationIn_t end,
                                      core::value_type length,
                                      core::ConstraintSetPtr_t constraints,
                                      const std::size_t pathDofRank)
      {
    LimbRRTPath* ptr = new LimbRRTPath (device, init, end,
                          length, constraints, pathDofRank);
    LimbRRTPathPtr_t shPtr (ptr);
    ptr->init (shPtr);
        ptr->checkPath ();
    return shPtr;
      }

      /// Create copy and return shared pointer
      /// \param path path to copy
      static LimbRRTPathPtr_t createCopy (const LimbRRTPathPtr_t& path)
      {
    LimbRRTPath* ptr = new LimbRRTPath (*path);
    LimbRRTPathPtr_t shPtr (ptr);
    ptr->initCopy (shPtr);
        ptr->checkPath ();
    return shPtr;
      }

      /// Create copy and return shared pointer
      /// \param path path to copy
      /// \param constraints the path is subject to
      static LimbRRTPathPtr_t createCopy
    (const LimbRRTPathPtr_t& path, const core::ConstraintSetPtr_t& constraints)
      {
    LimbRRTPath* ptr = new LimbRRTPath (*path, constraints);
    LimbRRTPathPtr_t shPtr (ptr);
    ptr->initCopy (shPtr);
        ptr->checkPath ();
    return shPtr;
      }

      /// Return a shared pointer to this
      ///
      /// As LimbRRTPathP are immutable, and refered to by shared pointers,
      /// they do not need to be copied.
      virtual core::PathPtr_t copy () const
      {
    return createCopy (weak_.lock ());
      }

      /// Return a shared pointer to a copy of this and set constraints
      ///
      /// \param constraints constraints to apply to the copy
      /// \precond *this should not have constraints.
      virtual core::PathPtr_t copy (const core::ConstraintSetPtr_t& constraints) const
      {
    return createCopy (weak_.lock (), constraints);
      }


      /// Extraction/Reversion of a sub-path
      /// \param subInterval interval of definition of the extract path
      /// If upper bound of subInterval is smaller than lower bound,
      /// result is reversed.
      virtual core::PathPtr_t extract (const core::interval_t& subInterval) const
        throw (core::projection_error);

      /// Modify initial configuration
      /// \param initial new initial configuration
      /// \pre input configuration should be of the same size as current initial
      /// configuration
      void initialConfig (core::ConfigurationIn_t initial)
      {
    assert (initial.size () == initial_.size ());
    model::value_type dof = initial_[pathDofRank_];
    initial_ = initial;
    initial_[pathDofRank_] = dof;
      }

      /// Modify end configuration
      /// \param end new end configuration
      /// \pre input configuration should be of the same size as current end
      /// configuration
      void endConfig (core::ConfigurationIn_t end)
      {
    assert (end.size () == end_.size ());
    model::value_type dof = end_[pathDofRank_];
    end_ = end;
    end_[pathDofRank_] = dof;
      }

      /// Return the internal robot.
      core::DevicePtr_t device () const;

      /// Get the initial configuration
      core::Configuration_t initial () const
      {
        return initial_;
      }

      /// Get the final configuration
      core::Configuration_t end () const
      {
        return end_;
      }

    protected:
      /// Print path in a stream
      virtual std::ostream& print (std::ostream &os) const
      {
    os << "LimbRRTPath:" << std::endl;
    os << "interval: [ " << timeRange ().first << ", "
       << timeRange ().second << " ]" << std::endl;
    os << "initial configuration: " << initial_.transpose () << std::endl;
    os << "final configuration:   " << end_.transpose () << std::endl;
    return os;
      }
      /// Constructor
      LimbRRTPath (const core::DevicePtr_t& robot, core::ConfigurationIn_t init,
            core::ConfigurationIn_t end, core::value_type length,
                   const std::size_t pathDofRank);

      /// Constructor with constraints
      LimbRRTPath (const core::DevicePtr_t& robot, core::ConfigurationIn_t init,
            core::ConfigurationIn_t end, core::value_type length,
            core::ConstraintSetPtr_t constraints, const std::size_t pathDofRank);

      /// Copy constructor
      LimbRRTPath (const LimbRRTPath& path);

      /// Copy constructor with constraints
      LimbRRTPath (const LimbRRTPath& path,
            const core::ConstraintSetPtr_t& constraints);

      void init (LimbRRTPathPtr_t self)
      {
    parent_t::init (self);
    weak_ = self;
        checkPath ();
      }

      void initCopy (LimbRRTPathPtr_t self)
      {
    parent_t::initCopy (self);
    weak_ = self;
      }

      virtual bool impl_compute (core::ConfigurationOut_t result,
                 core::value_type param) const;

    private:
      core::DevicePtr_t device_;
      core::Configuration_t initial_;
      core::Configuration_t end_;

    public:
      const std::size_t pathDofRank_;

    private:
      LimbRRTPathWkPtr_t weak_;
    }; // class LimbRRTPath
} // namespace interpolation
} // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_LIMBRRT_PATH_HH
