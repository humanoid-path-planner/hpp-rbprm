//
// Copyright (c) 2017 CNRS
// Authors: Pierre Fernbach
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


#ifndef HPP_RBPRM_TIMED_PARABOLA_PATH_HH
#define HPP_RBPRM_TIMED_PARABOLA_PATH_HH


#include <hpp/rbprm/planner/parabola-path.hh>


namespace hpp {
  namespace rbprm {


    // forward declaration of class
    HPP_PREDEF_CLASS (TimedParabolaPath);
    // Planner objects are manipulated only via shared pointers
    typedef boost::shared_ptr <TimedParabolaPath> TimedParabolaPathPtr_t;


    /// ballistic path between 2 configurations
    ///
    /// call parabola-path but work with the time as parameter instead of x_theta
    class TimedParabolaPath : public ParabolaPath
    {
    public:
      typedef ParabolaPath parent_t;
      /// Destructor
      virtual ~TimedParabolaPath () throw () {}

      /// Create instance and return shared pointer
      /// \param device Robot corresponding to configurations
      /// \param init, end Start and end configurations of the path
      /// \param parabolaPath : the path used to compute the position at given time
      static TimedParabolaPathPtr_t create (const core::DevicePtr_t& device,
                                       core::ConfigurationIn_t init,
                                       core::ConfigurationIn_t end,
                                       ParabolaPathPtr_t parabolaPath)
      {
        TimedParabolaPath* ptr = new TimedParabolaPath (device, init, end, parabolaPath);
        TimedParabolaPathPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      /// Create instance and return shared pointer
      /// \param device Robot corresponding to configurations
      /// \param init, end Start and end configurations of the path
      /// \param length Distance between the configurations.
      static TimedParabolaPathPtr_t create (const core::DevicePtr_t& device,
                                       core::ConfigurationIn_t init,
                                       core::ConfigurationIn_t end,
                                       core::value_type length,
                                       core::vector_t coefficients)
      {
        TimedParabolaPath* ptr = new TimedParabolaPath (device, init, end, length,
                                              coefficients);
        TimedParabolaPathPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      /// Create instance and return shared pointer
      /// \param device Robot corresponding to configurations
      /// \param init, end Start and end configurations of the path
      /// \param length Distance between the configurations.
      /// \param V0, Vimp initial and final velocity vectors
      /// \param initialROMnames, endROMnames initial and final ROM names
      static TimedParabolaPathPtr_t create(const core::DevicePtr_t& device,
                                      core::ConfigurationIn_t init,
                                      core::ConfigurationIn_t end,
                                      core::value_type length,
                                      core::vector_t coefficients,
                                      core::vector_t V0, core::vector_t Vimp,
                                      std::vector <std::string> initialROMnames,
                                      std::vector <std::string> endROMnames)
      {
        TimedParabolaPath* ptr = new TimedParabolaPath (device, init, end, length,
                                              coefficients, V0, Vimp,
                                              initialROMnames, endROMnames);
        TimedParabolaPathPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      /// Create copy and return shared pointer
      /// \param path path to copy
      static TimedParabolaPathPtr_t createCopy (const TimedParabolaPathPtr_t& path)
      {
        TimedParabolaPath* ptr = new TimedParabolaPath (*path);
        TimedParabolaPathPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      /// Create copy and return shared pointer
      /// \param path path to copy
      /// \param constraints the path is subject to
      /// <!> constraints part NOT IMPLEMENTED YET
      static TimedParabolaPathPtr_t createCopy
      (const TimedParabolaPathPtr_t& path, const core::ConstraintSetPtr_t& /*constraints*/)
      {
        //TimedParabolaPath* ptr = new TimedParabolaPath (*path, constraints);
        TimedParabolaPath* ptr = new TimedParabolaPath (*path);
        TimedParabolaPathPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      /// Return a shared pointer to this
      ///
      /// As TimedParabolaPath are immutable, and refered to by shared pointers,
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
      virtual core::PathPtr_t extract (const core::interval_t& subInterval) const throw (core::projection_error);

      /// Reversion of a path
      /// \return a new path that is this one reversed.
      virtual core::PathPtr_t reverse () const;


      /// Modify initial configuration
      /// \param initial new initial configuration
      /// \pre input configuration should be of the same size as current initial
      /// configuration
      void initialConfig (core::ConfigurationIn_t initial)
      {
        assert (initial.size () == initial_.size ());
        initial_ = initial;
      }

      /// Modify end configuration
      /// \param end new end configuration
      /// \pre input configuration should be of the same size as current end
      /// configuration
      void endConfig (core::ConfigurationIn_t end)
      {
        assert (end.size () == end_.size ());
        end_ = end;
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

      /// Get previously computed length
      virtual core::value_type length () const {
        return length_;
      }

    protected :
      /// Constructor
      TimedParabolaPath (const core::DevicePtr_t& robot,
                    core::ConfigurationIn_t init,
                    core::ConfigurationIn_t end, ParabolaPathPtr_t parabolaPath);

      /// Constructor
      TimedParabolaPath (const core::DevicePtr_t& robot,
                    core::ConfigurationIn_t init,
                    core::ConfigurationIn_t end, core::value_type length,
                    core::vector_t coefficients);

      /// Constructor with velocities and ROMnames
      TimedParabolaPath (const core::DevicePtr_t& robot,
                    core::ConfigurationIn_t init,
                    core::ConfigurationIn_t end,
                    core::value_type length,
                    core::vector_t coefs,
                    core::vector_t V0, core::vector_t Vimp,
                    std::vector <std::string> initialROMnames,
                    std::vector <std::string> endROMnames);

      /// Copy constructor
      TimedParabolaPath (const TimedParabolaPath& path);


      void init (TimedParabolaPathPtr_t self)
      {
        parent_t::init (self);
        weak_ = self;
      }

      /// Param is the time
      virtual bool impl_compute (core::ConfigurationOut_t result,
                                 core::value_type t) const;

      virtual double computeTimedLength(double x_theta, double v0, double alpha0);
      virtual double computeTimedLength(ParabolaPathPtr_t parabolaPath);

      /// Print path in a stream
      virtual std::ostream& print (std::ostream &os) const
      {
        os << "TimedParabolaPath:" << std::endl;
        os << "interval: [ " << timeRange ().first << ", "
           << timeRange ().second << " ]" << std::endl;
        os << "initial configuration: " << initial_.transpose () << std::endl;
        os << "final configuration:   " << end_.transpose () << std::endl;
        return os;
      }

    private:
      core::DevicePtr_t device_;
      core::Configuration_t initial_;
      core::Configuration_t end_;
      TimedParabolaPathWkPtr_t weak_;
      ParabolaPathPtr_t parabolaPath_;
      mutable core::value_type length_;


    }; // class TimedParabolaPath
  } //   namespace rbprm
} // namespace hpp


#endif // HPP_RBPRM_TIMED_PARABOLA_PATH_HH
