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


#ifndef HPP_RBPRM_BEZIER_PATH_HH
#define HPP_RBPRM_BEZIER_PATH_HH

# include <spline/bezier_curve.h>
# include <hpp/core/path.hh>
# include <vector>
# include <map>

namespace hpp {
namespace rbprm {

typedef spline::bezier_curve  <double, double, 3, true, Eigen::Vector3d > bezier_t;
typedef boost::shared_ptr<bezier_t> bezier_Ptr;
HPP_PREDEF_CLASS (BezierPath);
typedef boost::shared_ptr <BezierPath> BezierPathPtr_t;
typedef boost::shared_ptr <const BezierPath> BezierPathConstPtr_t;

/// \addtogroup path
/// \{
/// Bezier curve representation between two configurations
///
/// This class only implement the bezier curve of dimension 3, need to template the dimension of the points
/// Use the Bezier curve for the translation of the root and standard linear interpolation for the other DoF

class BezierPath : public core::Path
{

public:
    typedef Path parent_t;
    /// Destructor
    virtual ~BezierPath () throw () {}

    /// Create instance and return shared pointer
    /// \param device Robot corresponding to configurations
    /// \param curve the curve that define this path
    /// \param timeRange : the time definition of the curve.
    static BezierPathPtr_t create (const core::DevicePtr_t& device,
                                   const bezier_Ptr& curve,
                                   core::ConfigurationIn_t init,
                                   core::ConfigurationIn_t end,
                                   core::interval_t timeRange)
    {
        BezierPath* ptr = new BezierPath (device, curve,init,end, timeRange);
        BezierPathPtr_t shPtr (ptr);
        ptr->init (shPtr);
        ptr->checkPath ();
        return shPtr;
    }

    /// Create instance and return shared pointer
    /// \param device Robot corresponding to configurations
    /// \param wpBegin iterator to the first waypoint
    /// \param wpEnd iterator to the last wp
    /// \param length Distance between the configurations.
    static BezierPathPtr_t create (const core::DevicePtr_t& device,std::vector<bezier_t::point_t>::const_iterator wpBegin,std::vector<bezier_t::point_t>::const_iterator wpEnd,
                                   core::ConfigurationIn_t init,
                                   core::ConfigurationIn_t end,core::interval_t timeRange)
    {
        BezierPath* ptr = new BezierPath (device, wpBegin,wpEnd,init,end, timeRange);
        BezierPathPtr_t shPtr (ptr);
        ptr->init (shPtr);
        ptr->checkPath ();
        return shPtr;
    }

    /// Create copy and return shared pointer
    /// \param path path to copy
    static BezierPathPtr_t createCopy (const BezierPathPtr_t& path)
    {
        BezierPath* ptr = new BezierPath (*path);
        BezierPathPtr_t shPtr (ptr);
        ptr->initCopy (shPtr);
        return shPtr;
    }


    /// Create copy and return shared pointer
    /// \param path path to copy
    /// \param constraints the path is subject to
    static BezierPathPtr_t createCopy
    (const BezierPathPtr_t& path, const core::ConstraintSetPtr_t& constraints)
    {
        BezierPath* ptr = new BezierPath (*path, constraints);
        BezierPathPtr_t shPtr (ptr);
        ptr->initCopy (shPtr);
        ptr->checkPath ();
        return shPtr;
    }


    /// Return a shared pointer to this
    ///
    /// As BezierPath are immutable, and refered to by shared pointers,
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




    /// Get the initial configuration
    virtual core::Configuration_t initial () const{
        return (*curve_)(timeRange().first);
    }

    /// Get the final configuration
    virtual core::Configuration_t end () const {
        return (*curve_)(timeRange().second);
    }


    core::Configuration_t operator () (const core::value_type& t) const
    {
        core::Configuration_t result (outputSize ());
        impl_compute (result, t);
        if (constraints()) {
            constraints()->apply (result);
        }
        return result;
    }


protected :
    /// Print path in a stream
    virtual std::ostream& print (std::ostream &os) const
    {
        os << "BezierPath:" << std::endl;
        os << "interval: [ " << timeRange ().first << ", "
           << timeRange ().second << " ]" << std::endl;
        os << "initial configuration: " << initial().transpose() << std::endl;
        os << "final configuration:   " << end().transpose () << std::endl;
        os<<"Curve of degree :"<<curve_->degree_<<std::endl;
        os<<"waypoints = "<<std::endl;
        for(bezier_t::cit_point_t wpit = curve_->waypoints().begin() ; wpit != curve_->waypoints().end() ; ++wpit){
            os<<(*wpit).transpose()<<std::endl;
        }
        return os;
    }

    ///constructor with curve
    BezierPath (const core::DevicePtr_t& robot, const bezier_Ptr& curve,
                core::ConfigurationIn_t init,
                core::ConfigurationIn_t end,core::interval_t timeRange);

    ///constructor with waypoints
    BezierPath (const core::DevicePtr_t& robot, std::vector<bezier_t::point_t>::const_iterator wpBegin, std::vector<bezier_t::point_t>::const_iterator wpEnd,
                core::ConfigurationIn_t init,
                core::ConfigurationIn_t end, core::interval_t timeRange);


    /// Copy constructor
    BezierPath (const BezierPath& path);


    /// Copy constructor with constraints
    BezierPath (const BezierPath& path,
                const core::ConstraintSetPtr_t& constraints);


    void init (BezierPathPtr_t self)
    {
        parent_t::init (self);
        weak_ = self;
        checkPath ();
    }

    void initCopy (BezierPathPtr_t self)
    {
        parent_t::initCopy (self);
        weak_ = self;
    }

    virtual bool impl_compute (core::ConfigurationOut_t result,
                               core::value_type param) const;

    /*
    /// Virtual implementation of derivative
    virtual void impl_derivative (core::vectorOut_t result, const core::value_type& t,
                                  core::size_type order) const;
     */


private:
    model::DevicePtr_t device_;
    bezier_Ptr curve_;
    core::Configuration_t initial_;
    core::Configuration_t end_;
    BezierPathWkPtr_t weak_;

};//class Bezier Path
}//rbprm
}//hpp

#endif // HPP_RBPRM_BEZIER_PATH_HH
