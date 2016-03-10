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

#ifndef HPP_RBPRM_SAMPLEDB_HH
# define HPP_RBPRM_SAMPLEDB_HH

#include <hpp/rbprm/config.h>
#include <hpp/rbprm/sampling/sample.hh>
#include <hpp/fcl/octree.h>
#include <vector>
#include <map>

namespace hpp {

  namespace rbprm {
  namespace sampling{

    HPP_PREDEF_CLASS(SampleDB);

    typedef std::vector<double> T_Double;
    typedef std::map<std::string, T_Double> T_Values;
    typedef std::vector<sampling::Sample> T_Sample;


    /// Defines an evaluation function for a sample.
    /// \param sample sample candidate
    /// \param direction overall direction of motion
    /// \param normal contact surface normal relatively to the candidate
    typedef double (*evaluate) (const sampling::Sample& sample);
    typedef std::map<std::string, evaluate> T_evaluate;
    //first sample index, number of samples
    typedef std::pair<std::size_t, std::size_t> VoxelSampleId;
    typedef std::map<long int, VoxelSampleId> T_VoxelSampleId;

    struct SampleDBPImpl;
    /// Sample configuration for a robot limb, stored
    /// in an octree and used for proximity requests for contact creation.
    /// assumes that joints are compact, ie they all are consecutive in configuration.
    class HPP_RBPRM_DLLAPI SampleDB : boost::noncopyable
    {
    public:
         SampleDB(const model::JointPtr_t limb, const std::string& effector, const std::size_t nbSamples,
                  const fcl::Vec3f& offset= fcl::Vec3f(0,0,0), const double resolution = 0.1, const T_evaluate& data = T_evaluate(), const std::string& staticValue ="");
        ~SampleDB(){};

    public:
        const double resolution_;
        T_Sample samples_;
        const boost::shared_ptr<const octomap::OcTree> octomapTree_;
        fcl::OcTree* octree_; // deleted with geometry_
        const boost::shared_ptr<fcl::CollisionGeometry> geometry_;
        T_Values values_;
        T_VoxelSampleId samplesInVoxels_;

    }; // class SampleDB

    HPP_RBPRM_DLLAPI SampleDB& addValue(SampleDB& database, const std::string& valueName, const evaluate eval, bool isStaticValue=true, bool sortSamples=true);
    HPP_RBPRM_DLLAPI SampleDB loadLimbDatabase(const std::string& limbId, const std::string& dbFileName);
    HPP_RBPRM_DLLAPI bool saveLimbDatabase(const SampleDB& database, const std::string& dbFileName);

  } // namespace sampling
} // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_SAMPLEDB_HH
