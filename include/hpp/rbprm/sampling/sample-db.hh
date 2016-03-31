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
#include <hpp/rbprm/sampling/heuristic.hh>
#include <hpp/fcl/octree.h>
#include <vector>
#include <map>

namespace hpp {

  namespace rbprm {
  namespace sampling{

    /// Collision report for a Sample for which the octree node is
    /// colliding with the environment.
    struct OctreeReport
    {
      OctreeReport(const Sample*, const fcl::Contact, const double, const fcl::Vec3f& normal);
      /// Sample considered for contact generation
      const Sample* sample_;
      /// Contact information returned from fcl
      fcl::Contact contact_;
      /// heuristic evaluation of the sample
      double value_;
      /// normal vector of the surface in contact
      fcl::Vec3f normal_;
    };


    /// Comparaison operator between Samples
    /// Used to sort the contact candidates depending
    /// on their heuristic value
    struct sample_compare {
      bool operator() (const OctreeReport& lhs, const OctreeReport& rhs) const{
          return lhs.value_ > rhs.value_;
      }
    };

    typedef std::multiset<OctreeReport, sample_compare> T_OctreeReport;

    HPP_PREDEF_CLASS(SampleDB);

    typedef std::vector<double> T_Double;
    typedef std::map<std::string, T_Double> T_Values;
    typedef std::vector<sampling::Sample> T_Sample;


    /// Defines an evaluation function for a sample.
    /// \param SampleDB used database with already computed values
    /// \param sample sample candidate
    /// \param normal contact surface normal relatively to the candidate
    //typedef double (*evaluate) (const SampleDB& sampleDB, const sampling::Sample& sample);
    typedef boost::function <double (const SampleDB& sampleDB, const sampling::Sample& sample) > evaluate;
    typedef std::map<std::string, evaluate> T_evaluate;
    //first sample index, number of samples
    typedef std::pair<std::size_t, std::size_t> VoxelSampleId;
    typedef std::map<long int, VoxelSampleId> T_VoxelSampleId;

    /// Sample configuration for a robot limb, stored
    /// in an octree and used for proximity requests for contact creation.
    /// assumes that joints are compact, ie they all are consecutive in configuration.
    class HPP_RBPRM_DLLAPI SampleDB
    {
    public:
         SampleDB(std::ifstream& databaseStream, bool loadValues = true);
         SampleDB(const model::JointPtr_t limb, const std::string& effector, const std::size_t nbSamples,
                  const fcl::Vec3f& offset= fcl::Vec3f(0,0,0), const double resolution = 0.1, const T_evaluate& data = T_evaluate(), const std::string& staticValue ="");
        ~SampleDB();

    private:
         SampleDB(const SampleDB&);
         const SampleDB& operator=(const SampleDB&);

    public:
        double resolution_;
        T_Sample samples_;
        boost::shared_ptr<const octomap::OcTree> octomapTree_;
        fcl::OcTree* octree_; // deleted with geometry_
        boost::shared_ptr<fcl::CollisionGeometry> geometry_;
        T_Values values_;
        T_VoxelSampleId samplesInVoxels_;
        /// fcl collision object used for collisions with environment
        fcl::CollisionObject treeObject_;
        /// Bounding boxes of areas of interest of the octree
        std::map<std::size_t, fcl::CollisionObject*> boxes_;


    }; // class SampleDB

    HPP_RBPRM_DLLAPI SampleDB& addValue(SampleDB& database, const std::string& valueName, const evaluate eval, bool isStaticValue=true, bool sortSamples=true);
    HPP_RBPRM_DLLAPI bool saveLimbDatabase(const SampleDB& database, std::ofstream& dbFile);

    /// Given the current position of a robot, returns a set
    /// of candidate sample configurations for contact generation.
    /// The set is strictly ordered using a heuristic to determine
    /// the most relevant contacts.
    ///
    /// \param sc the SampleDB containing all the samples for a given limb
    /// \param treeTrf the current transformation of the root of the robot
    /// \param direction the current direction of motion, used to evaluate the sample
    /// heuristically
    /// \param evaluate heuristic used to sort candidates
    /// \return a set of OctreeReport with all the possible candidates for contact
    HPP_RBPRM_DLLAPI T_OctreeReport GetCandidates(const SampleDB& sc, const fcl::Transform3f& treeTrf,
                                            const hpp::model::CollisionObjectPtr_t& o2,
                                            const fcl::Vec3f& direction, const heuristic evaluate = 0);

    /// Given the current position of a robot, returns a set
    /// of candidate sample configurations for contact generation.
    /// The set is strictly ordered using a heuristic to determine
    /// the most relevant contacts.
    ///
    /// \param sc the SampleDB containing all the samples for a given limb
    /// \param treeTrf the current transformation of the root of the robot
    /// \param direction the current direction of motion, used to evaluate the sample
    /// heuristically
    /// \param a set of OctreeReport updated as the samples are explored
    /// \param evaluate heuristic used to sort candidates
    /// \return true if at least one candidate was found
    HPP_RBPRM_DLLAPI bool GetCandidates(const SampleDB& sc, const fcl::Transform3f& treeTrf,
                                            const hpp::model::CollisionObjectPtr_t& o2,
                                            const fcl::Vec3f& direction, T_OctreeReport& report, const heuristic evaluate = 0);

  } // namespace sampling
} // namespace rbprm
} // namespace hpp
#endif // HPP_RBPRM_SAMPLEDB_HH
