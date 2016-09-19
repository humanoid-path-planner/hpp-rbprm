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

#include <hpp/rbprm/interpolation/com-rrt.hh>
#include <hpp/rbprm/interpolation/limb-rrt.hh>
#include <hpp/rbprm/interpolation/spline/effector-rrt.hh>
#include <hpp/rbprm/interpolation/com-rrt.hh>
#include <hpp/rbprm/tools.hh>
#include <hpp/model/joint.hh>
#include <hpp/core/bi-rrt-planner.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/discretized-path-validation.hh>
#include <hpp/constraints/position.hh>


namespace hpp {
using namespace core;
  namespace rbprm {
  namespace interpolation {

    typedef std::pair<value_type, vector_t> Waypoint;
    typedef std::vector<Waypoint, Eigen::aligned_allocator<Waypoint > > T_Waypoint;
    typedef T_Waypoint::iterator  IT_Waypoint;
    typedef T_Waypoint::const_iterator CIT_Waypoint;

    const value_type epsilon = std::numeric_limits<value_type>::epsilon();

    bool IsLine(const T_Waypoint& wayPoints)
    {
        const vector_t& init  = wayPoints.front().second;
        // compute line between first and last
        vector_t dir =  wayPoints.back().second- init;
        vector_t dir2;
        dir.normalize();
        for(CIT_Waypoint cit = wayPoints.begin() + 1; cit != wayPoints.end()-1; ++cit)
        {
            dir2  = cit->second - init;
            if(dir.norm() > epsilon)
                dir2.normalize();
            else
                throw("todo waypoint distance nul in end efefctor interpolation");
            if(dir.dot(dir2) < 1 - epsilon *4)
                return false;
        }
        return true;
    }

    bool Is2d(const T_Waypoint& wayPoints)
    {
        const std::size_t dim = wayPoints.front().second.rows() -1;
        const core::value_type ref = wayPoints.front().second[dim];
        value_type z_var = 0.;
        for(CIT_Waypoint cit = wayPoints.begin()+1 ; cit != wayPoints.end(); ++cit)
        {
            z_var = std::max(z_var, fabs(cit->second[dim] - ref));
        }
        return z_var < 0.02;
    }

    vector_t GetEffectorPositionAt(core::PathPtr_t path, constraints::PositionPtr_t position, const value_type time)
    {
        vector_t result (position->outputSize());
        position->operator ()(result, path->operator ()(time));
        return result;
    }

    T_Waypoint getWayPoints(model::DevicePtr_t device, core::PathPtr_t path,
                         const JointPtr_t effector, const value_type effectorDistance, bool& isLine)
    {
        //create evaluation function
        constraints::PositionPtr_t position = createPositionMethod(device,fcl::Vec3f(), effector);
        // define arbitrary number of way points depending on length
        std::size_t nbWayPoints = std::size_t(std::max(effectorDistance * 10, 3.));
        std::size_t dim = position->outputSize();
        if(!(nbWayPoints % 2)) nbWayPoints+=1;
        value_type pathIncrement = path->length() / nbWayPoints;
        T_Waypoint res;
        value_type t = path->timeRange().first;
        res.push_back(std::make_pair(0,GetEffectorPositionAt(path,position,t)));
        for(std::size_t i = 1; i <nbWayPoints-1; ++i)
        {
            t += pathIncrement;
            res.push_back(std::make_pair(i,GetEffectorPositionAt(path,position, t)));
        }
        res.push_back(std::make_pair(nbWayPoints-1,GetEffectorPositionAt(path,position,path->timeRange().second)));
        if(IsLine(res))
        {
            value_type height = effectorDistance < 0.1 ? 0.01 : std::max(nbWayPoints* 0.015, 0.02) ;
            isLine = true;
            T_Waypoint res2;
            res2.push_back(res.front());
            res2.push_back(std::make_pair(1, vector_t::Ones(position->outputSize()) * height +  res.front().second +  (res.back().second - res.front().second) / 2. ));
            res2.push_back(std::make_pair(2, res.back().second));
            return res2;
        }
        else if(Is2d(res))
        {
            isLine = true;
            std::size_t apex = nbWayPoints / 2;
            value_type max_height = effectorDistance < 0.1 ? 0.01 : std::max(nbWayPoints* 0.015, 0.02);
            value_type inc = max_height / apex;
            std::size_t current = 0;
            std::size_t inc_fac = 0;
            for(IT_Waypoint it = res.begin(); it != res.end()-1; ++it, ++current)
            {
                if(current>apex)
                {
                    it->second[dim-1] += inc_fac* (inc);
                    --inc_fac;
                }
                else
                {
                    it->second[dim-1] += inc_fac* inc;
                    if(current==apex)
                        --inc_fac;
                    else
                        ++inc_fac;
                }
            }
        }
        return res;
    }

    JointPtr_t getEffector(RbPrmFullBodyPtr_t fullbody,
                           const  State &startState, const State &nextState)
    {
        std::string effectorVar = nextState.contactCreations(startState).front();
        return fullbody->device_->getJointByName(fullbody->GetLimbs().at(effectorVar)->effector_->name());
    }

    exact_cubic_Ptr splineFromEffectorTraj(RbPrmFullBodyPtr_t fullbody, JointPtr_t effector, core::PathPtr_t path,
                                          const  State &startState, const State &nextState, bool& isLine)
    {
        // estimate length of distance travelled
        value_type length = effectorDistance(startState, nextState);
        T_Waypoint wayPoints = getWayPoints(fullbody->device_, path, effector, length, isLine);
        exact_cubic_Ptr ptr = exact_cubic_Ptr(new exact_cubic_t(wayPoints.begin(), wayPoints.end()));
        return ptr;
    }

    core::PathPtr_t effectorRRT(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem, const PathPtr_t comPath,
                           const  State &startState, const State &nextState,
                           const  std::size_t numOptimizations,
                           const bool keepExtraDof)
    {
        core::PathPtr_t fullBodyComPath = comRRT(fullbody, referenceProblem, comPath, startState, nextState, numOptimizations, true);
        if(effectorDistance(startState, nextState) < std::numeric_limits<value_type>::epsilon())
            return fullBodyComPath;
        EffectorRRTShooterFactory shooterFactory(fullBodyComPath);
        JointPtr_t effector =  getEffector(fullbody, startState, nextState);        
        bool isLine(false);
        exact_cubic_Ptr refEffector = splineFromEffectorTraj(fullbody, effector, fullBodyComPath, startState, nextState, isLine);
        if(!isLine)
        {
            return fullBodyComPath;
        }

        //removing extra dof
        core::SizeInterval_t interval(0, fullBodyComPath->initial().rows()-1);
        core::SizeIntervals_t intervals;
        intervals.push_back(interval);
        fullBodyComPath = core::SubchainPath::create(fullBodyComPath,intervals);

/*std::cout << "spline " << (*refEffector).min() << (*refEffector).max() << std::endl;
std::cout <<"[";
for(double i = 0; i< refEffector->max(); i += refEffector->max() / 10.)
{
    Eigen::Matrix<value_type, 3, 1> pt = (*refEffector)(i);
    std::cout << "[" << pt[0] << ","<< pt[1] << "," << pt[2] <<"], " << std::endl;
}
std::cout <<"]" << std::endl;*/

        SetEffectorRRTConstraints constraintFactory(comPath, refEffector, effector);
        T_StateFrame stateFrames;
        stateFrames.push_back(std::make_pair(comPath->timeRange().first, startState));
        stateFrames.push_back(std::make_pair(comPath->timeRange().second, nextState));
        return interpolateStatesFromPath<EffectorRRTHelper, EffectorRRTShooterFactory, SetEffectorRRTConstraints>
                (fullbody, referenceProblem, shooterFactory, constraintFactory, comPath,
                 stateFrames.begin(), stateFrames.begin()+1, numOptimizations % 10, keepExtraDof, 0.01);
    }
  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp
