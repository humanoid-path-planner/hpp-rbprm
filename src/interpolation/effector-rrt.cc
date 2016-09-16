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
        std::cout << "it s a line " << std::endl;
        return true;
    }

    bool Is2d(const T_Waypoint& wayPoints)
    {
        const std::size_t dim = wayPoints.front().second.rows();
        value_type z_var = 0.;
        for(CIT_Waypoint cit = wayPoints.begin()+1 ; cit != wayPoints.end(); ++cit)
        {
            z_var += cit->second[dim-1] - (cit-1)->second[dim-1];
        }
        if(z_var < 0.02)
            std::cout << "it s flat " << std::endl;
        return z_var < 0.02;
    }

    vector_t GetEffectorPositionAt(core::PathPtr_t path, constraints::deprecated::PositionPtr_t position, const value_type time)
    {
        vector_t result (position->outputSize());
        position->operator ()(result, path->operator ()(time));
        return result;
    }

    T_Waypoint getWayPoints(model::DevicePtr_t device, core::PathPtr_t path,
                         const JointPtr_t effector, const value_type effectorDistance)
    {
        //create evaluation function
        constraints::deprecated::PositionPtr_t position = createPositionMethod(device,fcl::Vec3f(), effector);
        // define arbitrary number of way points depending on length
        std::size_t nbWayPoints = std::size_t(std::max(effectorDistance * 10, 3.));
        std::size_t dim = position->outputSize();
//nbWayPoints = 3;
        if(!(nbWayPoints % 2)) nbWayPoints+=1;
        value_type pathIncrement = path->length() / nbWayPoints;
        //value_type timeIncrement = 1 / nbWayPoints;
        T_Waypoint res;
        value_type t = path->timeRange().first;
        res.push_back(std::make_pair(0,GetEffectorPositionAt(path,position,t)));
        for(std::size_t i = 1; i <nbWayPoints-1; ++i)
        {
            t += pathIncrement;
            res.push_back(std::make_pair(i-1,GetEffectorPositionAt(path,position, t)));
        }
        res.push_back(std::make_pair(1,GetEffectorPositionAt(path,position,path->timeRange().second)));
        if(IsLine(res))
        {
            T_Waypoint res2;
            res2.push_back(res.front());
            res2.push_back(std::make_pair(1, -vector_t::Ones(position->outputSize()) * nbWayPoints* 0.005 +  res.front().second +  (res.back().second - res.front().second) / 2. ));
            res2.push_back(std::make_pair(2, res.back().second));
            return res2;
        }
        else if(Is2d(res))
        {
            std::cout << "ini" << std::endl;
            std::cout <<"[";
            for(CIT_Waypoint cit = res.begin() ; cit != res.end(); ++cit)
            {
                vector_t pt = cit->second;
                std::cout << "[" << pt <<"], " << std::endl;
            }
            std::cout <<"]" << std::endl;

            std::size_t apex = nbWayPoints / 2;
            value_type max_height = (nbWayPoints* 0.005);
            value_type default_height = res.front().second[dim-1];
            value_type inc = max_height / apex;
            std::size_t current = 0;
            std::size_t inc_fac = 0;
            std::cout << "adding to z in line, num wp, max height " << nbWayPoints << "; "<< max_height << std::endl;
            for(IT_Waypoint it = res.begin(); it != res.end()-1; ++it, ++current)
            {
                if(current>apex)
                {
                    std::cout << "adding to z in line " << inc_fac* (inc) << std::endl;
                    it->second[dim-1] = default_height + inc_fac* (inc);
                    --inc_fac;
                }
                else
                {
                    std::cout << "adding to z in line " << inc_fac* (inc) << std::endl;
                    it->second[dim-1] = default_height + inc_fac* inc;
                    if(current==apex)
                        --inc_fac;
                    else
                        ++inc_fac;
                }
            }

            std::cout << "corrected" << std::endl;
            std::cout <<"[";
            for(CIT_Waypoint cit = res.begin() ; cit != res.end(); ++cit)
            {
                vector_t pt = cit->second;
                std::cout << "[" << pt  <<"], " << std::endl;
            }
            std::cout <<"]" << std::endl;
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
                                          const  State &startState, const State &nextState)
    {
        // estimate length of distance travelled
        value_type length = effectorDistance(startState, nextState);
        T_Waypoint wayPoints = getWayPoints(fullbody->device_, path, effector, length);

        std::cout << "config diff " << path->initial() - startState.configuration_ << std::endl;

//bezier curveis
std::vector<Eigen::Matrix<value_type, 1, 1> > pts;
//std::vector < std::pair<double, Eigen::Matrix<value_type, 1, 1> > > pts;
for(CIT_Waypoint cit = wayPoints.begin(); cit != wayPoints.end(); ++cit)
{
    std::cout << "point " << cit->second << std::endl;
    Eigen::Matrix<value_type, 1, 1> pt = (cit->second.tail<1>());
    std::cout << "point in vec " << pt << std::endl;
    //pts.push_back(std::make_pair(cit->first, pt));
    pts.push_back(pt);
}
    exact_cubic_Ptr ptr = exact_cubic_Ptr(new exact_cubic_t(pts.begin(), pts.end()));
    return ptr;
    }

    core::PathPtr_t effectorRRT(RbPrmFullBodyPtr_t fullbody, core::ProblemPtr_t referenceProblem, const PathPtr_t comPath,
                           const  State &startState, const State &nextState,
                           const  std::size_t numOptimizations,
                           const bool keepExtraDof)
    {
        core::PathPtr_t fullBodyComPath = comRRT(fullbody, referenceProblem, comPath, startState, nextState, numOptimizations, false);
        std::cout << "called comRRT " << std::endl;
        if(effectorDistance(startState, nextState) < std::numeric_limits<value_type>::epsilon())
            return fullBodyComPath;
        std::cout << "calling effector interpolation " << std::endl;
        EffectorRRTShooterFactory shooterFactory(fullBodyComPath);
        JointPtr_t effector =  getEffector(fullbody, startState, nextState);
        exact_cubic_Ptr refEffector = splineFromEffectorTraj(fullbody, effector, fullBodyComPath, startState, nextState);

        std::cout << "spline " << (*refEffector).min() << (*refEffector).max() << std::endl;
        std::cout <<"[";
        for(int i = 0; i< 11; ++i)
        {
            value_type t = i / 10.;
            //Eigen::Vector3d pt = refEffector->operator ()(t);
            Eigen::Matrix<value_type, 1, 1> pt = (*refEffector)(t);
            //std::cout << "[" << pt[0] << ","<< pt[1] << "," << pt[2] <<"], " << std::endl;
            std::cout << "[" << pt << "], " << std::endl;
        }
        std::cout <<"]" << std::endl;

        SetEffectorRRTConstraints constraintFactory(comPath, refEffector, effector);
        T_StateFrame stateFrames;
        stateFrames.push_back(std::make_pair(comPath->timeRange().first, startState));
        stateFrames.push_back(std::make_pair(comPath->timeRange().second, nextState));
        return interpolateStatesFromPath<EffectorRRTHelper, EffectorRRTShooterFactory, SetEffectorRRTConstraints>
                (fullbody, referenceProblem, shooterFactory, constraintFactory, comPath, stateFrames.begin(), stateFrames.begin()+1, numOptimizations, keepExtraDof);
    }
  }// namespace interpolation
  }// namespace rbprm
}// namespace hpp
