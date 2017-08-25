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

#include <hpp/rbprm/sampling/heuristic.hh>
#include <hpp/model/configuration.hh>
#include <time.h>

#include <Eigen/Eigen>

using namespace hpp;
using namespace hpp::model;
using namespace hpp::rbprm;
using namespace hpp::rbprm::sampling;

namespace
{

double dynamicHeuristic(const sampling::Sample & sample, const Eigen::Vector3d & /*direction*/, const Eigen::Vector3d & /*normal*/, const HeuristicParam & params)
{
    fcl::Vec3f effectorPosition = transform(sample.effectorPosition_, params.tfWorldRoot_.getTranslation(), params.tfWorldRoot_.getRotation());
    std::map <std::string, fcl::Vec3f> contacts;
    contacts.insert(params.contactPositions_.begin(), params.contactPositions_.end());
    contacts.insert(std::make_pair(params.sampleLimbName_, effectorPosition));
    removeNonGroundContacts(contacts, 0.25); // keep only ground contacts

    double g(-9.80665);
    double w2(params.comPosition_[2]/g); // w2 < 0
    double w1x(-100.*w2); // w1 > 0
    double w1y(-100.*w2); // w1 > 0

    // We want : |w1*comSpeed| > |w2*comAcceleration|
    if(params.comSpeed_[0] != 0 && w2*params.comAcceleration_[0] !=0)
    {
        while(std::abs(w1x*params.comSpeed_[0]) <= std::abs(w2*params.comAcceleration_[0]))
        {
            w1x *= 1.5;
        }
    }
    if(params.comSpeed_[1] != 0 && w2*params.comAcceleration_[0] !=0 )
    {
        while(std::abs(w1y*params.comSpeed_[1]) <= std::abs(w2*params.comAcceleration_[1]))
        {
            w1y *= 1.5;
        }
    }

    double x_interest(params.comPosition_[0] + w1x*params.comSpeed_[0] + w2*params.comAcceleration_[0]);
    double y_interest(params.comPosition_[1] + w1y*params.comSpeed_[1] + w2*params.comAcceleration_[1]);

    Vec2D interest(x_interest, y_interest);

    double result;
    try
    {
        Vec2D wcentroid(weightedCentroidConvex2D(convexHull(computeSupportPolygon(contacts))));
        result = Vec2D::euclideanDist(interest, wcentroid);
    }
    catch(const std::string & s)
    {
        std::cout << s << std::endl;
        result = std::numeric_limits<double>::max();
    }

    return -result; // '-' because minimize a value is equivalent to maximimze its opposite
}

double EFORTHeuristic(const sampling::Sample& sample,
                      const Eigen::Vector3d& direction, const Eigen::Vector3d& normal, const HeuristicParam & /*params*/)
{
    double EFORT = -direction.transpose() * sample.jacobianProduct_.block<3,3>(0,0) * (-direction);
    return EFORT * Eigen::Vector3d::UnitZ().dot(normal);
}

double EFORTNormalHeuristic(const sampling::Sample& sample,
                      const Eigen::Vector3d& direction, const Eigen::Vector3d& normal, const HeuristicParam & /*params*/)
{
    double EFORT = -direction.transpose() * sample.jacobianProduct_.block<3,3>(0,0) * (-direction);
    return EFORT * direction.dot(normal);
}

double ManipulabilityHeuristic(const sampling::Sample& sample,
                               const Eigen::Vector3d& /*direction*/, const Eigen::Vector3d& normal, const HeuristicParam & /*params*/)
{
    if(Eigen::Vector3d::UnitZ().dot(normal) < 0.7) return -1;
    return sample.staticValue_ * 10000 * Eigen::Vector3d::UnitZ().dot(normal) * 100000  +  ((double)rand()) / ((double)(RAND_MAX));
}

double RandomHeuristic(const sampling::Sample& /*sample*/,
                       const Eigen::Vector3d& /*direction*/, const Eigen::Vector3d& /*normal*/, const HeuristicParam & /*params*/)
{
    return ((double)rand()) / ((double)(RAND_MAX));
}


double ForwardHeuristic(const sampling::Sample& sample,
                      const Eigen::Vector3d& direction, const Eigen::Vector3d& normal, const HeuristicParam & /*params*/)
{
 /*   //hppDout(notice,"static value : "<<sample.staticValue_);
  hppDout(notice,"eff position = "<<sample.effectorPosition_);
  hppDout(notice,"limb frame   = "<<sample.effectorPositionInLimbFrame_);
  hppDout(notice,"direction    = "<<direction);*/


    return sample.staticValue_ * 1000.  * Eigen::Vector3d::UnitZ().dot(normal) + 1000. * sample.effectorPositionInLimbFrame_.dot(fcl::Vec3f(direction(0),direction(1),sample.effectorPositionInLimbFrame_[2])) + ((double)rand()) / ((double)(RAND_MAX));
}



double BackwardHeuristic(const sampling::Sample& sample,
                      const Eigen::Vector3d& direction, const Eigen::Vector3d& normal, const HeuristicParam & /*params*/)
{
    return sample.staticValue_ * 10000 * Eigen::Vector3d::UnitZ().dot(normal) - 100  * sample.effectorPosition_.dot(fcl::Vec3f(direction(0),direction(1),direction(2))) + ((double)rand()) / ((double)(RAND_MAX));
}

double StaticHeuristic(const sampling::Sample& sample,
                      const Eigen::Vector3d& /*direction*/, const Eigen::Vector3d& /*normal*/, const HeuristicParam & /*params*/)
{
    /*hppDout(info,"sample : ");
    hppDout(info,"sample : "<<&sample);
    hppDout(info,"id = "<<sample.id_);
    hppDout(info,"length = "<<sample.length_);
    hppDout(info,"startRank = "<<sample.startRank_);
    hppDout(info,"effectorPosition = "<<sample.effectorPosition_);
    hppDout(info,"configuration = "<<sample.configuration_);
    hppDout(info,"staticValue = "<<sample.staticValue_);
    */
    return sample.staticValue_;

}


double DistanceToLimitHeuristic(const sampling::Sample& sample,
                      const Eigen::Vector3d& /*direction*/, const Eigen::Vector3d& /*normal*/, const HeuristicParam & /*params*/)
{
    return sample.configuration_.norm();
}
}

HeuristicFactory::HeuristicFactory()
{
    unsigned int seed =  (unsigned int) (time(NULL)) ;
    //seed = 1485441926; // prepare_jump
    // seed = 1486147856; // stairs (18)
    //seed = 1486392757; // sideWall HyQ
    // seed = 1486721923; //hrp2 downSLope
   // seed = 1487082431; // hrp2 +0.15 z axis
    //seed = 1488532591; // downSLope (good contact but not interp) (yaml 1)
    // seed = 1488545915 ; //downSlope (need test)
    // seed = 1488550692 ; // downslope 2
    seed = 1491571994; // straight walk 2 m static
    //seed = 1491580336 ; // straight walk dynamic 0.5
    //seed = 1492176551; // walk 0.3 !!
    //seed = 1493208163 ; // stairs static contacts
    std::cout<<"seed = "<<seed<<std::endl;
    srand ( seed);
    hppDout(notice,"SEED for heuristic = "<<seed);
    heuristics_.insert(std::make_pair("static", &StaticHeuristic));
    heuristics_.insert(std::make_pair("EFORT", &EFORTHeuristic));
    heuristics_.insert(std::make_pair("EFORT_Normal", &EFORTNormalHeuristic));
    heuristics_.insert(std::make_pair("manipulability", &ManipulabilityHeuristic));
    heuristics_.insert(std::make_pair("random", &RandomHeuristic));
    heuristics_.insert(std::make_pair("forward", &ForwardHeuristic));
    heuristics_.insert(std::make_pair("backward", &BackwardHeuristic));
    heuristics_.insert(std::make_pair("jointlimits", &DistanceToLimitHeuristic));
    heuristics_.insert(std::make_pair("dynamic", &dynamicHeuristic));
}

HeuristicFactory::~HeuristicFactory(){}

bool HeuristicFactory::AddHeuristic(const std::string& name, const heuristic func)
{
    if(heuristics_.find(name) != heuristics_.end())
        return false;
    heuristics_.insert(std::make_pair(name,func));
    return true;
}
