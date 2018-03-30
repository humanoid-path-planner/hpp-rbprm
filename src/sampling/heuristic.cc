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
#include <hpp/rbprm/sampling/heuristic-tools.hh>
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
    return sample.staticValue_ * 1000.  * Eigen::Vector3d::UnitZ().dot(normal) + 100. * sample.effectorPositionInLimbFrame_.dot(fcl::Vec3f(direction(0),direction(1),sample.effectorPositionInLimbFrame_[2])) + ((double)rand()) / ((double)(RAND_MAX));
}

double DynamicWalkHeuristic(const sampling::Sample& sample,
                      const Eigen::Vector3d& direction, const Eigen::Vector3d& normal, const HeuristicParam & params)
{
  fcl::Vec3f dir(direction);
  fcl::Vec3f pos(sample.effectorPositionInLimbFrame_);
  fcl::Vec3f n(normal);
  n.normalize();
  double weightDir,weightStatic;
  weightStatic=100.;

  if(direction.norm() == 0 || std::isnan(direction.norm())){ // test for null vector, functions called before this one can try to normalize direction resulting in NaN
      weightDir=0;
      dir=fcl::Vec3f(0,0,1);
  }
  else{
      weightDir=100.;
      weightStatic=weightStatic*Eigen::Vector3d::UnitZ().dot(normal);
      dir[2]=0; // FIXME : replace this by a projection on the surface plan ( we know the normal)
      dir = dir.normalize();
      //hppDout(notice,"limb frame   vlb = ["<<sample.configuration_[0]<<","<<sample.configuration_[1]<<","<<sample.configuration_[2]<<","<<pos[0]<<","<<pos[1]<<","<<0<<" ]");
      pos = (params.tfWorldRoot_.getRotation()*(pos));
      pos[2] = 0; // FIXME : replace this by a projection on the surface plan ( we know the normal)
      //pos = pos.normalize();
      //hppDout(notice,"root transform : "<<params.tfWorldRoot_);
      fcl::Vec3f limbRoot = sample.effectorPosition_-sample.effectorPositionInLimbFrame_;
      //hppDout(notice,"limb origin : "<<limbRoot);

      // compute signed angle between dir and pos
      double angle = atan2((dir.cross(pos)).dot(n),dir.dot(pos));
      if(limbRoot[1]>0.){ // the current limb is on the left of the root
        //hppDout(notice,"left limb");
        // test if the pos vector is on the right of the dir vector
        if(angle<0){
          weightDir/=10.;
          dir = - dir;
          //hppDout(notice,"pos vector is on the wrong side of dir");
        }
      }else{// the current limb is on the right of the root
        //hppDout(notice,"right limb");
        // test if the pos vector is on the left of the dir vector
        if(angle>0){
          weightDir/=10.;
          dir = - dir;
          //hppDout(notice,"pos vector is on the wrong side of dir");
        }
      }
  }// if dir not null

 /* hppDout(notice,"eff position = "<<sample.effectorPosition_);
  hppDout(notice,"limb frame   vl = ["<<sample.configuration_[0]<<","<<sample.configuration_[1]<<","<<sample.configuration_[2]<<","<<pos[0]<<","<<pos[1]<<","<<pos[2]<<" ]");
  hppDout(notice,"direction   vd = ["<<sample.configuration_[0]<<","<<sample.configuration_[1]<<","<<sample.configuration_[2]<<","<<dir[0]<<","<<dir[1]<<","<<dir[2]<<" ]");
  hppDout(notice,"value of dot product = "<<pos.dot(dir));
  hppDout(notice,"static value = "<<sample.staticValue_);*/

    return weightStatic*sample.staticValue_  + weightDir * pos.dot(dir)
        + 1. * pos.dot(fcl::Vec3f(params.comAcceleration_[0],params.comAcceleration_[1],dir[2])) + ((double)rand()) / ((double)(RAND_MAX));
}


double fixedStepHeuristic(const sampling::Sample& sample,
                      const Eigen::Vector3d& direction, const Eigen::Vector3d& normal, const HeuristicParam & params,const double t_step){
    if(! params.comPath_){
        hppDout(notice,"In heuristic : comPath was not provided, use default heuristic");
        return DynamicWalkHeuristic(sample,direction,normal,params);
    }
    //hppDout(notice,"FixedStep heuristic : comPath exist");
    bool success;
    // Compute the 'ideal' contact position in t_step
    Configuration_t q_target = (*params.comPath_)(params.currentPathId_+t_step,success).head<7>();
    Transform3f tRootTarget;
    tRootTarget.setTranslation(fcl::Vec3f(q_target.head<3>()));
    fcl::Quaternion3f quatRoot(q_target[3],q_target[4],q_target[5],q_target[6]);
    tRootTarget.setQuatRotation(quatRoot);
    //hppDout(notice,"heuristic : tRootTarget = "<<tRootTarget);
    fcl::Vec3f pTarget = (tRootTarget * params.limbReferenceOffset_).getTranslation();
    //hppDout(notice,"heuristic : pTarget = ["<<pTarget[0]<<","<<pTarget[1]<<","<<pTarget[2]<<"]");
    // FIXME : we could factorize all of the above and only do it once for each position of the CoM. But this require to know t_step in contact_generation::generate_contact ...
    fcl::Vec3f pSample = (params.tfWorldRoot_ * sample.effectorPosition_).getTranslation();
    return (5.-(pSample-pTarget).squaredNorm()) + sample.staticValue_; // 10 - because it's an heuristic and not a cost
}


double fixedStep1Heuristic(const sampling::Sample& sample,
                      const Eigen::Vector3d& direction, const Eigen::Vector3d& normal, const HeuristicParam & params){
    return fixedStepHeuristic(sample,direction,normal,params,1.);
}

double fixedStep08Heuristic(const sampling::Sample& sample,
                      const Eigen::Vector3d& direction, const Eigen::Vector3d& normal, const HeuristicParam & params){
    return fixedStepHeuristic(sample,direction,normal,params,0.8);
}

double fixedStep06Heuristic(const sampling::Sample& sample,
                      const Eigen::Vector3d& direction, const Eigen::Vector3d& normal, const HeuristicParam & params){
    return fixedStepHeuristic(sample,direction,normal,params,0.6);
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
   // seed = 1491571994; // straight walk 2 m static
    //seed = 1491580336 ; // straight walk dynamic 0.5
    //seed = 1492176551; // walk 0.3 !!
    //seed = 1493208163 ; // stairs static contacts
    //seed = 1504012308; // slalom static
    //seed = 1504099153; // slalom dynamic
    //seed = 1505997146 ; // bauzil walk
    //seed = 1507215254; // bauzil 2
    //seed = 1507302685; // darpa
    //seed = 1510137769; // walk bauzil 2
   // seed = 1511447706; // walk bauzil small steps
  //  seed = 1513330208; // walk straight line with weight static = 10 000 dit = 100
  //  seed = 1513588852; // walk straight line (tested on robot)
  //  seed = 1510645311 ; // darpa line 1 repositionning // step = 0.05
  //  seed = 1510645700 ; // darpa line // step = 0.1
  //  seed = 1516093154; // walk bauzil v=0.3, bof
   // seed = 1516107025;// walk bauzil v=0.2
    //seed = 1516278938; // walk bauzil ; v=0.2 ; a=0.1 ; heuristic = fixedStep06
    //seed = 1517496129 ; //walk test
   // seed = 1517496129 ; // test intersection hyq sideWall (pictures)
   // seed = 1518458850; // sidewall intersection, feasability treshold = 0.1
    //seed = 1518716775; // walk hyq
    //seed = 1519148173; // double step on stairs
    //seed = 1519209574 ; // stairs muscod work
   // seed = 1518867071 ; // sidewall v = 0.2
    //seed = 1519488948 ; // stairs fail
   // seed = 1519638309 ; // new hyq sideWall v0
   // seed = 1519663861 ; // hyq sideWall, vplanning, screen traj
   // seed = 1519655157 ; // stairs w/ ramp screen actuels
    //seed = 1521197622 ; //darpa hrp2 (Justin RSS)
    std::cout<<"seed HEURISTIC = "<<seed<<std::endl;
    srand (seed);
    hppDout(notice,"SEED for heuristic = "<<seed);
    /*std::ofstream fout;
    fout.open("/local/fernbac/bench_iros18/success/log_success.log",std::fstream::app);
    fout<<"seed = "<<seed<<std::endl;
    fout.close();*/
    heuristics_.insert(std::make_pair("static", &StaticHeuristic));
    heuristics_.insert(std::make_pair("EFORT", &EFORTHeuristic));
    heuristics_.insert(std::make_pair("EFORT_Normal", &EFORTNormalHeuristic));
    heuristics_.insert(std::make_pair("manipulability", &ManipulabilityHeuristic));
    heuristics_.insert(std::make_pair("random", &RandomHeuristic));
    heuristics_.insert(std::make_pair("forward", &ForwardHeuristic));
    heuristics_.insert(std::make_pair("dynamicWalk", &DynamicWalkHeuristic));
    heuristics_.insert(std::make_pair("backward", &BackwardHeuristic));
    heuristics_.insert(std::make_pair("jointlimits", &DistanceToLimitHeuristic));
    heuristics_.insert(std::make_pair("dynamic", &dynamicHeuristic));
    heuristics_.insert(std::make_pair("fixedStep1", &fixedStep1Heuristic));
    heuristics_.insert(std::make_pair("fixedStep08", &fixedStep08Heuristic));
    heuristics_.insert(std::make_pair("fixedStep06", &fixedStep06Heuristic));
}

HeuristicFactory::~HeuristicFactory(){}

bool HeuristicFactory::AddHeuristic(const std::string& name, const heuristic func)
{
    if(heuristics_.find(name) != heuristics_.end())
        return false;
    heuristics_.insert(std::make_pair(name,func));
    return true;
}
