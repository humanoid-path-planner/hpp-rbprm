//
// Copyright (c) 2017 CNRS
// Authors: Fernbach Pierre
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

#include <hpp/rbprm/planner/rbprm-node.hh>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/intersect.h>
#include "hpp/rbprm/utils/algorithms.h"
#include <hpp/util/debug.hh>
#include <hpp/util/timer.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/core/collision-validation-report.hh>
#include <hpp/rbprm/rbprm-device.hh>
namespace hpp{
namespace core{

typedef centroidal_dynamics::MatrixXX MatrixXX;
typedef centroidal_dynamics::Matrix6X Matrix6X;
typedef centroidal_dynamics::Vector3 Vector3;
typedef centroidal_dynamics::Matrix3 Matrix3;
typedef centroidal_dynamics::Matrix63 Matrix63;
typedef centroidal_dynamics::Vector6 Vector6;
typedef centroidal_dynamics::VectorX VectorX;

Eigen::Quaterniond RbprmNode::getQuaternion(){
    ConfigurationPtr_t q=configuration();
    Eigen::Quaterniond quat((*q)[6],(*q)[3],(*q)[4],(*q)[5]);
    return quat;
}

bool computeIntersectionSurface(const core::CollisionValidationReportPtr_t report,geom::T_Point& inter, geom::Point& pn,pinocchio::DeviceData& deviceData){

    core::CollisionObjectConstPtr_t obj_rom = report->object1;
    core::CollisionObjectConstPtr_t obj_env = report->object2;
    //hppDout(notice,"~~ compute intersection between : "<<obj_rom->name() << " and "<<obj_env->name());
    // convert the two objects  :
    geom::BVHModelOBConst_Ptr_t model_rom =  geom::GetModel(obj_rom,deviceData);
    geom::BVHModelOBConst_Ptr_t model_env =  geom::GetModel(obj_env,deviceData);

    hppStartBenchmark (COMPUTE_INTERSECTION);
    geom::T_Point plane = geom::intersectPolygonePlane(model_rom,model_env,pn);
    // plane contains a list of points : the intersections between model_rom and the infinite plane defined by model_env.
    // but they may not be contained inside the shape defined by model_env
    hppStopBenchmark (COMPUTE_INTERSECTION);
    hppDisplayBenchmark (COMPUTE_INTERSECTION);
    if(plane.size() > 0)
      inter = geom::compute3DIntersection(plane,geom::convertBVH(model_env)); // hull contain only points inside the model_env shape
    else
      return false;
    if(inter.size() == 0)
      return false;
    else
      return true;

}


bool centerOfRomIntersection(const core::CollisionValidationReportPtr_t report, geom::Point& pn, geom::Point& center,pinocchio::DeviceData& deviceData){

    geom::T_Point hull;
    bool success  = computeIntersectionSurface(report,hull,pn,deviceData);
    if(!success)
      return false;
    // compute center point of the hull
    center = geom::center(hull.begin(),hull.end());

    hppDout(notice,"Center : "<<center.transpose());
    hppDout(notice,"Normal : "<<pn.transpose());
    return true;
}

/**
 * @brief approximateContactPoint Compute the approximation of the contact Point.
 * Current implementation : closest point of the reference config inside the contact surface.
 * @param report the collision report
 * @param pn output the normal of the contact
 * @param result output the contact point
 * @param config configuration of the robot (only the root's configuration is used here)
 * @param device
 * @return bool success
 */
bool approximateContactPoint(const std::string romName,const core::CollisionValidationReportPtr_t report, geom::Point& pn, geom::Point& result,core::ConfigurationPtr_t config,pinocchio::RbPrmDevicePtr_t device){
    geom::T_Point hull;
    pinocchio::DeviceSync deviceSync (device);
    hppDout(notice,"Approximate contact point for rom "<<romName);
    bool success  = computeIntersectionSurface(report,hull,pn,deviceSync.d());
    //hppDout(notice,"Number of points in the intersection : "<<hull.size());
    if(success){
      fcl::Vec3f reference = device->getEffectorReference(romName);
      if(reference.norm() != 0){
        //hppDout(notice,"Reference position for rom"<<romName<<" = "<<reference.transpose());
        fcl::Transform3f tRoot;
        tRoot.setTranslation(fcl::Vec3f((*config)[0],(*config)[1],(*config)[2]));
        fcl::Quaternion3f quat((*config)[6],(*config)[3],(*config)[4],(*config)[5]);
        tRoot.setRotation(quat.matrix());
        reference = (tRoot*reference).getTranslation();
        geom::Point refPoint(reference);
        //hppDout(notice,"Reference after root transform = "<<refPoint.transpose());
        geom::projectPointInsidePlan(hull,refPoint,pn,hull.front(),result);
        hppDout(notice,"Approximate contact point found : "<<result.transpose());
        return true;
      }else{
        hppDout(notice,"No reference end effector position defined, use center of intersection as approximation");
        result = geom::center(hull.begin(),hull.end());
        return true;
      }
    }else{
      hppDout(notice,"Error in the approximation of the ocntact point");
      return false;
    }
}

void computeNodeMatrixForOnePoint(const geom::Point pn, const geom::Point center, const double sizeFootX,const double sizeFootY,const bool rectangularContact, const double mu, const Eigen::Quaterniond quat, size_t& indexRom, MatrixXX& V, Matrix6X& IP_hat){
    Vector3 ti1,ti2;
    //hppDout(notice,"normal for this contact : "<<getNormal());
    hppDout(notice,"mu used in computeNodeMatrix : "<<mu);
    // compute tangent vector :
    //tProj is the the direction of the head of the robot projected in plan (x,y)
    Vector3 tProj = quat*Vector3(1,0,0);
    tProj[2] = 0;
    tProj.normalize();
    ti1 = pn.cross(tProj);
    if(ti1.dot(ti1)<0.001)
        ti1 = pn.cross(Vector3(1,0,0));
    if(ti1.dot(ti1)<0.001)
        ti1 = pn.cross(Vector3(0,1,0));
    ti2 = pn.cross(ti1);

    // hppDout(info,"t"<<indexRom<<"1 : "<<ti1.transpose());
    // hppDout(info,"t"<<indexRom<<"2 : "<<ti2.transpose());

    //fill V with generating ray ([ n_i + \mu t_{i1} & n_i - \mu t_{i1} & n_i + \mu t_{i2} & n_i - \mu t_{i2}]
    MatrixXX Vi = MatrixXX::Zero(3,4);
    Vi.col(0) = (pn + mu*ti1);
    Vi.col(1) = (pn - mu*ti1);
    Vi.col(2) = (pn + mu*ti2);
    Vi.col(3) = (pn - mu*ti2);
    for(size_t i = 0 ; i<4 ; i++)
        Vi.col(i).normalize();

    if(rectangularContact){
        Vector3 pContact;
        Vector3 shiftX,shiftY;
        shiftX = sizeFootX*ti2;
        shiftY = sizeFootY*ti1;


        // hppDout(notice,"shift x = "<<shiftX.transpose());
        // hppDout(notice,"shift y = "<<shiftY.transpose());

        hppDout(notice,"Center of rom collision :  ["<<center[0]<<" , "<<center[1]<<" , "<<center[2]<<"]");
        for(size_t i = 0 ; i<4 ; ++i){
            // make a rectangle around center :
            pContact = center;
            if(i < 2 )
                pContact += shiftX;
            else
                pContact -= shiftX;
            if(i%2 == 0)
                pContact += shiftY;
            else
                pContact -= shiftY;

            //fill IP_hat with position : [I_3  pi_hat] ^T
            IP_hat.block<3,3>(0,3*(indexRom+i)) = MatrixXX::Identity(3,3);
            IP_hat.block<3,3>(3,3*(indexRom+i)) = centroidal_dynamics::crossMatrix(pContact);

            hppDout(notice,"position of rom collision :  ["<<pContact[0]<<" , "<<pContact[1]<<" , "<<pContact[2]<<"]");
            // ssContacts<<"["<<pContact[0]<<" , "<<pContact[1]<<" , "<<pContact[2]<<"],";
            // hppDout(info,"p"<<(indexRom+i)<<"^T = "<<pContact.transpose());
            //hppDout(info,"IP_hat at iter "<<indexRom<< " = \n"<<IP_hat);

            //hppDout(notice,"V"<<indexRom<<" = \n"<<Vi);
            V.block<3,4>(3*(indexRom+i),4*(indexRom+i)) = Vi;
            // hppDout(info,"V at iter "<<indexRom<<" : \n"<<V);
        }
        indexRom+=4;
    }else{
        //fill IP_hat with position : [I_3  pi_hat] ^T
        IP_hat.block<3,3>(0,3*indexRom) = MatrixXX::Identity(3,3);
        IP_hat.block<3,3>(3,3*indexRom) = centroidal_dynamics::crossMatrix(center);

        //hppDout(notice,"Center of rom collision :  ["<<center[0]<<" , "<<center[1]<<" , "<<center[2]<<"]");
        hppDout(info,"p"<<indexRom<<"^T = "<<center.transpose());
        //hppDout(info,"IP_hat at iter "<<indexRom<< " = \n"<<IP_hat);

        // hppDout(notice,"V"<<indexRom<<" = \n"<<Vi);
        V.block<3,4>(3*indexRom,4*indexRom) = Vi;
        //hppDout(info,"V at iter "<<indexRom<<" : \n"<<V);
        indexRom++;
    }

}

void RbprmNode::fillNodeMatrices(ValidationReportPtr_t report, bool rectangularContact, double sizeFootX, double sizeFootY, double m, double mu, pinocchio::RbPrmDevicePtr_t device){
    assert(device && "Error in dynamic cast of problem device to rbprmDevice");
    hppStartBenchmark(FILL_NODE_MATRICE);

    core::RbprmValidationReportPtr_t rbReport = boost::dynamic_pointer_cast<core::RbprmValidationReport> (report);
    // checks : (use assert ? )
    if(!rbReport)
    {
        hppDout(error,"~~ Validation Report cannot be cast");
        return;
    }
    collisionReport(rbReport);

    if(rbReport->trunkInCollision)
    {
        hppDout(error,"~~ ComputeGIWC : trunk is in collision"); // shouldn't happen
    }
    if(!rbReport->romsValid)
    {
        hppDout(error,"~~ ComputeGIWC : roms filter not respected"); // shouldn't happen
    }

    hppDout(notice,"Robot mass used to compute matrices : "<<m);
    assert(m>0. && " Robot mass during computation of dynamic matrices is 0.");
    //FIX ME : position of contact is in center of the collision surface
    size_type numContactpoints = (rbReport->ROMReports.size() + 3*rectangularContact*rbReport->ROMReports.size());
    setNumberOfContacts(numContactpoints);
    hppDout(notice,"number of contact points = "<<numContactpoints);
    MatrixXX V = MatrixXX::Zero(3*numContactpoints,4*numContactpoints);
    Matrix6X IP_hat = Matrix6X::Zero(6,3*numContactpoints);
    // get the 2 object in contact for each ROM :
    hppDout(info,"~~ Number of roms in collision : "<<rbReport->ROMReports.size());
    size_t indexRom = 0 ;
    std::ostringstream ssContact;
    ssContact<<"[";
    bool pointExist;
    for(std::map<std::string,core::CollisionValidationReportPtr_t>::const_iterator it = rbReport->ROMReports.begin() ; it != rbReport->ROMReports.end() ; ++it)
    {
        hppDout(info,"~~ for rom : "<<it->first);
        geom::Point pn,contactPoint;
        pointExist = approximateContactPoint(it->first,it->second,pn,contactPoint,configuration(),device);
        ssContact<<"["<<contactPoint[0]<<" , "<<contactPoint[1]<<" , "<<contactPoint[2]<<"],";

        if(!pointExist){
            hppDout(error,"Unable to compute the approximation of the contact point");
            // save infos needed for LP problem in node structure
            // FIXME : Or retry with another obstacle ???
            setV(V);
            setIPHat(IP_hat);

            // compute other LP values : (constant for each nodes)
            setG(IP_hat*V);
            Vector3 c(3);
            c << (*configuration())[0],(*configuration())[1],(*configuration())[2];
            Matrix63 H = Matrix63::Zero(6,3);
            H.block<3,3>(0,0) = Matrix3::Identity(3,3);
            H.block<3,3>(3,0) = centroidal_dynamics::crossMatrix(c);
            setH(m*H);
            Vector6 h = Vector6::Zero(6);
            Vector3 g;
            g<< 0,0,-9.81 ; // FIXME : retrieve it from somewhere ? instead of hardcoded
            h.head(3) = -g;
            h.tail(3) = c.cross(-g);
            seth(m*h);
        }

        computeNodeMatrixForOnePoint(pn,contactPoint,sizeFootX,sizeFootY,rectangularContact, mu,getQuaternion(), indexRom, V,IP_hat);


    } // for each ROMS


    // save infos needed for LP problem in node structure
    setV(V);
    setIPHat(IP_hat);

    // compute other LP values : (constant for each nodes)
    setG(IP_hat*V);
    Vector3 c(3);
    c << (*configuration())[0],(*configuration())[1],(*configuration())[2];
    Matrix63 H = Matrix63::Zero(6,3);
    H.block<3,3>(0,0) = Matrix3::Identity(3,3);
    H.block<3,3>(3,0) = centroidal_dynamics::crossMatrix(c);
    setH(m*H);
    Vector6 h = Vector6::Zero(6);
    Vector3 g;
    g<< 0,0,-9.81 ; // FIXME : retrieve it from somewhere ? instead of hardcoded
    h.head(3) = -g;
    h.tail(3) = c.cross(-g);
    seth(m*h);

    // debug output :
    /*hppDout(info,"G = \n"<<getG());
      hppDout(info,"c^T = "<<c.transpose());
      hppDout(info,"m = "<<m);
      hppDout(info,"h^T = "<<geth().transpose());
      hppDout(info,"H = \n"<<getH());
*/
    hppDout(notice,"list of all centers = "<<ssContact.str()<<"]");

    hppStopBenchmark(FILL_NODE_MATRICE);
    hppDisplayBenchmark(FILL_NODE_MATRICE);

}

void RbprmNode::chooseBestContactSurface(ValidationReportPtr_t report, pinocchio::RbPrmDevicePtr_t device){
    assert(device && "Error in dynamic cast of problem device to rbprmDevice");
    core::RbprmValidationReportPtr_t rbReport = boost::dynamic_pointer_cast<core::RbprmValidationReport> (report);
    for(std::map<std::string,core::CollisionValidationReportPtr_t>::const_iterator it = rbReport->ROMReports.begin() ; it != rbReport->ROMReports.end() ; ++it){
        core::AllCollisionsValidationReportPtr_t romReports = boost::dynamic_pointer_cast<core::AllCollisionsValidationReport>(it->second);
        if(!romReports){
            hppDout(warning,"For rom : "<<it->first<<" unable to cast in a AllCollisionsValidationReport, did you correctly call computeAllContacts(true) before generating the report ? ");
            return;
        }
        if(romReports->collisionReports.size()> 1){
          // for each rom , for each different environnement surface in collision with the rom:
          // 1) compute the projection of the reference position inside the intersection of the rom and the environnement surface
          // 2) find the surface with the minimal distance between the reference and the projection
          // 3) modify the report such that this surface is at the top of the list

          fcl::Vec3f reference = device->getEffectorReference(it->first);
          hppDout(notice,"Reference position for rom"<<it->first<<" = "<<reference);
          core::ConfigurationPtr_t q = configuration();
          fcl::Transform3f tRoot;
          tRoot.setTranslation(fcl::Vec3f((*q)[0],(*q)[1],(*q)[2]));
          fcl::Quaternion3f quat((*q)[6],(*q)[3],(*q)[4],(*q)[5]);
          //fcl::Matrix3f rot = quat.matrix();
          tRoot.setRotation(quat.matrix());
          reference = (tRoot*reference).getTranslation();
          geom::Point refPoint(reference);
          hppDout(notice,"Reference after root transform = "<<reference);
          geom::Point normal,proj;
          double minDistance = std::numeric_limits<double>::max();
          double distance;
          CollisionValidationReportPtr_t bestReport(romReports->collisionReports.front());
          bool successInter;
          geom::T_Point intersection;
          hppDout(notice,"Number of possible surfaces for rom : "<<romReports->collisionReports.size());
          for(std::vector<CollisionValidationReportPtr_t>::const_iterator itAff = romReports->collisionReports.begin() ; itAff != romReports->collisionReports.end() ; ++itAff){
            pinocchio::DeviceSync deviceSync (device);
            successInter = computeIntersectionSurface(*itAff,intersection,normal,deviceSync.d());
              if(successInter){
                distance = geom::projectPointInsidePlan(intersection,refPoint,normal,intersection.front(),proj);
                hppDout(notice,"Distance found : "<<distance);
                if(distance < minDistance){
                    minDistance = distance;
                    bestReport = *itAff;
                }
            }
          }
          //3)
          romReports->object1 = bestReport->object1;
          romReports->object2 = bestReport->object2;
          romReports->result = bestReport->result;
        }
    } // end for all rom

}


}//core
}//hpp
