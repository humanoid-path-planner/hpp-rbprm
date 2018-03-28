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
#include "utils/algorithms.h"
#include <hpp/util/debug.hh>
#include <hpp/util/timer.hh>
#include <hpp/model/configuration.hh>
namespace hpp{
  namespace core{

    typedef centroidal_dynamics::MatrixXX MatrixXX;
    typedef centroidal_dynamics::Matrix6X Matrix6X;
    typedef centroidal_dynamics::Vector3 Vector3;
    typedef centroidal_dynamics::Matrix3 Matrix3;
    typedef centroidal_dynamics::Matrix63 Matrix63;
    typedef centroidal_dynamics::Vector6 Vector6;
    typedef centroidal_dynamics::VectorX VectorX;

    void RbprmNode::fillNodeMatrices(ValidationReportPtr_t report, bool rectangularContact, double sizeFootX, double sizeFootY, double m,double mu){
      hppStartBenchmark(FILL_NODE_MATRICE);
      core::ConfigurationPtr_t q = configuration();

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


      //FIX ME : position of contact is in center of the collision surface
      int numContactpoints = (rbReport->ROMReports.size() + 3*rectangularContact*rbReport->ROMReports.size());
      setNumberOfContacts(numContactpoints);
      hppDout(notice,"number of contact points = "<<numContactpoints);
      MatrixXX V = MatrixXX::Zero(3*numContactpoints,4*numContactpoints);
      Matrix6X IP_hat = Matrix6X::Zero(6,3*numContactpoints);
      MatrixXX Vi;
      // get the 2 object in contact for each ROM :
      hppDout(info,"~~ Number of roms in collision : "<<rbReport->ROMReports.size());
      size_t indexRom = 0 ;
      std::ostringstream ssContacts,ssCenters;
      ssContacts<<"[";
      ssCenters<<"[";
      for(std::map<std::string,core::CollisionValidationReportPtr_t>::const_iterator it = rbReport->ROMReports.begin() ; it != rbReport->ROMReports.end() ; ++it)
      {
        //hppDout(info,"~~ for rom : "<<it->first);
        core::CollisionObjectPtr_t obj1 = it->second->object1;
        core::CollisionObjectPtr_t obj2 = it->second->object2;
        //hppDout(notice,"~~ collision between : "<<obj1->name() << " and "<<obj2->name());
        fcl::CollisionResult result = it->second->result;
        // debug display :
        size_t numContact =result.numContacts();
        //hppDout(notice,"~~ number of contact : "<<numContact);
    /*    std::ostringstream ss;
        ss<<"[";
        for(size_t i = 0 ; i < numContact ; i++)
        { // print with python formating :
          ss<<"["<<result.getContact(i).pos[0]<<","<<result.getContact(i).pos[1]<<","<<result.getContact(i).pos[2]<<"]";
          if(i< (numContact-1))
             ss<<",";
        }
        ss<<"]";
        */
       // std::cout<<"contact point : "<<std::endl;
       // std::cout<<ss.str()<<std::endl;


        // get intersection between the two objects :
        geom::T_Point vertices1;
        geom::BVHModelOBConst_Ptr_t model1 =  geom::GetModel(obj1->fcl());
        geom::T_Point vertices2;
        geom::BVHModelOBConst_Ptr_t model2 =  geom::GetModel(obj2->fcl());

/*      // display intersection for debug
        //hppDout(info,"vertices obj1 : "<<obj1->name()<< " ( "<<model1->num_vertices<<" ) ");
        std::ostringstream ss1;
        ss1<<"[";
        for(int i = 0 ; i < model1->num_vertices ; ++i)
        {
          vertices1.push_back(Eigen::Vector3d(model1->vertices[i][0], model1->vertices[i][1], model1->vertices[i][2]));
          //hppDout(notice,"vertices : "<<model1->vertices[i]);
          ss1<<"["<<model1->vertices[i][0]<<","<<model1->vertices[i][1]<<","<<model1->vertices[i][2]<<"]";
          if(i< (model1->num_vertices-1))
            ss1<<",";
        }
        ss1<<"]";
        std::cout<<"obj "<<obj1->name()<<std::endl;
        std::cout<<ss1.str()<<std::endl;


        //hppDout(info,"vertices obj2 : "<<obj2->name()<< " ( "<<model2->num_vertices<<" ) ");
        std::ostringstream ss2;
        ss2<<"[";
        for(int i = 0 ; i < model2->num_vertices ; ++i)
        {
          vertices2.push_back(Eigen::Vector3d(model2->vertices[i][0], model2->vertices[i][1], model2->vertices[i][2]));
          // hppDout(notice,"vertices : "<<model2->vertices[i]);
          ss2<<"["<<model2->vertices[i][0]<<","<<model2->vertices[i][1]<<","<<model2->vertices[i][2]<<"]";
          if(i< (model2->num_vertices -1))
            ss2<<",";

        }
        ss2<<"]";
       // std::cout<<"obj "<<obj2->name()<<std::endl;
       // std::cout<<ss2.str()<<std::endl;
        hppDout(notice," "<<ss2.str());
*/




        hppStartBenchmark (COMPUTE_INTERSECTION);
        geom::Point pn;
        // FIX ME : compute plan equation first
        geom::T_Point plane = geom::intersectPolygonePlane(model1,model2,pn);
        hppStopBenchmark (COMPUTE_INTERSECTION);
        hppDisplayBenchmark (COMPUTE_INTERSECTION);
        geom::T_Point hull;
        if(plane.size() > 0)
          hull = geom::compute3DIntersection(plane,geom::convertBVH(model2));


        if(hull.size() == 0){
          hppDout(error,"No intersection between rom and environnement");
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
          return ;
        }

        // compute center point of the hull
        geom::Point center = geom::center(hull.begin(),hull.end());

        hppDout(notice,"Center : "<<center.transpose());
        hppDout(notice,"Normal : "<<pn.transpose());



        normal(pn);
        Vector3 ti1,ti2;
        //hppDout(notice,"normal for this contact : "<<getNormal());
        // compute tangent vector :
        //tProj is the the direction of the head of the robot projected in plan (x,y)
        Eigen::Quaterniond quat((*q)[3],(*q)[4],(*q)[5],(*q)[6]);
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
        Vi = MatrixXX::Zero(3,4);
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
          ssCenters<<"["<<center[0]<<" , "<<center[1]<<" , "<<center[2]<<"],";
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
            ssContacts<<"["<<pContact[0]<<" , "<<pContact[1]<<" , "<<pContact[2]<<"],";
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
          ssCenters<<"["<<center[0]<<" , "<<center[1]<<" , "<<center[2]<<"],";
          hppDout(info,"p"<<indexRom<<"^T = "<<center.transpose());
          //hppDout(info,"IP_hat at iter "<<indexRom<< " = \n"<<IP_hat);

         // hppDout(notice,"V"<<indexRom<<" = \n"<<Vi);
          V.block<3,4>(3*indexRom,4*indexRom) = Vi;
          //hppDout(info,"V at iter "<<indexRom<<" : \n"<<V);
          indexRom++;
        }

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
      hppDout(notice,"list of all contacts = "<<ssContacts.str()<<"]");
      hppDout(notice,"list of all centers = "<<ssCenters.str()<<"]");

      hppStopBenchmark(FILL_NODE_MATRICE);
      hppDisplayBenchmark(FILL_NODE_MATRICE);

    }


  }//core
}//hpp
