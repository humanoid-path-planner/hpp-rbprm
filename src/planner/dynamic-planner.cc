//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
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

#include <hpp/rbprm/planner/dynamic-planner.hh>
#include <boost/tuple/tuple.hpp>
#include <hpp/util/debug.hh>
#include <hpp/util/timer.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/node.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/path.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/kinodynamic-distance.hh>
#include <hpp/core/steering-method/steering-kinodynamic.hh>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/intersect.h>
#include "utils/algorithms.h"
#include <hpp/core/path-projector.hh>
#include <hpp/rbprm/planner/rbprm-roadmap.hh>
#include <robust-equilibrium-lib/static_equilibrium.hh>



namespace hpp {
  namespace rbprm {
    using model::displayConfig;
    using model::value_type;
    using core::BiRRTPlanner;
    using core::Problem;
    using core::Roadmap;
    using core::RoadmapPtr_t;
    using core::Path;
    using core::PathPtr_t;
    using core::Configuration_t;
    using core::ConfigurationPtr_t;


    typedef robust_equilibrium::MatrixXX MatrixXX;
    typedef robust_equilibrium::Matrix6X Matrix6X;
    typedef robust_equilibrium::Vector3 Vector3;
    typedef robust_equilibrium::Matrix3 Matrix3;
    typedef robust_equilibrium::Matrix63 Matrix63;
    typedef robust_equilibrium::Vector6 Vector6;
    typedef robust_equilibrium::VectorX VectorX;


    DynamicPlannerPtr_t DynamicPlanner::createWithRoadmap
    (const Problem& problem, const RoadmapPtr_t& roadmap)
    {
      DynamicPlanner* ptr = new DynamicPlanner (problem, roadmap);
      return DynamicPlannerPtr_t (ptr);
    }

    DynamicPlannerPtr_t DynamicPlanner::create (const Problem& problem)
    {
      DynamicPlanner* ptr = new DynamicPlanner (problem);
      return DynamicPlannerPtr_t (ptr);
    }

    DynamicPlanner::DynamicPlanner (const Problem& problem):
      BiRRTPlanner (problem),
      qProj_ (new core::Configuration_t(problem.robot()->configSize())),
      roadmap_(boost::dynamic_pointer_cast<core::Roadmap>(core::RbprmRoadmap::create (problem.distance (),problem.robot()))),
      sm_(boost::dynamic_pointer_cast<core::steeringMethod::Kinodynamic>(problem.steeringMethod()))
    {
          assert(sm_ && "steering method should be a kinodynamic steering method for this solver");
    }

    DynamicPlanner::DynamicPlanner (const Problem& problem,
                                    const RoadmapPtr_t& roadmap) :
      BiRRTPlanner (problem, roadmap),
      qProj_ (new core::Configuration_t(problem.robot()->configSize())),
      roadmap_(boost::dynamic_pointer_cast<core::Roadmap>(core::RbprmRoadmap::create (problem.distance (),problem.robot()))),
      sm_(boost::dynamic_pointer_cast<core::steeringMethod::Kinodynamic>(problem.steeringMethod()))
    {
          assert(sm_ && "steering method should be a kinodynamic steering method for this solver");
    }

    void DynamicPlanner::init (const DynamicPlannerWkPtr_t& weak)
    {
      BiRRTPlanner::init (weak);
      weakPtr_ = weak;
    }

    core::PathPtr_t DynamicPlanner::extendInternal (core::ConfigurationPtr_t &qProj_, const core::NodePtr_t& near,
                    const core::ConfigurationPtr_t& target, bool reverse)
    {
        const core::ConstraintSetPtr_t& constraints (sm_->constraints ());
        if (constraints)
        {
            core::ConfigProjectorPtr_t configProjector (constraints->configProjector ());
            if (configProjector)
            {
                configProjector->projectOnKernel (*(near->configuration ()), *target,
                        *qProj_);
            }
            else
            {
                *qProj_ = *target;
            }

            if (constraints->apply (*qProj_))
            {
                setSteeringMethodBounds(near,qProj_,reverse);
                return reverse ? (*sm_) (*qProj_, *(near->configuration ())) : (*sm_) (*(near->configuration ()), *qProj_);
            }
            else
            {
                return PathPtr_t ();
            }
        }
        setSteeringMethodBounds(near,target,reverse);
        return reverse ? (*sm_) (*target, *(near->configuration ())) : (*sm_) (*(near->configuration ()), *target);
    }

    void DynamicPlanner::oneStep ()
    {
      hppDout(info,"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ new Step ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
      PathPtr_t validPath, path;
      core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
      value_type distance;
      core::NodePtr_t near, reachedNodeFromStart;
      bool startComponentConnected(false), pathValidFromStart(false),pathValidFromEnd(false);
      ConfigurationPtr_t q_new;
      // first try to connect to start component
      ConfigurationPtr_t q_rand = configurationShooter_->shoot ();
      hppDout(info,"Random configuration : "<<displayConfig(*q_rand));
      near = roadmap()->nearestNode (q_rand, startComponent_, distance);
      core::RbprmNodePtr_t castNode = static_cast<core::RbprmNodePtr_t>(near);
      if(castNode)
        hppDout(notice,"Node casted correctly");
      else
        hppDout(notice,"Impossible to cast node to rbprmNode");


      path = extendInternal (qProj_, near, q_rand);
      if (path)
      {
        core::PathValidationReportPtr_t report;
        pathValidFromStart = pathValidation->validate (path, false, validPath, report);
        // Insert new path to q_near in roadmap
        if(validPath){
          value_type t_final = validPath->timeRange ().second;
          if (t_final != path->timeRange ().first)
          {
            startComponentConnected = true;
            q_new = ConfigurationPtr_t (new Configuration_t(validPath->end ()));
            reachedNodeFromStart = roadmap()->addNodeAndEdge(near, q_new, validPath);
            computeGIWC(reachedNodeFromStart);
            core::RbprmNodePtr_t castNode2 = static_cast<core::RbprmNodePtr_t>(reachedNodeFromStart);
            if(castNode2)
              hppDout(notice,"Node casted correctly");
            else
              hppDout(notice,"Impossible to cast node to rbprmNode");
            hppDout(info,"~~~~~~~~~~~~~~~~~~~~ New node added to start component : "<<displayConfig(*q_new));

          }
        }
      }

      hppDout(info,"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Try to connect end component ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");

      // now try to connect to end components
      for (std::vector<core::ConnectedComponentPtr_t>::const_iterator itcc =
           endComponents_.begin ();
           itcc != endComponents_.end (); ++itcc)
      {
        near = roadmap()->nearestNode (q_rand, *itcc, distance,true);
        core::RbprmNodePtr_t castNode3 = static_cast<core::RbprmNodePtr_t>(near);
        if(castNode3)
          hppDout(notice,"Node casted correctly");
        else
          hppDout(notice,"Impossible to cast node to rbprmNode");
        path = extendInternal (qProj_, near, q_rand, true);
        if (path)
        {
          core::PathValidationReportPtr_t report;
          pathValidFromEnd = pathValidation->validate (path, true, validPath, report);
          if(pathValidFromEnd && pathValidFromStart)
          {
            // we won, a path is found
            roadmap()->addEdge(reachedNodeFromStart, near, validPath);
            hppDout(info,"~~~~~~~~~~~~~~~~~~~~ Start and goal component connected !!!!!! "<<displayConfig(*q_new));
            return;
          }
          else if (validPath)
          {
            value_type t_final = validPath->timeRange ().second;
            if (t_final != path->timeRange ().first)
            {
              ConfigurationPtr_t q_newEnd = ConfigurationPtr_t (new Configuration_t(validPath->initial()));
              core::NodePtr_t newNode = roadmap()->addNodeAndEdge(q_newEnd, near, validPath);
              computeGIWC(newNode);
              core::RbprmNodePtr_t castNode4 = static_cast<core::RbprmNodePtr_t>(newNode);
              if(castNode4)
                hppDout(notice,"Node casted correctly");
              else
                hppDout(notice,"Impossible to cast node to rbprmNode");
              hppDout(info,"~~~~~~~~~~~~~~~~~~~~~~ New node added to end component : "<<displayConfig(*q_newEnd));

              // now try to connect both nodes
              if(startComponentConnected)
              {
                path = (*(problem().steeringMethod())) (*q_new, *q_newEnd);
                if(path && pathValidation->validate (path, false, validPath, report))
                {
                  roadmap()->addEdge (reachedNodeFromStart, newNode, path);
                  hppDout(info,"~~~~~~~~ both new nodes connected together !!!!!! "<<displayConfig(*q_new));
                  return;
                }
              }
            }
          }
        }
      }
    }



    void DynamicPlanner::computeGIWC(const core::NodePtr_t x){
      core::ValidationReportPtr_t report;
      problem().configValidations()->validate(*(x->configuration()),report);
      computeGIWC(x,report);
    }


    void DynamicPlanner::computeGIWC(const core::NodePtr_t xNode, core::ValidationReportPtr_t report){
      core::RbprmNodePtr_t node = static_cast<core::RbprmNodePtr_t>(xNode);
      hppDout(notice,"## compute GIWC");
      core::ConfigurationPtr_t q = node->configuration();
      // fil normal information in node
      if(node){
        hppDout(info,"~~ NODE cast correctly");
      }else{
        hppDout(error,"~~ NODE cannot be cast");
        return;
      }

      hppDout(info,"~~ q = "<<displayConfig(*q));

      core::RbprmValidationReportPtr_t rbReport = boost::dynamic_pointer_cast<core::RbprmValidationReport> (report);
      // checks : (use assert ? )
      if(!rbReport)
      {
        hppDout(error,"~~ Validation Report cannot be cast");
        return;
      }
      if(rbReport->trunkInCollision)
      {
        hppDout(error,"~~ ComputeGIWC : trunk is in collision"); // shouldn't happen
      }
      if(!rbReport->romsValid)
      {
        hppDout(error,"~~ ComputeGIWC : roms filter not respected"); // shouldn't happen
      }

      //FIX ME : position of contact is in center of the collision surface
      node->setNumberOfContacts((int)rbReport->ROMReports.size());
      hppDout(info,"Number of contacts : "<<node->getNumberOfContacts());
      MatrixXX V = MatrixXX::Zero(3*rbReport->ROMReports.size(),4*rbReport->ROMReports.size());
      Matrix6X IP_hat = Matrix6X::Zero(6,3*rbReport->ROMReports.size());
      MatrixXX Vi;
      // get the 2 object in contact for each ROM :
      hppDout(info,"~~ Number of roms in collision : "<<rbReport->ROMReports.size());
      size_t indexRom = 0 ;
      for(std::map<std::string,core::CollisionValidationReportPtr_t>::const_iterator it = rbReport->ROMReports.begin() ; it != rbReport->ROMReports.end() ; ++it)
      {
        hppDout(info,"~~ for rom : "<<it->first);
        core::CollisionObjectPtr_t obj1 = it->second->object1;
        core::CollisionObjectPtr_t obj2 = it->second->object2;
        hppDout(notice,"~~ collision between : "<<obj1->name() << " and "<<obj2->name());
        fcl::CollisionResult result = it->second->result;
        // debug display :
        /* size_t numContact =result.numContacts();
        hppDout(notice,"~~ number of contact : "<<numContact);
        std::ostringstream ss;
        ss<<"[";
        for(size_t i = 0 ; i < numContact ; i++)
        { // print with python formating :
          ss<<"["<<result.getContact(i).pos[0]<<","<<result.getContact(i).pos[1]<<","<<result.getContact(i).pos[2]<<"]";
          if(i< (numContact-1))
             ss<<",";
        }
        ss<<"]";
        std::cout<<ss.str()<<std::endl;
      */

        // get intersection between the two objects :
        obj1->fcl();
        geom::T_Point vertices1;
        geom::BVHModelOBConst_Ptr_t model1 =  geom::GetModel(obj1->fcl());
        hppDout(info,"vertices obj1 : "<<obj1->name()<< " ( "<<model1->num_vertices<<" ) ");
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
        //std::cout<<ss1.str()<<std::endl;


        obj2->fcl();
        geom::T_Point vertices2;
        geom::BVHModelOBConst_Ptr_t model2 =  geom::GetModel(obj2->fcl());
        hppDout(info,"vertices obj2 : "<<obj2->name()<< " ( "<<model2->num_vertices<<" ) ");
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
        //std::cout<<ss2.str()<<std::endl;





        hppStartBenchmark (COMPUTE_INTERSECTION);
        // FIX ME : compute plan equation first
        geom::T_Point hull = geom::intersectPolygonePlane(model1,model2,fcl::Vec3f(0,0,1),geom::ZJUMP,result);
        hppStopBenchmark (COMPUTE_INTERSECTION);
        hppDisplayBenchmark (COMPUTE_INTERSECTION);
        /*
        if(hull.size() == 0){
          hppDout(error,"No intersection between rom and environnement");
         // TODO switch to new data structure create matrix with good size to avoid crash ?
          return;
        }

        // compute center point of the hull
        geom::Point center = geom::center(hull.begin(),hull.end());
        */
        // FIXME : temporary, for test only
        geom::Point center;
        center << (*node->configuration())[0],(*node->configuration())[1], 0;
        if(indexRom == 0){ //front left
          center[0] -= 0.3;
          center[1] += 0.3;
        }else if(indexRom == 1){ //front right
          center[0] += 0.3;
          center[1] += 0.3;
        }else if(indexRom == 2){ //back right
          center[0] += 0.3;
          center[1] -= 0.3;
        }else{ //back left
          center[0] -= 0.3;
          center[1] -= 0.3;
        }

        //fill IP_hat with position : [I_3  pi_hat] ^T
        Vector3 ni,ti1,ti2;
        IP_hat.block<3,3>(0,3*indexRom) = MatrixXX::Identity(3,3);
        IP_hat.block<3,3>(3,3*indexRom) = robust_equilibrium::crossMatrix(center);

        //hppDout(notice,"Center of rom collision :  ["<<center[0]<<" , "<<center[1]<<" , "<<center[2]<<"]");
        hppDout(info,"p"<<indexRom<<" = "<<center.transpose());
        hppDout(info,"IP_hat at iter "<<indexRom<< " = \n"<<IP_hat);
        ni = -result.getContact(0).normal;
        node->normal(ni);
        hppDout(notice,"normal for this contact : "<<node->getNormal());
        // compute tangent vector :
        ti1 = ni.cross(Vector3(1,0,0));
        if(ti1.dot(ti1)<0.001)
          ti1 = ni.cross(Vector3(0,1,0));
        ti2 = ni.cross(ti1);

        hppDout(info,"t"<<indexRom<<"1 : "<<ti1.transpose());
        hppDout(info,"t"<<indexRom<<"2 : "<<ti2.transpose());

        //TODO : fill V with generating ray ([ n_i + \mu t_{i1} & n_i - \mu t_{i1} & n_i + \mu t_{i2} & n_i - \mu t_{i2}]
        Vi = MatrixXX::Zero(3,4);
        Vi.col(0) = ni + mu*ti1;
        Vi.col(1) = ni - mu*ti1;
        Vi.col(2) = ni + mu*ti2;
        Vi.col(3) = ni - mu*ti2;
        V.block<4,3>(3*indexRom,4*indexRom) = Vi;
        hppDout(info,"V at iter "<<indexRom<<" : \n"<<V);

        indexRom++;
      } // for each ROMS


      // save infos needed for LP problem in node structure
      node->setV(V);
      node->setIPHat(IP_hat);

      // compute other LP values : (constant for each nodes)
      node->setG(IP_hat*V);
      double m = problem().robot()->mass();
      Vector3 c(3);
      c << (*node->configuration())[0],(*node->configuration())[1],(*node->configuration())[2];
      Matrix63 H = Matrix63::Zero(6,3);
      H.block<3,3>(0,0) = Matrix3::Identity(3,3);
      H.block<3,3>(3,0) = robust_equilibrium::crossMatrix(c);
      node->setH(m*H);
      Vector6 h = Vector6::Zero(6);
      Vector3 g;
      g<< 0,0,-9.81 ; // FIXME : retrieve it from somewhere ? instead of hardcoded
      h.head(3) = g;
      h.tail(3) = c.cross(-g);

      // debug output :
      hppDout(info,"G = "<<node->getG());
      hppDout(info,"c = "<<c);
      hppDout(info,"m = "<<m);
      hppDout(info,"h = "<<node->geth());
      hppDout(info,"H = "<<node->getH());


    }// computeGIWC


    // re implement virtual method, same as base class but without the symetric edge (goal -> start)
    void DynamicPlanner::tryDirectPath ()
    {
      // call steering method here to build a direct conexion
      const core::SteeringMethodPtr_t& sm (problem ().steeringMethod ());
      core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
      core::PathProjectorPtr_t pathProjector (problem ().pathProjector ());
      core::PathPtr_t validPath, projPath, path;
      core::NodePtr_t initNode = roadmap ()->initNode();
      computeGIWC(initNode);
      for (core::Nodes_t::const_iterator itn = roadmap ()->goalNodes ().begin();
           itn != roadmap ()->goalNodes ().end (); ++itn) {
        computeGIWC(*itn);
        core::ConfigurationPtr_t q1 ((initNode)->configuration ());
        core::ConfigurationPtr_t q2 ((*itn)->configuration ());
        assert (*q1 != *q2);
        path = (*sm) (*q1, *q2);
        if (!path) continue;
        if (pathProjector) {
          if (!pathProjector->apply (path, projPath)) continue;
        } else {
          projPath = path;
        }
        if (projPath) {
          core::PathValidationReportPtr_t report;
          bool pathValid = pathValidation->validate (projPath, false, validPath,
                                                     report);
          if (pathValid && validPath->timeRange ().second !=
              path->timeRange ().first) {
            roadmap ()->addEdge (initNode, *itn, projPath);
          }
        }
      }
    }

    void DynamicPlanner::setSteeringMethodBounds(const core::NodePtr_t& near, const core::ConfigurationPtr_t target,bool reverse){
      //TODO compute the maximal acceleration from near, on a direction from near to target (oposite if revezrse == true)
      core::RbprmNodePtr_t node = static_cast<core::RbprmNodePtr_t>(near);
      assert(node && "Unable to cast near node to rbprmNode");

      // compute direction (v) :

      double alpha0=1.; // main variable of our LP problem
      Vector3 to,from,v;
      if(reverse){
        to = near->configuration()->head(3);
        from = target->head(3);
      }else{
        from = near->configuration()->head(3);
        to = target->head(3);
      }
      v = (to - from);
      v.normalize();
      hppDout(info,"from = "<<from);
      hppDout(info,"to   = "<<to);
      hppDout(info, "Direction of motion v = "<<v);

      // define LP problem : with m+1 variables and m+6 constraints
      int m = node->getNumberOfContacts() * 4;

      MatrixXX A = MatrixXX::Zero(6+m, m+1);
      // build A : [ -G (Hv)^T] :
      A.topLeftCorner(6,m) = - node->getG();
      MatrixXX Hv = (node->getH() * v);
      assert(Hv.rows() == 6 && Hv.cols()==1 && "Hv should be a vector 6");
      A.topRightCorner(6,1) = Hv;
      hppDout(info,"H = "<<node->getH());
      hppDout(info," Hv^T = "<<Hv);
      hppDout(info,"A = "<<A);
/*
      // call to robust_equilibrium_lib :
      //FIX ME : build it only once and store it as attribut ?
      robust_equilibrium::StaticEquilibrium sEq(problem().robot()->name(), problem().robot()->mass(),4,robust_equilibrium::SOLVER_LP_QPOASES,true,10,false);
      sEq.findMaximumAcceleration(A, node->geth(),alpha0);

      hppDout(info,"Amax found : "<<alpha0);*/
      sm_->setAmax(alpha0*v);
      sm_->setVmax(1.*v); //FIXME: read it from somewhere ?
    }


  } // namespace core
} // namespace hpp

