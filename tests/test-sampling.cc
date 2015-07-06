// Copyright (C) 2014 LAAS-CNRS
// Author: Steve Tonneau
//
// This file is part of the hpp-rbprm.
//
// hpp-core is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// test-hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-core.  If not, see <http://www.gnu.org/licenses/>.

#include "test-tools.hh"
#include <hpp/rbprm/sampling/sample-container.hh>
#include <hpp/fcl/octree.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/collision.h>

#define BOOST_TEST_MODULE test-sampling
#include <boost/test/included/unit_test.hpp>

using namespace hpp;
using namespace hpp::model;
using namespace rbprm;
using namespace sampling;

namespace
{

DevicePtr_t initDevice()
{
    DevicePtr_t rom = Device::create("rom");


    JointSO3* jointSO3Rom2 = new JointSO3 (fcl::Transform3f(fcl::Vec3f(1,0,0)));

    jointSO3Rom2->name("elbow");
    jointSO3Rom2->isBounded (0, true);
    jointSO3Rom2->isBounded (1, true);
    jointSO3Rom2->isBounded (2, true);
    jointSO3Rom2->lowerBound(0,-3.);
    jointSO3Rom2->upperBound(0,3.);
    jointSO3Rom2->lowerBound(1,-3.);
    jointSO3Rom2->upperBound(1,3.);
    jointSO3Rom2->lowerBound(2,-3.);
    jointSO3Rom2->upperBound(2,3.);

    JointSO3* jointSO3Rom = new JointSO3 (fcl::Transform3f());

    jointSO3Rom->name("arm");
    jointSO3Rom->isBounded (0, true);
    jointSO3Rom->isBounded (1, true);
    jointSO3Rom->isBounded (2, true);
    jointSO3Rom->lowerBound(0,-3.);
    jointSO3Rom->upperBound(0,3.);
    jointSO3Rom->lowerBound(1,-3.);
    jointSO3Rom->upperBound(1,3.);
    jointSO3Rom->lowerBound(2,-3.);
    jointSO3Rom->upperBound(2,3.);

    JointTranslation<3>* jointTrRom = new JointTranslation<3> (fcl::Transform3f());

    jointTrRom->isBounded (0, true);
    jointTrRom->isBounded (1, true);
    jointTrRom->isBounded (2, true);
    jointTrRom->lowerBound(0,0.);
    jointTrRom->upperBound(0,0.);
    jointTrRom->lowerBound(1,0.);
    jointTrRom->upperBound(1,0.);
    jointTrRom->lowerBound(2,0.);
    jointTrRom->upperBound(2,0.);

    rom->rootJoint(jointTrRom);
    jointTrRom->addChildJoint (jointSO3Rom);
    jointSO3Rom->addChildJoint (jointSO3Rom2);

    CollisionGeometryPtr_t romcg (new fcl::Box (1, 1, 1));
    CollisionObjectPtr_t obstacleRom = CollisionObject::create
        (romcg, fcl::Transform3f (), "rombox");

    obstacleRom->move(fcl::Vec3f(0,0,0));
    BodyPtr_t body = new Body;
    body->name ("rom");
    jointTrRom->setLinkedBody (body);
    body->addInnerObject(obstacleRom, true, true);
    return rom;
}


bool tg()
{

    CollisionObjectPtr_t obstacle =  MeshObstacleBox();
    DevicePtr_t robot = initDevice();
    JointPtr_t joint = robot->getJointByName("arm");
    SampleContainer sc(joint,100,0.1);
    fcl::DistanceResult result;
    fcl::DistanceRequest request;
    fcl::distance(&sc.treeObject_, obstacle->fcl().get(), request, result);
    std::cout << result.b1 << "\n" << result.b2 << std::endl;
    fcl::CollisionRequest req(10);
    fcl::CollisionResult res;
    fcl::collide(&sc.treeObject_, obstacle->fcl().get(), req, res);
    std::vector<fcl::Contact> contacts; res.getContacts(contacts);
    for(std::vector<fcl::Contact>::const_iterator cit = contacts.begin();
        cit!= contacts.end(); ++cit)
    {
        std::cout << "contact id " << cit->b1 << std::endl;
        sampling::SampleContainer::T_VoxelSample::const_iterator voxelIt = sc.voxelSamples_.find(cit->b1);
        if(voxelIt != sc.voxelSamples_.end())
        {
            std::cout <<  "found" << std::endl;
            const std::vector<const sampling::Sample*>& samples = voxelIt->second;
            for(std::vector<const sampling::Sample*>::const_iterator sit = samples.begin();
                sit != samples.end(); ++sit)
            {
                std::cout << "sample position \n" << (*sit)->effectorPosition_ << std::endl;
            }
        }
        else
        {
            std::cout <<  "not found" << std::endl;
        }
    }
    bool tg(false);
    return tg;
}

}

BOOST_AUTO_TEST_SUITE(test_generation_samples)

BOOST_AUTO_TEST_CASE (sampleGeneration) {
    DevicePtr_t robot = initDevice();
    JointPtr_t joint = robot->getJointByName("arm");
    std::deque<Sample> res = GenerateSamples(joint, 10);
    for(std::deque<Sample>::const_iterator cit = res.begin();
        cit != res.end(); ++cit)
    {
        const Sample& s = *cit;
        BOOST_CHECK_MESSAGE (s.configuration_.rows()==8,
                                                  "Sample should contain 8  variables");
    }
}

BOOST_AUTO_TEST_CASE (sampleContainerGeneration) {
    DevicePtr_t robot = initDevice();
    JointPtr_t joint = robot->getJointByName("arm");
    SampleContainer sc(joint,10,0.1);
    for(std::deque<Sample>::const_iterator cit = sc.samples_.begin();
        cit != sc.samples_.end(); ++cit)
    {
        const Sample& s = *cit;
        BOOST_CHECK_MESSAGE (s.configuration_.rows()==8,
                                                  "Sample should contain 8  variables");
    }
}


BOOST_AUTO_TEST_CASE (octreeCreation) {
    CollisionObjectPtr_t obstacle =  MeshObstacleBox();
    tg();
}

BOOST_AUTO_TEST_SUITE_END()



