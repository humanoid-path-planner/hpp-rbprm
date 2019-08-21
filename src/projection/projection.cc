//
// Copyright (c) 2017 CNRS
// Authors: Steve Tonneau
//
// This file is part of hpp-rbprm
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

#include <hpp/rbprm/projection/projection.hh>
#include <hpp/rbprm/interpolation/interpolation-constraints.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/constraints/relative-com.hh>
#include <hpp/constraints/symbolic-calculus.hh>
#include <hpp/constraints/symbolic-function.hh>

#ifdef PROFILE
    #include "hpp/rbprm/rbprm-profiler.hh"
#endif




namespace hpp {
namespace rbprm {
namespace projection{

const double epsilon = 10e-4;

ProjectionReport::ProjectionReport(const ProjectionReport& from)
    : success_(from.success_)
    , result_(from.result_)
    , status_(from.status_)
{
    // NOTHING
}


std::vector<bool> setMaintainRotationConstraints()
{
    std::vector<bool> res;
    for(std::size_t i =0; i <3; ++i)
        res.push_back(true);
    return res;
}

void CreateContactConstraints(hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const hpp::rbprm::State& currentState, core::ConfigProjectorPtr_t proj)
{
    pinocchio::DevicePtr_t device = fullBody->device_;
    std::vector<bool> cosntraintsR = setMaintainRotationConstraints();
    std::vector<std::string> fixed = currentState.fixedContacts(currentState);
    for(std::vector<std::string>::const_iterator cit = fixed.begin(); cit != fixed.end(); ++cit)
    {
        const std::string& effector = *cit;
        RbPrmLimbPtr_t limb = fullBody->GetLimbs().at(effector);
        const fcl::Vec3f& ppos  = currentState.contactPositions_.at(effector);
        const pinocchio::Frame effectorFrame = device->getFrameByName(limb->effector_.name());
        pinocchio::JointPtr_t effectorJoint = effectorFrame.joint();

        std::vector<bool> mask; mask.push_back(true); mask.push_back(true); mask.push_back(true);
        pinocchio::Transform3f localFrame(1), globalFrame(1);
        globalFrame.translation(ppos);
        proj->add(constraints::Implicit::create( constraints::Position::create(effector,device,
                                             effectorJoint,
                                             effectorFrame.pinocchio().placement * localFrame,
                                             globalFrame,
                                             mask)));

        /*proj->add(constraints::Implicit::create (
                                constraints:::Position::create("",device,
                                                              effectorJoint,fcl::Vec3f(0,0,0), ppos)));*/
        if(limb->contactType_ == hpp::rbprm::_6_DOF)
        {

            pinocchio::Transform3f rotation(1);
            rotation.rotation(currentState.contactRotation_.at(effector) * effectorFrame.pinocchio().placement.rotation().transpose() );
            proj->add(constraints::Implicit::create ( constraints::Orientation::create("", device,
                                                                                           effectorJoint,
                                                                                           rotation,
                                                                                           cosntraintsR)));

            //const fcl::Matrix3f& rotation = currentState.contactRotation_.at(effector);
            /*proj->add(constraints::Implicit::create (constraints::deprecated::Orientation::create("", device,
                                                                              effectorJoint,
                                                                              rotation,
                                                                              cosntraintsR)));*/
        }
    }
}

void CreateRootPosConstraint(hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const fcl::Vec3f& target, core::ConfigProjectorPtr_t proj)
{
    pinocchio::Transform3f position(1);
    position.translation(target);
    proj->add(constraints::Implicit::create (
                            constraints::Position::create("",fullBody->device_,
                                                          fullBody->device_->rootJoint(),pinocchio::Transform3f(1), position)));
}

typedef constraints::PointCom PointCom;
typedef constraints::CalculusBaseAbstract<PointCom::ValueType_t, PointCom::JacobianType_t> s_t;
typedef constraints::SymbolicFunction<s_t> PointComFunction;
typedef constraints::SymbolicFunction<s_t>::Ptr_t PointComFunctionPtr_t;

void CreateComPosConstraint(hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const fcl::Vec3f& target, core::ConfigProjectorPtr_t proj)
{
    pinocchio::DevicePtr_t device = fullBody->device_;
    pinocchio::CenterOfMassComputationPtr_t comComp = pinocchio::CenterOfMassComputation::create (device);
    comComp->add (device->rootJoint());
    comComp->compute ();
    PointComFunctionPtr_t comFunc = PointComFunction::create ("COM-constraint",
        device, PointCom::create (comComp));
    constraints::ComparisonTypes_t equals (3, constraints::Equality);
    constraints::ImplicitPtr_t comEq = constraints::Implicit::create(comFunc, equals);
    proj->add(comEq);
    proj->rightHandSide(comEq,target);
}

void CreatePosturalTaskConstraint(hpp::rbprm::RbPrmFullBodyPtr_t fullBody,core::ConfigProjectorPtr_t proj){
  //hppDout(notice,"create postural task, in projection.cc, ref config = "<<pinocchio::displayConfig(fullBody->referenceConfig()));
  std::vector <bool> mask (fullBody->device_->numberDof(),false);
  // mask : 0 for the freeflyer and the extraDoFs :
  for(size_t i = 0 ; i < 3 ; i++){
    mask[i]=false;
  }
  for(size_type i = fullBody->device_->numberDof()-7 ; i < fullBody->device_->numberDof() ; i++){
    mask[i]=false;
  }

  /*for(size_t i = 3 ; i <= 9 ; i++){
    mask[i] = true;
  }*/
 // mask[5] = false; // z root rotation ????

  Configuration_t weight(fullBody->device_->numberDof());
  if(fullBody->postureWeights().size() == fullBody->device_->numberDof()){
    weight = fullBody->postureWeights();
  }else{
    for(size_type i = 0 ; i < fullBody->device_->numberDof() ; ++i)
      weight[i] = 1.;
  }


  // normalize weight array :
  /*value_type moy =0;
  int num_active = 0;
  for (size_type i = 0 ; i < weight.size() ; i++){
    if(mask[i]){
      moy += weight[i];
      num_active++;
    }
  }
  moy = moy/num_active;
  for (size_type i = 0 ; i < weight.size() ; i++)
    weight[i] = weight[i]/moy;

  */

  std::ostringstream oss;
  for (size_type i=0; i < weight.size (); ++i){
    oss << weight [i] << ",";
  }
  //hppDout(notice,"weight postural task = "<<oss.str());

  //constraints::ConfigurationConstraintPtr_t postFunc = constraints::ConfigurationConstraint::create("Postural_Task",fullBody->device_,fullBody->referenceConfig(),weight,mask);
  constraints::ConfigurationConstraintPtr_t postFunc = constraints::ConfigurationConstraint::create("Postural_Task",fullBody->device_,fullBody->referenceConfig(),weight);
  ComparisonTypes_t comps; comps.push_back(constraints::Equality);
  const constraints::ImplicitPtr_t posturalTask = constraints::Implicit::create (postFunc, comps);
  proj->add(posturalTask,segments_t (0),1);
  //proj->updateRightHandSide();
}

ProjectionReport projectToRootPosition(hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const fcl::Vec3f& target,
                                           const hpp::rbprm::State& currentState)
{
    ProjectionReport res;
    core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(fullBody->device_,"proj", 1e-4, 100);
    CreateContactConstraints(fullBody, currentState, proj);
    CreateRootPosConstraint(fullBody, target, proj);
    pinocchio::Configuration_t configuration = currentState.configuration_;
    res.success_ = proj->apply(configuration);
    res.result_ = currentState;
    res.result_.configuration_ = configuration;
    return res;
}

typedef std::vector<pinocchio::JointPtr_t> T_Joint;

T_Joint getJointsFromLimbs(const rbprm::T_Limb limbs)
{
    T_Joint res;
    for(rbprm::CIT_Limb cit = limbs.begin(); cit != limbs.end(); ++cit)
        res.push_back(cit->second->limb_);
    return res;
}

bool not_a_limb(pinocchio::JointPtr_t cJoint, T_Joint limbs)
{
    for(T_Joint::const_iterator cit = limbs.begin(); cit != limbs.end(); ++cit)
    {
        if(cJoint->name() == (*cit)->name()) return false;
    }
    return true;
}

void LockFromRootRec(pinocchio::JointPtr_t cJoint, const std::vector<pinocchio::JointPtr_t>& jointLimbs, pinocchio::ConfigurationIn_t targetRootConfiguration, core::ConfigProjectorPtr_t& projector)
{
    if(not_a_limb(cJoint, jointLimbs))
    {
        core::size_type rankInConfiguration = (cJoint->rankInConfiguration ());
        projector->add(core::LockedJoint::create(cJoint, LiegroupElement(targetRootConfiguration.segment(rankInConfiguration, cJoint->configSize()),cJoint->configurationSpace())));
        //if (cJoint->numberChildJoints() !=1)
        //    return;
        for(std::size_t i =0; i< cJoint->numberChildJoints(); ++i)
            LockFromRootRec(cJoint->childJoint(i), jointLimbs, targetRootConfiguration, projector);
    }
}

void LockFromRoot(hpp::pinocchio::DevicePtr_t device, const rbprm::T_Limb& limbs, pinocchio::ConfigurationIn_t targetRootConfiguration, core::ConfigProjectorPtr_t& projector)
{
    std::vector<pinocchio::JointPtr_t> jointLimbs = getJointsFromLimbs(limbs);
    pinocchio::JointPtr_t cJoint = device->rootJoint();
    LockFromRootRec(cJoint, jointLimbs, targetRootConfiguration, projector);
}


ProjectionReport projectToRootConfiguration(hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const pinocchio::ConfigurationIn_t conf,
                                           const hpp::rbprm::State& currentState, const Vector3 offset)
{
    ProjectionReport res;
    core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(fullBody->device_,"proj", 1e-4, 100);
    CreateContactConstraints(fullBody, currentState, proj);
    if (offset == Vector3::Zero())
        LockFromRoot(fullBody->device_, fullBody->GetLimbs(), conf, proj);
    else
    {
        const std::string rootJointName("root_joint");
        const fcl::Vec3f ppos  = conf.head<3>();
        const pinocchio::Frame effectorFrame = fullBody->device_->getFrameByName(rootJointName);
        pinocchio::JointPtr_t effectorJoint = effectorFrame.joint();

        std::vector<bool> mask; mask.push_back(true); mask.push_back(true); mask.push_back(true);
        pinocchio::Transform3f localFrame(1), globalFrame(1);
        localFrame.translation(offset);
        globalFrame.translation(ppos);
        proj->add(constraints::Implicit::create( constraints::Position::create(rootJointName,fullBody->device_,
                                             effectorJoint,
                                             effectorFrame.pinocchio().placement * localFrame,
                                             globalFrame,
                                             mask)));

    }
    pinocchio::Configuration_t configuration = currentState.configuration_;
    res.success_ = proj->apply(configuration);
    res.result_ = currentState;
    res.result_.configuration_ = configuration;
    return res;
}

ProjectionReport setCollisionFree(hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const core::CollisionValidationPtr_t& validation ,
                                  const std::string& limbName,const hpp::rbprm::State& currentState,bool sort)
{
    ProjectionReport res;
    res.result_ = currentState;
    pinocchio::Configuration_t configuration = currentState.configuration_;
    hpp::core::ValidationReportPtr_t valRep (new hpp::core::CollisionValidationReport);
    if(validation->validate(configuration, valRep) )
    {
        res.result_.configuration_ = configuration;
        res.success_ = true;
        hppDout(notice,"Found collision free conf : current configuration was already valid !");
        return res;
    }

    RbPrmLimbPtr_t limb = fullBody->GetLimb(limbName);
    sampling::T_Sample samples(limb->sampleContainer_.samples_);
    if(sort){
        sampling::sample_greater sortAlgo;
        std::sort(samples.begin(),samples.end(),sortAlgo);
    }
    for(sampling::SampleVector_t::const_iterator cit = samples.begin();cit != samples.end(); ++cit)
    {
        sampling::Load(*cit, configuration);
        hppDout(notice,"Set collision free : static value = "<<cit->staticValue_);
        if(validation->validate(configuration, valRep) )
        {
            res.result_.configuration_ = configuration;
            res.success_ = true;
            hppDout(notice,"Found collision free conf !");
            return res;
        }
    }
    return res;
}

std::vector<bool> setRotationConstraints()
{
    std::vector<bool> res;
    for(std::size_t i =0; i <3; ++i)
    {
        res.push_back(true);
    }
    return res;
}

std::vector<bool> setTranslationConstraints()
{
    std::vector<bool> res;
    for(std::size_t i =0; i <3; ++i)
    {
        res.push_back(true);
    }
    return res;
}
ProjectionReport projectEffector(hpp::core::ConfigProjectorPtr_t proj, const hpp::rbprm::RbPrmFullBodyPtr_t& body, const std::string& limbId, const hpp::rbprm::RbPrmLimbPtr_t& limb,
                          core::CollisionValidationPtr_t validation, pinocchio::ConfigurationOut_t configuration,
                          const fcl::Matrix3f& rotationTarget, std::vector<bool> rotationFilter, const fcl::Vec3f& positionTarget, const fcl::Vec3f& normal,
                          const hpp::rbprm::State& current)
{
    //hppDout(notice,"Project effector : ");
    ProjectionReport rep;
    // Add constraints to resolve Ik
    rep.success_ = false;
    rep.result_ = current;
    // Add constraints to resolve Ik

    if(body->usePosturalTaskContactCreation())
      rotationFilter[2] = false;

    const pinocchio::Frame effectorFrame = body->device_->getFrameByName(limb->effector_.name());
    pinocchio::JointPtr_t effectorJoint = effectorFrame.joint();
    Transform3f localFrame(1), globalFrame(1);
    globalFrame.translation(positionTarget);
    proj->add(constraints::Implicit::create (constraints::Position::create("",body->device_,
                                                                               effectorJoint,
                                                                               effectorFrame.pinocchio().placement * localFrame,
                                                                               globalFrame,
                                                                               setTranslationConstraints())));
    if(limb->contactType_ == hpp::rbprm::_6_DOF)
    {
        Transform3f rotation(1);
        //rotation.rotation(rotationTarget);
        rotation.rotation(rotationTarget * effectorFrame.pinocchio().placement.rotation().transpose());
        proj->add(constraints::Implicit::create (constraints::Orientation::create("",body->device_,
                                                                                      effectorJoint,
                                                                                      rotation,
                                                                                      rotationFilter)));
    }

    if(body->usePosturalTaskContactCreation()){
      CreatePosturalTaskConstraint(body,proj);
      proj->errorThreshold(1e-3);
      proj->maxIterations(1000);
      proj->lastIsOptional(true);
    }


#ifdef PROFILE
    RbPrmProfiler& watch = getRbPrmProfiler();
    watch.start("ik");
#endif
    if(proj->apply(configuration))
    {
        hppDout(notice,"apply contact constraints");
#ifdef PROFILE
        watch.stop("ik");
#endif
#ifdef PROFILE
        RbPrmProfiler& watch = getRbPrmProfiler();
        watch.start("collision");
#endif
        hpp::core::ValidationReportPtr_t valRep (new hpp::core::CollisionValidationReport);
        if(validation->validate(configuration, valRep))
    {
        hppDout(notice,"No collision !");
#ifdef PROFILE
        watch.stop("collision");
#endif
        hppDout(notice,"Projection successfull, add new contact info :");
        body->device_->currentConfiguration(configuration);
        body->device_->computeForwardKinematics();
        State tmp (current);
        tmp.contacts_[limbId] = true;
        tmp.contactPositions_[limbId] = limb->effector_.currentTransformation().translation();
        tmp.contactRotation_[limbId] = limb->effector_.currentTransformation().rotation();
        tmp.contactNormals_[limbId] = normal;
        tmp.contactOrder_.push(limbId);
        tmp.configuration_ = configuration;
        ++tmp.nbContacts;
        rep.success_ = true;
        rep.result_ = tmp;
        }else
        {
            #ifdef PROFILE
            watch.stop("collision");
            #endif
            hppDout(notice,"state in collison after projection, report : "<<*valRep);
            hppDout(notice,"state in collision : r(["<<pinocchio::displayConfig(configuration)<<"])");
        }
    }else{
        #ifdef PROFILE
        watch.stop("ik");
        #endif
        hppDout(notice,"unable to apply contact constraints");
    }
    return rep;
}

fcl::Transform3f computeProjectionMatrix(const hpp::rbprm::RbPrmFullBodyPtr_t& body, const hpp::rbprm::RbPrmLimbPtr_t& limb, const pinocchio::ConfigurationIn_t configuration,
                                         const fcl::Vec3f& normal, const fcl::Vec3f& position)
{
   // hppDout(notice,"computeProjection matrice : normal = "<<normal.transpose());
    body->device_->currentConfiguration(configuration);
    body->device_->computeForwardKinematics();
    // the normal is given by the normal of the contacted object
    //hppDout(notice,"effector rot : \n"<<limb->effector_.currentTransformation().rotation());
    //hppDout(notice,"limb normal : "<<limb->normal_.transpose());
    const fcl::Vec3f z = limb->effector_.currentTransformation().rotation() * limb->normal_;
   // hppDout(notice,"z = "<<z.transpose());
    const fcl::Matrix3f alignRotation = tools::GetRotationMatrix(z,normal);
    //hppDout(notice,"alignRotation : \n"<<alignRotation);
    const fcl::Matrix3f rotation = alignRotation * limb->effector_.currentTransformation().rotation();
    //hppDout(notice,"rotation : \n"<<rotation);
    fcl::Vec3f posOffset = position - rotation * limb->offset_;
    posOffset = posOffset + normal * epsilon;
    return fcl::Transform3f(rotation,posOffset);
}

ProjectionReport projectToObstacle(core::ConfigProjectorPtr_t proj, const hpp::rbprm::RbPrmFullBodyPtr_t& body,const std::string& limbId, const hpp::rbprm::RbPrmLimbPtr_t& limb,
                                   core::CollisionValidationPtr_t validation, pinocchio::ConfigurationOut_t configuration, const hpp::rbprm::State& current,
                                   const fcl::Vec3f& normal, const fcl::Vec3f& position)
{
    fcl::Transform3f pM = computeProjectionMatrix(body, limb, configuration, normal, position);
    return projectEffector(proj, body, limbId, limb, validation, configuration, pM.getRotation(), setRotationConstraints(),pM.getTranslation(), normal, current);
}

// are p1 and p2 on the same side of the line AB ?
bool SameSide(const fcl::Vec3f& p1, const fcl::Vec3f& p2, const fcl::Vec3f& a, const fcl::Vec3f& b)
{
    fcl::Vec3f cp1 = (b-a).cross(p1-a);
    fcl::Vec3f cp2 = (b-a).cross(p2-a);
    return cp1.dot(cp2)>=0;
}

// is p inside ABC ?
bool PointInTriangle(const fcl::Vec3f& p, const fcl::Vec3f& a, const fcl::Vec3f& b, const fcl::Vec3f& c)
{
    return SameSide(p,a, b,c) && SameSide(p,b, a,c) && SameSide(p,c, a,b);
}

double clamp( const double& val, const double& lo, const double& hi)
{
    return std::min( std::max( val, lo ), hi );
}

fcl::Vec3f closestPointInTriangle(const fcl::Vec3f& sourcePosition, const fcl::Vec3f& t0, const fcl::Vec3f& t1, const fcl::Vec3f& t2, const double epsilon =0. )
{
    const fcl::Vec3f edge0 = t1 - t0;
    const fcl::Vec3f edge1 = t2 - t0;
    const fcl::Vec3f v0 = t0 - sourcePosition;

    double a = edge0.dot( edge0 );
    double b = edge0.dot( edge1 );
    double c = edge1.dot( edge1 );
    double d = edge0.dot( v0 );
    double e = edge1.dot( v0 );

    double det = a*c - b*b;
    double s = b*e - c*d;
    double t = b*d - a*e;

    if ( s + t < det )
    {
        if ( s < 0.f )
        {
            if ( t < 0.f )
            {
                if ( d < 0.f )
                {
                    s = clamp( -d/a, 0.f, 1.f );
                    t = 0.f;
                }
                else
                {
                    s = 0.f;
                    t = clamp( -e/c, 0.f, 1.f );
                }
            }
            else
            {
                s = 0.f;
                t = clamp( -e/c, 0.f, 1.f );
            }
        }
        else if ( t < 0.f )
        {
            s = clamp( -d/a, 0.f, 1.f );
            t = 0.f;
        }
        else
        {
            double invDet = 1.f / det;
            s *= invDet;
            t *= invDet;
        }
    }
    else
    {
        if ( s < 0.f )
        {
            double tmp0 = b+d;
            double tmp1 = c+e;
            if ( tmp1 > tmp0 )
            {
                double numer = tmp1 - tmp0;
                double denom = a-2*b+c;
                s = clamp( numer/denom, 0.f, 1.f );
                t = 1-s;
            }
            else
            {
                t = clamp( -e/c, 0.f, 1.f );
                s = 0.f;
            }
        }
        else if ( t < 0.f )
        {
            if ( a+d > b+e )
            {
                double numer = c+e-b-d;
                double denom = a-2*b+c;
                s = clamp( numer/denom, 0.f, 1.f );
                t = 1-s;
            }
            else
            {
                s = clamp( -e/c, 0.f, 1.f );
                t = 0.f;
            }
        }
        else
        {
            double numer = c+e-b-d;
            double denom = a-2*b+c;
            s = clamp( numer/denom, 0.f, 1.f );
            t = 1.f - s;
        }
    }

    fcl::Vec3f res = t0 + s * edge0 + t * edge1;
    return res + (res - sourcePosition).normalized() * epsilon;
}

ProjectionReport projectSampleToObstacle(const hpp::rbprm::RbPrmFullBodyPtr_t& body,const std::string& limbId, const hpp::rbprm::RbPrmLimbPtr_t& limb,
                                         const sampling::OctreeReport& report, core::CollisionValidationPtr_t validation,
                                         pinocchio::ConfigurationOut_t configuration, const hpp::rbprm::State& current)
{
    sampling::Load(*report.sample_, configuration);
    fcl::Vec3f normal = report.normal_;
    normal.normalize();
   // hppDout(notice,"contact normal = "<<normal);
    Transform3f rootT;
    if (body->GetLimb(limbId)->limb_->parentJoint())
        rootT = body->GetLimb(limbId)->limb_->parentJoint()->currentTransformation();
    else
        rootT = body->GetLimb(limbId)->limb_->currentTransformation();
    //compute the orthogonal projection of the end effector on the plan :
    const fcl::Vec3f pEndEff = (rootT.act(report.sample_->effectorPosition_)); // compute absolute position (in world frame)
    fcl::Vec3f pos = pEndEff-(normal.dot(pEndEff-report.v1_))*normal; // orthogonal projection on the obstacle surface
    // make sure contact pos is actually on triangle, and take 1 cm margin ...
    //hppDout(notice,"projectSampleToObstacle,                              pos = "<<pos.transpose());
    pos = closestPointInTriangle(pEndEff,report.v1_, report.v2_, report.v3_, 0.01);
    //hppDout(notice,"projectSampleToObstacle, pos after projection in triangle = "<<pos.transpose());
    //hppDout(notice,"Effector position : "<<report.sample_->effectorPosition_);
    //hppDout(notice,"pEndEff = ["<<pEndEff[0]<<","<<pEndEff[1]<<","<<pEndEff[2]<<"]");
    //hppDout(notice,"pos = ["<<pos[0]<<","<<pos[1]<<","<<pos[2]<<"]");
    core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(body->device_,"proj", 1e-4, 100);
    hpp::tools::LockJointRec(limb->limb_->name(), body->device_->rootJoint(), proj);
    return projectToObstacle(proj, body, limbId, limb, validation, configuration, current, normal, pos);
}

ProjectionReport projectStateToObstacle(const hpp::rbprm::RbPrmFullBodyPtr_t& body, const std::string& limbId, const hpp::rbprm::RbPrmLimbPtr_t& limb,
                                        const hpp::rbprm::State& current, const fcl::Vec3f &normal, const fcl::Vec3f &position, bool lockOtherJoints)
{
   // core::CollisionValidationPtr_t dummy = core::CollisionValidation::create(body->device_);
    return projectStateToObstacle(body, limbId, limb, current, normal, position, body->GetCollisionValidation(),lockOtherJoints);
}

ProjectionReport projectStateToObstacle(const hpp::rbprm::RbPrmFullBodyPtr_t& body, const std::string& limbId, const hpp::rbprm::RbPrmLimbPtr_t& limb,
                                        const hpp::rbprm::State& current, const fcl::Vec3f &normal, const fcl::Vec3f &position, core::CollisionValidationPtr_t validation, bool lockOtherJoints)
{
    hpp::rbprm::State state = current;
    state.RemoveContact(limbId);
    pinocchio::Configuration_t configuration = current.configuration_;
    core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(body->device_,"proj", 1e-4, 1000);
    interpolation::addContactConstraints(body, body->device_,proj, state, state.fixedContacts(state));
    if(lockOtherJoints){ // lock all joints expect the ones of the moving limb
        hpp::tools::LockJointRec(limb->limb_->name(), body->device_->rootJoint(), proj);
    }
    // get current normal orientation
    return projectToObstacle(proj, body, limbId, limb, validation, configuration, state, normal, position);
}


ProjectionReport projectToComPosition(hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const fcl::Vec3f& target,
                                           const hpp::rbprm::State& currentState)
{
    ProjectionReport res;
    core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(fullBody->device_,"proj", 1e-4, 1000);
    CreateContactConstraints(fullBody, currentState, proj);
    CreateComPosConstraint(fullBody, target, proj);
   /* CreatePosturalTaskConstraint(fullBody,proj);
    proj->lastIsOptional(true);
    proj->numOptimize(500);
    proj->lastAsCost(false);
    proj->errorThreshold(1e-3);*/

    pinocchio::Configuration_t configuration = currentState.configuration_;
    res.success_ = proj->apply(configuration);
    res.result_ = currentState;
    res.result_.configuration_ = configuration;
    return res;
}

std::vector<std::string> extractEffectorsName(const rbprm::T_Limb& limbs)
{
    std::vector<std::string> res;
    for(rbprm::T_Limb::const_iterator cit = limbs.begin(); cit != limbs.end(); ++cit)
        res.push_back(cit->first);
    return res;
}

ProjectionReport projectToColFreeComPosition(hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const fcl::Vec3f& target,
                                           const hpp::rbprm::State& currentState)
{
    ProjectionReport res, tmp;
    core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(fullBody->device_,"proj", 1e-3, 1000);
    CreateContactConstraints(fullBody, currentState, proj);
    CreateComPosConstraint(fullBody, target, proj);
    CreatePosturalTaskConstraint(fullBody,proj);
    proj->lastIsOptional(true);
    //proj->numOptimize(500);
    proj->maxIterations(500);
    //proj->lastAsCost(true);
    proj->errorThreshold(1e-2);

    pinocchio::Configuration_t configuration = currentState.configuration_;
    res.success_ = proj->apply(configuration);
    res.result_ = currentState;
    res.result_.configuration_ = configuration;
    hppDout(notice,"Project to col free, first projection done : "<<res.success_);
    hppDout(notice,"projected state : "<<pinocchio::displayConfig(configuration));
    if(res.success_)
    {
        std::vector<std::string> effNames(extractEffectorsName(fullBody->GetLimbs()));
        std::vector<std::string> freeLimbs = rbprm::freeEffectors(currentState,effNames.begin(), effNames.end() );
        for(std::vector<std::string>::const_iterator cit = freeLimbs.begin(); cit != freeLimbs.end() && res.success_; ++cit)
        {
            hppDout(notice,"free effector in projection : "<<*cit);
            tmp = projection::setCollisionFree(fullBody,fullBody->GetLimbCollisionValidation().at(*cit),*cit,res.result_);
            if(!tmp.success_)
                res.success_ = false;
        }
    }
    hppDout(notice,"project to col free, set coll free success = "<<res.success_);
    if(res.success_)
    {
        res.success_ = proj->apply(configuration);
        res.result_.configuration_ = configuration;
        if(res.success_)
        {
            ValidationReportPtr_t report (ValidationReportPtr_t(new CollisionValidationReport));
            res.success_ = fullBody->GetCollisionValidation()->validate(configuration,report);
            hppDout(notice,"project to col free, collision test : "<<res.success_);
            if(!res.success_){
              CollisionValidationReportPtr_t repCast = boost::dynamic_pointer_cast<CollisionValidationReport>(report);
              hppDout(notice,"collision between "<<repCast->object1->name()<<" and "<<repCast->object2->name());
            }
        }
    }
    return res;
}


} // namespace projection
} // namespace rbprm
} // namespace hpp
