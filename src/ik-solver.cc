
#include "hpp/rbprm/ik-solver.hh"
#include "hpp/model/joint.hh"

#include <vector>
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
using namespace hpp;
using namespace hpp::model;
using namespace hpp::rbprm;

namespace
{
    typedef Eigen::Matrix<double, 6, 1> error_vec_t;
    typedef Eigen::Ref<error_vec_t>         error_vec_ref_t;
    typedef Eigen::Ref<const error_vec_t>   error_vec_cref_t;
    const fcl::Transform3f I4;

    matrix_t ComputeJacobian(RbPrmLimb& limb, const ConfigurationIn_t config)
    {
        limb.limb_->robot()->currentConfiguration (config);
        limb.limb_->computeJacobian();
        return limb.effector_->jacobian().block(0,limb.limb_->rankInVelocity()-1,6, limb.effector_->rankInVelocity() - limb.limb_->rankInVelocity()+1);
    }

    void ComputeJacobian(RbPrmLimb& limb, const ConfigurationIn_t config, matrixOut_t jac)
    {
        limb.limb_->robot()->currentConfiguration (config);
        limb.limb_->computeJacobian();
        jac = limb.effector_->jacobian().block(0,limb.limb_->rankInVelocity()-1,6, limb.effector_->rankInVelocity() - limb.limb_->rankInVelocity()+1);
    }

    void UpdateTransforms(RbPrmLimb& limb)
    {
        limb.limb_->recursiveComputePosition(limb.limb_->robot()->currentConfiguration (), I4);
    }

    void error(RbPrmLimb& limb, const fcl::Vec3f& target, const fcl::Matrix3f& rotationTarget, fcl::Matrix3f& Rerror_, double& tr,
               double& theta, error_vec_ref_t error)
    {
        const Transform3f& M = limb.effector_->currentTransformation ();

        // translation error
        const fcl::Vec3f& p = M.getTranslation();
        for(std::size_t i =0; i <3; ++i)
            error(i) = target[i] - p[i];

        //orientation error
        Rerror_ = M.getRotation(); Rerror_.transpose();
        Rerror_ = Rerror_ * rotationTarget;
        tr = Rerror_ (0, 0) + Rerror_ (1, 1) + Rerror_ (2, 2);
        if (tr > 3) tr = 3;
        if (tr < -1) tr = -1;
        theta = acos ((tr - 1)/2);
        if (theta > 1e-6)
        {
            theta /= (2*sin(theta));
            error[3] = theta*(Rerror_ (2, 1) - Rerror_ (1, 2));
            error[4] = theta*(Rerror_ (0, 2) - Rerror_ (2, 0));
            error[5] = theta*(Rerror_ (1, 0) - Rerror_ (0, 1));
        }
        else
        {
            error[3] = (Rerror_ (2, 1) - Rerror_ (1, 2))/2;
            error[4] = (Rerror_ (0, 2) - Rerror_ (2, 0))/2;
            error[5] = (Rerror_ (1, 0) - Rerror_ (0, 1))/2;
        }
    }

    void error(RbPrmLimb& limb, const fcl::Vec3f& target, ik::Vector3dRef error)
    {
        const Transform3f& M = limb.effector_->currentTransformation ();
        // translation error
        const fcl::Vec3f& p = M.getTranslation();
        for(std::size_t i =0; i <3; ++i)
            error(i) = target[i] - p[i];
    }
}


bool ik::apply(RbPrmLimb& limb, const fcl::Vec3f& target, const fcl::Matrix3f& targetRotation, model::ConfigurationOut_t configuration)
{
    std::size_t start = limb.limb_->rankInConfiguration();
    std::size_t end = limb.effector_->rankInConfiguration()+ limb.effector_->neutralConfiguration().rows();
    std::size_t length = end - start;
    Configuration_t config = limb.limb_->robot()->currentConfiguration().segment(start, length);
    matrix_t jacobian = ComputeJacobian(limb,configuration);
    matrix_t jacobianProd = jacobian.transpose() * jacobian;
    UpdateTransforms(limb);
    //const matrix_t id = Eigen::MatrixXd::Identity(jacobianProd.cols(),jacobianProd.cols());
    fcl::Matrix3f errormatrix;
    double tr, theta;
    const double lambda = 0.01;
    error_vec_t err;
    for(int i = 0; i< 10; ++i)
    {

        error(limb, target, targetRotation,errormatrix,tr,theta,err);
        std::cout << "config " << std::endl << config << std::endl;
        std::cout << "jacobianProd " << std::endl << jacobianProd << std::endl;
        std::cout << "err " << std::endl << err << std::endl;
        std::cout << "solving " << std::endl << jacobianProd.llt().solve(jacobian.transpose()) << std::endl;
        std::cout << "solving " << std::endl << jacobianProd.llt().solve(jacobian.transpose()) << std::endl;
        config += lambda * jacobianProd.llt().solve(jacobian.transpose()) * err;
        configuration.segment(start, length) = config;
        ComputeJacobian(limb, configuration, jacobian);
        jacobianProd = jacobian.transpose() * jacobian;
        UpdateTransforms(limb);
    }
    return err.norm() <= 1e-6;
}
