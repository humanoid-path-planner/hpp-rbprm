
#ifndef _CLASS_SUPPORT_POLYGON
#define _CLASS_SUPPORT_POLYGON


#include <Eigen/Dense>
#include <vector>

namespace hpp
{
namespace rbprm
{
namespace stability
{
    bool Contains(const Eigen::Matrix<double, Eigen::Dynamic, 1 > /*support*/, const Eigen::Vector3d& /*aPoint*/
                  , const Eigen::VectorXd& xs, const Eigen::VectorXd& ys);
}
}
}

#endif //_CLASS_SUPPORT_POLYGON
