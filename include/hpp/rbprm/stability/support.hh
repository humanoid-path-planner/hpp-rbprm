
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
    /// Implementation of the gift wrapping algorithm to determine the 2D projection of the convex hull of a 3D set of rectangles
    /// and whether a point belongs to it or not.
    /// \param support a Vector containing all the points at the center of the rectangles used to determine the convex hull
    /// \param aPoint The point for which to test belonging the the convex hull
    /// \param xs Vector of width offsets for the rectangle in support
    /// \param ys Vector of heights offsets for the rectangle in support
    /// \param aPoint The point for which to test belonging the the convex hull
    /// return whether aPoint belongs to the convex polygon determined as the convex hull of the rectangle indicated
    bool Contains(const Eigen::Matrix<double, Eigen::Dynamic, 1 > support, const Eigen::Vector3d& aPoint
                  , const Eigen::VectorXd& xs, const Eigen::VectorXd& ys);
}
}
}

#endif //_CLASS_SUPPORT_POLYGON
