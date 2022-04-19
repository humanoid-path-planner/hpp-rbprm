
#include "hpp/rbprm/stability/support.hh"

#include <math.h>

using namespace Eigen;

using namespace std;

namespace {
const double Epsilon = 0.001;

typedef std::vector<Vector3d, Eigen::aligned_allocator<Vector3d> > T_Point;
}  // namespace

// isLeft(): tests if a point is Left|On|Right of an infinite line.
//    Input:  three points P0, P1, and P2
//    Return: >0 for P2 left of the line through P0 and P1
//            =0 for P2 on the line
//            <0 for P2 right of the line
//    See: the January 201 Algorithm "Area of 2D and 3D Triangles and Polygons"
double isLeft(const Vector3d& P0, const Vector3d& P1, const Vector3d& P2) {
  return ((P1.x() - P0.x()) * (P2.y() - P0.y()) -
          (P2.x() - P0.x()) * (P1.y() - P0.y()));
}

const Vector3d& LeftMost(const T_Point& points) {
  unsigned int i = 0;
  for (unsigned int j = 1; j < points.size(); ++j) {
    if (points[j].x() < points[i].x()) {
      i = j;
    }
  }
  return points[i];
}

#include <iostream>
// http://en.wikipedia.org/wiki/Gift_wrapping_algorithm
T_Point ConvexHull(const T_Point& points) {
  T_Point res;
  Vector3d pointOnHull = LeftMost(points);
  Vector3d endPoint;
  int i = 0;
  do {
    ++i;
    Vector3d pi = pointOnHull;
    endPoint = points[0];
    for (unsigned int j = 1; j < points.size(); ++j) {
      if ((endPoint == pointOnHull) || (isLeft(pi, endPoint, points[j]) > 0)) {
        endPoint = points[j];
      }
    }
    res.push_back(pi);
    pointOnHull = endPoint;
  } while (endPoint != res[0]);
  res.push_back(endPoint);
  return res;
}

double DistancePointSegment(const Vector3d& pt, const Vector2d& A,
                            const Vector2d& B) {
  Vector2d point(pt.x(), pt.y());
  // first coefficient director
  double a = (A.y() - B.y()) / (A.x() - B.x());
  double b = B.y() - (a * B.x());

  // line that goes through and orthogonal to segment
  double c = -(1 / a);
  double d = point.y() - (point.x() * c);

  // now computing intersection point
  double Xint = (d - b) / (c - a);
  double Yint = a * Xint + b;

  // Xint on the segment if Aint and AB colinear and norm(Aint) < norm(AB)
  Vector2d inter(Xint, Yint);
  if ((inter - A).dot(B - A) > 0 && (inter - A).norm() < (B - A).norm()) {
    Vector2d newPosition(Xint, Yint);
    return (point - newPosition).norm();
  } else  // take closest extremity
  {
    return min((point - A).norm(), (point - B).norm());
  }
}

// source
// http://softsurfer.com/Archive/algorithm_0103/algorithm_0103.htm#wn_PinPolygon()
// wn_PnPoly(): winding double test for a Vector3 in a polygon
//      Input:   P = a Vector3,
//               points_ = Point vector of size n+1 with points_[n]=points_[0]
//      Return:  wn = the winding double (=0 only if P is outside points_[])
bool InPolygon(const T_Point& points, const Vector3d& P) {
  int wn = 0;  // the winding double counter
  int n = (int)(points.size()) - 1;
  if (n < 0) {
    return false;
  } else if (n == 0) {
    Vector2d p(P.x(), P.y());
    Vector2d p0(points[0].x(), points[0].y());
    return ((p - p0).norm() < Epsilon);
  } else if (n == 1) {
    Vector2d p0(points[0].x(), points[0].y());
    Vector2d p1(points[1].x(), points[1].y());
    return (DistancePointSegment(P, p0, p1) < Epsilon);
  }

  // loop through all edges of the polygon
  for (int i = 0; i < n; i++) {       // edge from points_[i] to points_[i+1]
    if (points[i].y() <= P.y()) {     // start y <= P.y()
      if (points[i + 1].y() > P.y())  // an upward crossing
      {
        if (isLeft(points[i], points[i + 1], P) > 0)  // P left of edge
          ++wn;  // have a valid up intersect
      }
    } else {                           // start y > P.y() (no test needed)
      if (points[i + 1].y() <= P.y())  // a downward crossing
      {
        if (isLeft(points[i], points[i + 1], P) < 0)  // P right of edge
          --wn;  // have a valid down intersect
      }
    }
  }
  return !(wn == 0);
}

using namespace hpp::rbprm::stability;

bool hpp::rbprm::stability::Contains(
    const Eigen::Matrix<double, Eigen::Dynamic, 1> support,
    const Eigen::Vector3d& aPoint, const Eigen::VectorXd& xs,
    const Eigen::VectorXd& ys) {
  T_Point points;
  int nbPoints = (int)support.rows() / 3;
  if (nbPoints < 1) return false;
  for (int i = 0; i < nbPoints; ++i) {
    Eigen::Vector3d point = support.segment<3>(i * 3);
    points.push_back(point + Eigen::Vector3d(xs[i], ys[i], 0));
    points.push_back(point + Eigen::Vector3d(-xs[i], ys[i], 0));
    points.push_back(point + Eigen::Vector3d(-xs[i], -ys[i], 0));
    points.push_back(point + Eigen::Vector3d(xs[i], -ys[i], 0));
  }
  points = ConvexHull(points);
  return InPolygon(points, aPoint);
}
