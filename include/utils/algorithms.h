
#ifndef GEOM__ALGORITHMS
#define GEOM__ALGORITHMS


#include <iostream>
#include <Eigen/Dense>
#include <Eigen/src/Core/util/Macros.h>
#include <vector>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/model/collision-object.hh>


namespace geom
{
  
  typedef Eigen::Vector3d Point;
  typedef std::vector<Point> T_Point;
  typedef T_Point::const_iterator CIT_Point;
  typedef T_Point::iterator IT_Point;  
  typedef const Eigen::Ref<const Point>& CPointRef;
  
  /*
  typedef fcl::Vec3f Point;
  typedef std::vector<Point> T_Point;
  typedef T_Point::const_iterator CIT_Point;
  typedef const Point& CPointRef;
  
*/
  typedef std::vector<Eigen::Vector2d> T_Point2D;
  typedef T_Point2D::const_iterator CIT_Point2D;

  /// Helper struct that saves the global position of the triangle
  /// vertices of a fcl::Triangle.
  struct TrianglePoints
  {
    fcl::Vec3f p1, p2, p3;
  };


  
  const double EPSILON = 1e-5;
  const double ZJUMP = 0.001; // value t for the floor in jump_easy_map
  
  typedef fcl::BVHModel<fcl::OBBRSS> BVHModelOB;
  typedef boost::shared_ptr<const BVHModelOB> BVHModelOBConst_Ptr_t;

  BVHModelOBConst_Ptr_t GetModel(const fcl::CollisionObjectConstPtr_t object);
  
  void projectZ(IT_Point pointsBegin, IT_Point pointsEnd);
  
  /// Implementation of the gift wrapping algorithm to determine the 2D projection of the convex hull of a set of points
  /// Dimension can be greater than two, in which case the points will be projected on the z = 0 plane
  /// and whether a point belongs to it or not.
  ///
  /// \param pointsBegin, pointsEnd iterators to first and last points of a set
  /// \return clockwise traversal of the 2D convex hull of the points
  /// ATTENTION: first point is included twice in representation (it is also the last point)
  
  T_Point convexHull(CIT_Point pointsBegin, CIT_Point pointsEnd);
  
  /// Test whether a 2d point belongs to a 2d convex hull
  /// source http://softsurfer.com/Archive/algorithm_0103/algorithm_0103.htm#wn_PinPolygon()
  ///
  /// \param pointsBegin, pointsEnd iterators to first and last points
  /// of the convex hull.ATTENTION: first point is included twice in
  /// representation (it is also the last point).
  /// \param aPoint The point for which to test belonging the the convex hull
  /// \return whether aPoint belongs to the convex polygon determined as the convex hull of the rectangle indicated
  /* template<int Dim=3, typename Numeric=double, typename Point=Eigen::Matrix<Numeric, Dim, 1>,
             typename Point2=Eigen::Matrix<Numeric, 2, 1>,
             typename CPointRef= const Eigen::Ref<const Point>&, typename In>
    bool containsHull(In pointsBegin, In pointsEnd, CPointRef aPoint, const Numeric Epsilon = 10e-6);
*/
  /// Test whether a 2d point belongs to a 2d convex hull determined by a list of unordered points
  ///
  /// \param pointsBegin, pointsEnd iterators to first and last points for which to consider the convex hull
  /// \param aPoint The point for which to test belonging the the convex hull
  /// \return whether aPoint belongs to the convex polygon determined as the convex hull of the rectangle indicated
  /* template<typename T, int Dim=3, typename Numeric=double, typename Point=Eigen::Matrix<Numeric, Dim, 1>,
             typename CPointRef= const Eigen::Ref<const Point>&, typename In>
    bool contains(In pointsBegin, In pointsEnd, const CPointRef& aPoint);
*/
  /// Computes whether two convex polygons intersect
  ///
  /// \param aPointsBegin, aPointsEnd iterators to first and last points of the first polygon
  /// \param bPointsBegin, bPointsEnd iterators to first and last points of the second polygon
  /// \return the convex polygon resulting from the intersection
  T_Point compute2DIntersection(CIT_Point aPointsBegin, CIT_Point aPointsEnd, CIT_Point bPointsBegin, CIT_Point bPointsEnd);
  
  /// Computes whether two convex polygons intersect
  ///
  /// \param subPolygon list of vertices of the first polygon
  /// \param clipPolygon list of vertices of the second polygon
  /// \return the convex polygon resulting from the intersection
  T_Point compute2DIntersection(T_Point subPolygon, T_Point clipPolygon);
  
  /// Computes whether two convex polygons intersect
  ///
  /// \param subPolygon list of vertices of the first polygon
  /// \param clipPolygon list of vertices of the second polygon
  /// \return the convex polygon resulting from the intersection
  T_Point compute3DIntersection(T_Point subPolygon, T_Point clipPolygon);
  
  /// isLeft(): tests if a point is Left|On|Right of an infinite line.
  /// \param lA 1st point of the line
  /// \param lB 2nd point of the line
  /// \param p2 point to test
  /// \return: >0 for p2 left of the line through p0 and p1
  ///          =0 for p2 on the line
  ///          <0 for p2 right of the line
  /// See: the January 2001 Algorithm "Area of 2D and 3D Triangles and Polygons"
  
  double isLeft(CPointRef lA, CPointRef lB, CPointRef p2);
  
  /// leftMost(): returns the point most "on the left" for a given set
  /// \param pointsBegin, pointsEnd iterators to first and last points of a set
  CIT_Point leftMost(CIT_Point pointsBegin, CIT_Point pointsEnd);
  
  
  /**
   * @brief area compute the area of a 2D polygon
   * @param pointsBegin
   * @param pointsEnd
   * @return 
   */
  double area(CIT_Point pointsBegin, CIT_Point pointsEnd);
  
  
  /**
   * @brief center compute the center of the polygon (planar)
   * @param pointsBegin
   * @param PointsEnd
   * @param n normal of the plan which contain the polygon
   * @param t 
   * @return 
   */
  Point centerPlanar (T_Point points, const fcl::Vec3f &n, double t);
  
  /**
   * @brief center compute the center using average method
   * @param points list of points of the polygon
   * @return the center of the polygon
   */
  Point center(CIT_Point pointsBegin, CIT_Point pointsEnd);

  Point centroid(CIT_Point pointsBegin, CIT_Point pointsEnd,double &area);

  
  /**
   * @brief distanceToPlane Distance point plan
   * @param n plan's normal
   * @param t plan's offset
   * @param v point
   * @return distance
   */
  double distanceToPlane(const fcl::Vec3f& n, double t, const fcl::Vec3f& v);
  
  /**
   * @brief computeTrianglePlaneDistance compute distance between each vertice of the triangle and a plane
   * @param tri_point
   * @param n plan's normal
   * @param t plan's offset
   * @param distance distance for vertice i
   * @param num_penetrating_points number of point with negative distance 
   */
  void computeTrianglePlaneDistance(fcl::Vec3f* tri_point, const fcl::Vec3f& n, double t, fcl::Vec3f *distance, unsigned int* num_penetrating_points);
  
  /**
   * @brief insideTriangle check if a point is inside a triangle
   * @param a 
   * @param b
   * @param c
   * @param p the point
   * @return bool 
   */
  bool insideTriangle(const fcl::Vec3f& a, const fcl::Vec3f& b, const fcl::Vec3f& c, const fcl::Vec3f&p);
  
  /**
   * @brief intersectGeoms compute intersection between 2 OBBRSS geometries
   * @param model1
   * @param model2
   * @param result the collision report between the 2 geometries
   */
  void intersect3DGeoms(BVHModelOBConst_Ptr_t model1,BVHModelOBConst_Ptr_t model2,fcl::CollisionResult result);

  /**
   * @brief intersectPolygonePlane Compute the intersection of a polygon and a plane
   * The returned point belongs to the surfaces of @param model2 corresponding to the plan equation.
   * @param polygone the polygon
   * @param model2 the meshs containing the plane
   * @param n normal of the plan
   * @param t offset of the plan
   * @param useT if false, all the plan with a normal n are used, if true we also check that t is equal
   * @param result
   */
  //T_Point intersectPolygonePlane(BVHModelOBConst_Ptr_t polygone, BVHModelOBConst_Ptr_t model2, fcl::Vec3f n , double t, fcl::CollisionResult result, bool useT = true, double epsilon = EPSILON);



  /**
   * @brief intersectTriangles compute intersection between 2 triangles (not commutative)
   * @param tri first triangle MUST BE DIMENSION 3
   * @param tri2 second triangle (used to compute the plane, the returned point belong to this triangle
   * @param ss 
   */
  T_Point intersectTriangles(fcl::Vec3f* tri, fcl::Vec3f* tri2,std::ostringstream* ss=0);
  
  /**
   * @brief intersectSegmentPlane compute the intersection between a segment and a plane (infinite)
   * @param s0 first point of the segment
   * @param s1 last point of the segment
   * @param n normal of the plan
   * @param p0 a point in the plan
   * @return A vector of point, empty if no intersection, with one point on the plane if there is an intersection, with s0 and s1 if the segment lie in the plan
   */
  T_Point intersectSegmentPlane(Point s0, Point s1, Eigen::Vector3d pn, Point p0 );

  /**
   * @brief intersectPolygonePlane compute the intersection between a polygone and a plane
   * @param polygone
   * @param plane
   * @param pn : output, normal of the plan
   * @return an ordoned list of point (clockwise), which belong to both the polygone and the plane
   */
  T_Point intersectPolygonePlane(BVHModelOBConst_Ptr_t polygone, BVHModelOBConst_Ptr_t plane, Eigen::Ref<Point> Pn);

  T_Point convertBVH(BVHModelOBConst_Ptr_t obj);
  
} //namespace geom

#endif //_FILE_ALGORITHMS
