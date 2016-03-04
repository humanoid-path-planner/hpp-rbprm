
#ifndef _FILE_ALGORITHMS
#define _FILE_ALGORITHMS


#include <iostream>
#include <Eigen/Dense>
#include <Eigen/src/Core/util/Macros.h>
#include <vector>

namespace geom
{

    /// Implementation of the gift wrapping algorithm to determine the 2D projection of the convex hull of a set of points
    /// Dimension can be greater than two, in which case the points will be projected on the z = 0 plane
    /// and whether a point belongs to it or not.
    ///
    /// \param pointsBegin, pointsEnd iterators to first and last points of a set
    /// \return clockwise traversal of the 2D convex hull of the points
    /// ATTENTION: first point is included twice in representation (it is also the last point)
    template<typename T, int Dim=3, typename Numeric=double, typename Point=Eigen::Matrix<Numeric, Dim, 1>,
             typename CPointRef= const Eigen::Ref<const Point>&, typename In>
    T convexHull(In pointsBegin, In pointsEnd);

    /// Test whether a 2d point belongs to a 2d convex hull
    /// source http://softsurfer.com/Archive/algorithm_0103/algorithm_0103.htm#wn_PinPolygon()
    ///
    /// \param pointsBegin, pointsEnd iterators to first and last points
    /// of the convex hull.ATTENTION: first point is included twice in
    /// representation (it is also the last point).
    /// \param aPoint The point for which to test belonging the the convex hull
    /// \return whether aPoint belongs to the convex polygon determined as the convex hull of the rectangle indicated
    template<int Dim=3, typename Numeric=double, typename Point=Eigen::Matrix<Numeric, Dim, 1>,
             typename Point2=Eigen::Matrix<Numeric, 2, 1>,
             typename CPointRef= const Eigen::Ref<const Point>&, typename In>
    bool containsHull(In pointsBegin, In pointsEnd, CPointRef aPoint, const Numeric Epsilon = 10e-6);

    /// Test whether a 2d point belongs to a 2d convex hull determined by a list of unordered points
    ///
    /// \param pointsBegin, pointsEnd iterators to first and last points for which to consider the convex hull
    /// \param aPoint The point for which to test belonging the the convex hull
    /// \return whether aPoint belongs to the convex polygon determined as the convex hull of the rectangle indicated
    template<typename T, int Dim=3, typename Numeric=double, typename Point=Eigen::Matrix<Numeric, Dim, 1>,
             typename CPointRef= const Eigen::Ref<const Point>&, typename In>
    bool contains(In pointsBegin, In pointsEnd, const CPointRef& aPoint);

    /// Computes whether two convex polygons intersect
    ///
    /// \param aPointsBegin, aPointsEnd iterators to first and last points of the first polygon
    /// \param bPointsBegin, bPointsEnd iterators to first and last points of the second polygon
    /// \return the convex polygon resulting from the intersection
    template<typename T, int Dim=3, typename Numeric=double, typename Point=Eigen::Matrix<Numeric, Dim, 1>,
             typename PointRef= Eigen::Ref<Point>&,
             typename CPointRef= const Eigen::Ref<const Point>&, typename In>
    T computeIntersection(In aPointsBegin, In aPointsEnd, In bPointsBegin, In bPointsEnd);

    /// isLeft(): tests if a point is Left|On|Right of an infinite line.
    /// \param lA 1st point of the line
    /// \param lB 2nd point of the line
    /// \param p2 point to test
    /// \return: >0 for p2 left of the line through p0 and p1
    ///          =0 for p2 on the line
    ///          <0 for p2 right of the line
    /// See: the January 2001 Algorithm "Area of 2D and 3D Triangles and Polygons"
    template<int Dim=3, typename Numeric=double, typename Point=Eigen::Matrix<Numeric, Dim, 1>,
             typename CPointRef= const Eigen::Ref<const Point>& >
    Numeric isLeft(CPointRef lA, CPointRef lB, CPointRef p2);

    /// leftMost(): returns the point most "on the left" for a given set
    /// \param pointsBegin, pointsEnd iterators to first and last points of a set
    template<int Dim=3, typename Numeric=double, typename Point=Eigen::Matrix<Numeric, Dim, 1>, typename In >
    In leftMost(In pointsBegin, In pointsEnd);
} //namespace geom

namespace geom
{
    template<typename CPointRef, typename Numeric=double>
    Numeric dot(CPointRef a, CPointRef b)
    {
        return a[0] * b[0] + a[1] * b[1];
    }

    template<int Dim=3, typename Numeric=double, typename Point=Eigen::Matrix<Numeric, Dim, 1>, typename CPointRef= const Eigen::Ref<const Point>& >
    Numeric isLeft(CPointRef lA, CPointRef lB, CPointRef p2)
    {
        return (lB[0] - lA[0]) * (p2[1] - lA[1]) - (p2[0] - lA[0]) * (lB[1] - lA[1]);
    }


    template<int Dim=3, typename Numeric=double, typename Point=Eigen::Matrix<Numeric, Dim, 1>, typename In >
    In leftMost(In pointsBegin, In pointsEnd)
    {
        In current = pointsBegin +1;In res = pointsBegin;
        while(current!= pointsEnd)
        {
            if(current->operator[](0) < res->operator[](0))
                res = current;
            ++current;
        }
        return res;
    }

    template<typename T, int Dim=3, typename Numeric=double, typename Point=Eigen::Matrix<Numeric, Dim, 1>,
             typename CPointRef= const Eigen::Ref<const Point>&, typename In>
    T convexHull(In pointsBegin, In pointsEnd)
    {
        T res;
        Point pointOnHull = *leftMost(pointsBegin, pointsEnd);
        Point lastPoint = *pointsBegin;
        do {
            lastPoint = *pointsBegin;
            for(In current = pointsBegin +1; current!= pointsEnd; ++current)
            {
                if((lastPoint == pointOnHull) || (isLeft(pointOnHull, lastPoint,*current) > 0))
                    lastPoint = *current;
            }
            res.insert(res.end(),pointOnHull);
            pointOnHull = lastPoint;
        } while(lastPoint != *res.begin());
        res.insert(res.end(), lastPoint);
        return res;
    }

    template<int Dim=3, typename Numeric=double, typename Point=Eigen::Matrix<Numeric, Dim, 1>,
             typename Point2=Eigen::Matrix<Numeric, 2, 1>,
             typename CPointRef= const Eigen::Ref<const Point>&, typename In>
    bool containsHull(In pointsBegin, In pointsEnd, CPointRef aPoint, const Numeric Epsilon = 10e-6)
    {
        int n = (int)(std::distance(pointsBegin, pointsEnd)- 1);
        if(n < 1)
            return false;
        else if(n == 1)
        {
            Numeric x = aPoint[0] - pointsBegin->operator[](0);
            Numeric y = aPoint[1] - pointsBegin->operator[](1);
            return sqrt(x*x + y*y) < Epsilon;
        }
        else if(n == 2)
        {
            Numeric x = pointsEnd->operator[](0) - pointsBegin->operator[](0);
            Numeric y = pointsEnd->operator[](1) - pointsBegin->operator[](1);
            return sqrt(x*x + y*y) < Epsilon;
        }

        // loop through all edges of the polygon
        In current = pointsBegin;
        In next= pointsBegin +1;
        for(;next!=pointsEnd;++current,++next)
        {
            if(isLeft(*current, *next, aPoint) > 0)
                return false;
        }
        return true;
    }

    template<typename T, int Dim=3, typename Numeric=double, typename Point=Eigen::Matrix<Numeric, Dim, 1>,
             typename CPointRef= const Eigen::Ref<const Point>&, typename In>
    bool contains(In pointsBegin, In pointsEnd, const CPointRef& aPoint)
    {
        T ch = convexHull<T, Dim, Numeric, Point, In>(pointsBegin, pointsEnd);
        return contains<Dim, Numeric, Point, In>(ch.begin(), ch.end(), aPoint);
    }

    template<typename T, int Dim=3, typename Numeric=double, typename Point=Eigen::Matrix<Numeric, Dim, 1>,
             typename PointRef= Eigen::Ref<Point>&,
             typename CPointRef= const Eigen::Ref<const Point>&>
    Point lineSect(CPointRef p1, CPointRef p2, CPointRef p3, CPointRef p4)
    {
        Point res;
        Numeric x1 = p1[0], x2 = p2[0], x3 = p3[0], x4 = p4[0];
        Numeric y1 = p1[1], y2 = p2[1], y3 = p3[1], y4 = p4[1];


        Numeric d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        // If d is zero, there is no intersection
        //not supposed to happen
        //if (d == 0) throw;

        // Get the x and y
        Numeric pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
        Numeric x = (pre * (x3 - x4) - (x1 - x2) * post) / d;
        Numeric y = (pre * (y3 - y4) - (y1 - y2) * post) / d;

        // Check if the x and y coordinates are within both lines
        // not supposed to happen
        /*if (x < min(x1, x2) || x > max(x1, x2) ||
        x < min(x3, x4) || x > max(x3, x4)) return NULL;
        if (y < min(y1, y2) || y > max(y1, y2) ||
        y < min(y3, y4) || y > max(y3, y4)) return NULL;*/

        // Return the point of intersection
        res[0] = x;
        res[1] = y;
        return res;
    }

    template<typename T, int Dim=3, typename Numeric=double, typename Point=Eigen::Matrix<Numeric, Dim, 1>,
             typename PointRef= Eigen::Ref<Point>&,
             typename CPointRef= const Eigen::Ref<const Point>&, typename In>
    T computeIntersection(In subBegin, In subEndHull, In clipBegin, In clipEndHull)
    {
        T outputList, inputList;
        In from = subBegin, to = subEndHull;
        for(In edge = clipBegin; edge != clipEndHull-1; ++edge)
        {
            In E = from;
            Numeric dirE, dirS = isLeft<Dim, Numeric, Point, CPointRef>(*edge, *(edge+1), *E);
            for(In S= from+1;
                S != to; ++E, ++S)
            {
                dirE = dirS;
                dirS = isLeft<Dim, Numeric, Point, CPointRef>(*edge, *(edge+1), *S);
                if(dirE < 0)
                {
                    if(dirS < 0)
                        outputList.insert(outputList.end(), *S);
                    else
                        outputList.insert(outputList.end(),
                                          lineSect<T, Dim, Numeric, Point, PointRef, CPointRef>(*S, *E, *edge, *(edge+1)));
                }
                else if(dirS < 0)
                {
                    outputList.insert(outputList.end(),
                                      lineSect<T, Dim, Numeric, Point, PointRef, CPointRef>(*S, *E, *edge, *(edge+1)));
                    outputList.insert(outputList.end(),*S);
                }
            }
            if(outputList.empty())
                return outputList;
            inputList = outputList;
            if(inputList.size()>3)
                inputList.insert(inputList.end(),*(inputList.begin()));
            from = inputList.begin();
            to = inputList.end();
            outputList.clear();
        }
        return inputList;
    }
} //namespace geom

#endif //_FILE_ALGORITHMS
