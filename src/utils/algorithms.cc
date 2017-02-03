#include "utils/algorithms.h"
#include <hpp/fcl/intersect.h>
#include <hpp/util/debug.hh>

namespace geom
{

  /// Computes the normal vector of a triangle based on the
  /// global position of its vertices. The normal is subject to convention!
  /// \param tri The global position of a triangles vertices
  Point TriangleNormal(TrianglePoints& tri)
  {
    Point normal = (tri.p2 - tri.p1).cross(tri.p3 - tri.p1);
    normal.normalize();
    hppDout(notice,"normal, in geom :: "<<normal);
    return normal;
  }
  
  BVHModelOBConst_Ptr_t GetModel(const fcl::CollisionObjectConstPtr_t object)
  {
    assert(object->collisionGeometry()->getNodeType() == fcl::BV_OBBRSS);
    const BVHModelOBConst_Ptr_t model = boost::static_pointer_cast<const BVHModelOB>(object->collisionGeometry());
    assert(model->getModelType() == fcl::BVH_MODEL_TRIANGLES);
    // todo avoid recopy, but if we keep the same ptr the geometry is changed 
    const BVHModelOBConst_Ptr_t modelTransform (new BVHModelOB(*model));
    for(int i = 0 ; i < model->num_vertices ; i++){
      modelTransform->vertices[i] = object->getTransform().transform(model->vertices[i]);
    }
    return modelTransform;
  }
  
  double dot(CPointRef a, CPointRef b)
  {
    return a[0] * b[0] + a[1] * b[1];
  }
  
  
  double isLeft(CPointRef lA, CPointRef lB, CPointRef p2)
  {
    return (lB[0] - lA[0]) * (p2[1] - lA[1]) - (p2[0] - lA[0]) * (lB[1] - lA[1]);
  }
  
  
  CIT_Point leftMost(CIT_Point pointsBegin, CIT_Point pointsEnd)
  {
    CIT_Point current = pointsBegin +1;CIT_Point res = pointsBegin;
    while(current!= pointsEnd)
    {
      if(current->operator[](0) < res->operator[](0))
        res = current;
      ++current;
    }
    return res;
  }
  
  double area(CIT_Point pointsBegin, CIT_Point pointsEnd){
    double a = 0;

    if((pointsBegin == pointsEnd) || ((pointsBegin+1) == pointsEnd))
      return 0;
    
    for( CIT_Point it = pointsBegin + 1 ; it != pointsEnd - 1 ; ++it){
      a += (*it)[0]*( (*(it+1))[1] - (*(it-1))[1] );
    }
    a += (*(pointsEnd-1))[0]*( (*(pointsBegin+1))[1] - (*(pointsEnd-2))[1] );
    a /= 2;
    return fabs(a);
  }
  
  Point center(CIT_Point pointsBegin, CIT_Point pointsEnd){
    double cx=0;
    double cy=0;
    double cz=0;
    size_t i = 0;
    for( CIT_Point it = pointsBegin ; it != pointsEnd - 1 ; ++it){
      cx += (*it)[0];
      cy += (*it)[1];
      cz += (*it)[2];
      i++;
    }
    cx = cx /(double)i;
    cy = cy /(double)i;
    cz = cz /(double)i;
    
    return Point(cx,cy,cz);
  }
  
  
  Point centerPlanar (T_Point points,const fcl::Vec3f& n, double t ){
    
    double cx =0;
    double cy = 0;
    double a = area(points.begin(),points.end());
    for(size_t i = 0 ; i < (points.size() - 1) ; ++i)
    {
      cx += (points[i][0] + points[i+1][0])*((points[i][0] * points[i+1][1]) - (points[i+1][0] * points[i][1])); 
      cy += (points[i][1] + points[i+1][1])*((points[i][0] * points[i+1][1]) - (points[i+1][0] * points[i][1])); 
    }
    
    cx = cx / (6*a);
    cy = cy / (6*a);
    double cz = -(n[0]*cx + n[1]*cy + t) / n[3] ; // deduce z from x,y and the plan equation
    return Point(cx,cy,cz);
  }
  
  
  void projectZ(IT_Point pointsBegin, IT_Point pointsEnd){
    for(IT_Point current = pointsBegin ; current != pointsEnd; ++current){
      (*current)[2]=0;
    }
  }
  
  
  T_Point convexHull(CIT_Point pointsBegin, CIT_Point pointsEnd)
  {
    
    T_Point res;
    Point pointOnHull = *leftMost(pointsBegin, pointsEnd);
    Point lastPoint = *pointsBegin;
    do {
      lastPoint = *pointsBegin;
      for(CIT_Point current = pointsBegin +1; current!= pointsEnd; ++current)
      {
        if((lastPoint == pointOnHull) || (isLeft(pointOnHull, lastPoint,*current) > 0)){
          if( ( std::find(res.begin(),res.end(),*current) == res.end() ) || ((*current) == (*(res.begin()))))// only selected it if not on the list (or the first)
          lastPoint = *current;
        }
      }
      res.insert(res.end(),pointOnHull);
      pointOnHull = lastPoint;
    } while(lastPoint != *res.begin()); // infinite loop with fcl types (instead of eigen)
    res.insert(res.end(), lastPoint);
    return res;
  }
  
  
  
  /*
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
*/
  /*
    template<typename T, int Dim=3, typename Numeric=double, typename Point=Eigen::Matrix<Numeric, Dim, 1>,
             typename CPointRef= const Eigen::Ref<const Point>&, typename In>
    bool contains(In pointsBegin, In pointsEnd, const CPointRef& aPoint)
    {
        T ch = convexHull<T, Dim, Numeric, Point, In>(pointsBegin, pointsEnd);
        return contains<Dim, Numeric, Point, In>(ch.begin(), ch.end(), aPoint);
    }
*/
  
  Point lineSect(CPointRef p1, CPointRef p2, CPointRef p3, CPointRef p4)
  {
    Point res;
    double x1 = p1[0], x2 = p2[0], x3 = p3[0], x4 = p4[0];
    double y1 = p1[1], y2 = p2[1], y3 = p3[1], y4 = p4[1];
    
    
    double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    // If d is zero, there is no intersection
    //not supposed to happen
    //if (d == 0) throw;
    
    // Get the x and y
    double pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
    double x = (pre * (x3 - x4) - (x1 - x2) * post) / d;
    double y = (pre * (y3 - y4) - (y1 - y2) * post) / d;
    
    // Check if the x and y coordinates are within both lines
    // not supposed to happen
    //if (x < min(x1, x2) || x > max(x1, x2) ||
    //x < min(x3, x4) || x > max(x3, x4)) return NULL;
    //if (y < min(y1, y2) || y > max(y1, y2) ||
    //y < min(y3, y4) || y > max(y3, y4)) return NULL;
    
    // Return the point of intersection
    res[0] = x;
    res[1] = y;
    return res;
  }

  Point lineSect3D(CPointRef p1, CPointRef p2, CPointRef p3, CPointRef p4)
  {
    Point res;
    double x1 = p1[0], x2 = p2[0], x3 = p3[0], x4 = p4[0];
    double y1 = p1[1], y2 = p2[1], y3 = p3[1], y4 = p4[1];
    double z1= p1[2];
    Point u = p2-p1; // vector director of the first line

    double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    // If d is zero, there is no intersection
    //not supposed to happen
    //if (d == 0) throw;

    // Get the x and y
    double pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
    double x = (pre * (x3 - x4) - (x1 - x2) * post) / d;
    double y = (pre * (y3 - y4) - (y1 - y2) * post) / d;

    // Check if the x and y coordinates are within both lines
    // not supposed to happen
    //if (x < min(x1, x2) || x > max(x1, x2) ||
    //x < min(x3, x4) || x > max(x3, x4)) return NULL;
    //if (y < min(y1, y2) || y > max(y1, y2) ||
    //y < min(y3, y4) || y > max(y3, y4)) return NULL;

    // Return the point of intersection
    res[0] = x;
    res[1] = y;
    // find the correct z coordinate :
    double t;
    if(u[0] != 0)
      t = (x - x1)/u[0];
    else if(u[1] != 0)
      t = (y - y1)/u[1];
    else{
      hppDout(notice,"in linesect there is no unique z value");
      t=1;
    }

    res[2] = z1 + t*u[2];

    return res;
  }
  
  
  T_Point compute2DIntersection(CIT_Point subBegin, CIT_Point subEndHull, CIT_Point clipBegin, CIT_Point clipEndHull)
  {
    T_Point outputList, inputList;
    CIT_Point from = subBegin, to = subEndHull;
    for(CIT_Point edge = clipBegin; edge != clipEndHull-1; ++edge)
    {
      CIT_Point E = from;
      double dirE, dirS = isLeft(*edge, *(edge+1), *E);
      for(CIT_Point S= from+1; S != to; ++E, ++S)
      {
        dirE = dirS;
        dirS = isLeft(*edge, *(edge+1), *S);
        if(dirE < 0)
        {
          if(dirS < 0)
            outputList.insert(outputList.end(), *S);
          else
            outputList.insert(outputList.end(),lineSect(*S, *E, *edge, *(edge+1)));
        }
        else if(dirS < 0)
        {
          outputList.insert(outputList.end(),lineSect(*S, *E, *edge, *(edge+1)));
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
  
  T_Point compute2DIntersection(T_Point subPolygon, T_Point clipPolygon)
  {
    T_Point outputList, inputList;
    double dirE ,dirS;
    outputList = subPolygon;
    for(CIT_Point edge = clipPolygon.begin() ; edge != clipPolygon.end()-1 ; ++edge){
      inputList = outputList;
      outputList.clear();
      CIT_Point s = inputList.end()-1;
      dirS = isLeft(*edge, *(edge+1),*s);      
      for(CIT_Point e = inputList.begin() ; e != inputList.end() ; ++e){
        dirE = isLeft(*edge, *(edge+1),*e);
        if(dirE <= 0 )// e is inside 
        {
          if(dirS > 0) // s not inside
          {
            outputList.insert(outputList.end(),lineSect(*s, *e, *edge, *(edge+1)));
          }
          outputList.insert(outputList.end(),*e);
        }else if (dirS <= 0) // s is inside
        {
          outputList.insert(outputList.end(),lineSect(*s, *e, *edge, *(edge+1)));
        }
        s=e;
        dirS = dirE;
      }
      
    }
    return outputList;
    
  }
  

  T_Point compute3DIntersection(T_Point subPolygon, T_Point clipPolygon)
  {
    T_Point outputList, inputList;
    double dirE ,dirS;
    outputList = subPolygon;
    for(CIT_Point edge = clipPolygon.begin() ; edge != clipPolygon.end()-1 ; ++edge){
      inputList = outputList;
      outputList.clear();
      CIT_Point s = inputList.end()-1;
      dirS = isLeft(*edge, *(edge+1),*s);
      for(CIT_Point e = inputList.begin() ; e != inputList.end() ; ++e){
        dirE = isLeft(*edge, *(edge+1),*e);
        if(dirE <= 0 )// e is inside
        {
          if(dirS > 0) // s not inside
          {
            outputList.insert(outputList.end(),lineSect3D(*s, *e, *edge, *(edge+1)));
          }
          outputList.insert(outputList.end(),*e);
        }else if (dirS <= 0) // s is inside
        {
          outputList.insert(outputList.end(),lineSect3D(*s, *e, *edge, *(edge+1)));
        }
        s=e;
        dirS = dirE;
      }

    }
    return outputList;

  }

  
  double distanceToPlane(const fcl::Vec3f& n, double t, const fcl::Vec3f& v)
  {
    return n.dot(v) - t;
  }
  
  void computeTrianglePlaneDistance(fcl::Vec3f* tri_point, const fcl::Vec3f& n, double t, fcl::Vec3f* distance, unsigned int* num_penetrating_points)
  {
    *num_penetrating_points = 0;
    
    
    
    for(unsigned int i = 0; i < 3; ++i)
    {
      (*distance)[i] = distanceToPlane(n, t, tri_point[i]);
      if((*distance)[i] < EPSILON) {
        (*num_penetrating_points)++;
      }
    }
  }
  
  bool insideTriangle(const fcl::Vec3f& a, const fcl::Vec3f& b, const fcl::Vec3f& c, const fcl::Vec3f&p)
  {
    fcl::Vec3f ab = b - a;
    fcl::Vec3f ac = c - a;
    fcl::Vec3f n = ab.cross(ac);
    
    fcl::Vec3f pa = a - p;
    fcl::Vec3f pb = b - p;
    fcl::Vec3f pc = c - p;
    
    if((pb.cross(pc)).dot(n) < -EPSILON) return false;
    if((pc.cross(pa)).dot(n) < -EPSILON) return false;
    if((pa.cross(pb)).dot(n) < -EPSILON) return false;
    
    return true;
  }
  
  void intersect3DGeoms(BVHModelOBConst_Ptr_t model1,BVHModelOBConst_Ptr_t model2,fcl::CollisionResult result){
    std::ostringstream ss7;
    ss7<<"[";
    
    
    
    for(size_t c = 0 ; c < result.numContacts() ; ++c){
      int i = result.getContact(c).b1;  // triangle index
      int j = result.getContact(c).b2;
      
      
      fcl::Vec3f tri[3] = {model1->vertices[model1->tri_indices[i][0]],model1->vertices[model1->tri_indices[i][1]],model1->vertices[model1->tri_indices[i][2]]};
      fcl::Vec3f tri2[3] = {model2->vertices[model2->tri_indices[j][0]],model2->vertices[model2->tri_indices[j][1]],model2->vertices[model2->tri_indices[j][2]]}; 
      
      intersectTriangles(tri,tri2,&ss7);
      intersectTriangles(tri2,tri,&ss7);
      
    } // for each contact point
    hppDout(notice,"clipped point : ");        
    ss7<<"]";
    std::cout<<ss7.str()<<std::endl;
  }
  
  T_Point intersectTriangles(fcl::Vec3f* tri, fcl::Vec3f* tri2,std::ostringstream* ss){
    T_Point res;
    fcl::Vec3f n2=0;
    fcl::FCL_REAL t2=0;
    fcl::Intersect::buildTrianglePlane(tri2[0],tri2[1],tri2[2], &n2, &t2);
    fcl::Vec3f distance;
    unsigned int num_penetrating_points=0;
    
    
    geom::computeTrianglePlaneDistance(tri,n2,t2,&distance,&num_penetrating_points);
    /*
    hppDout(notice,"Intersection between triangles : ");
    hppDout(notice,"[["<<tri[0][0]<<","<<tri[0][1]<<","<<tri[0][2]<<"],["<<tri[1][0]<<","<<tri[1][1]<<","<<tri[1][2]<<"],["<<tri[2][0]<<","<<tri[2][1]<<","<<tri[2][2]<<"],["<<tri[0][0]<<","<<tri[0][1]<<","<<tri[0][2]<<"]]");
    hppDout(notice,"[["<<tri2[0][0]<<","<<tri2[0][1]<<","<<tri2[0][2]<<"],["<<tri2[1][0]<<","<<tri2[1][1]<<","<<tri2[1][2]<<"],["<<tri2[2][0]<<","<<tri2[2][1]<<","<<tri2[2][2]<<"],["<<tri2[0][0]<<","<<tri2[0][1]<<","<<tri2[0][2]<<"]]");  
    */
    if(num_penetrating_points > 2 ){
      hppDout(error,"triangle in the wrong side of the plane"); // shouldn't happen
      return res;
    }
    if(num_penetrating_points == 2 )
      distance = - distance ; // like this we always work with the same case for later computation (one point of the triangle inside the plan)
    
    
    
    // distance have one and only one negative distance, we want to separate them
    double dneg=0;
    fcl::Vec3f pneg;
    fcl::Vec3f ppos[2];
    double dpos[2];
    int numPos = 0;
    for(int k = 0 ; k < 3 ; ++k){
      if(distance[k] < 0 ){
        dneg = distance[k];
        pneg = tri[k];
      }else{
        dpos[numPos] = distance[k];
        ppos[numPos] = tri[k];              
        numPos++;
      }
    }
    // TODO case : intersection with vertice : only 1 intersection point
    // compute the first intersection point
    double s1 = dneg / (dneg - dpos[0]);
    fcl::Vec3f i1 = pneg + (ppos[0] - pneg)*s1;
    // compute the second intersection point
    double s2 = dneg / (dneg - dpos[1]);
    fcl::Vec3f i2 = pneg + (ppos[1] - pneg)*s2; 
    if(geom::insideTriangle(tri2[0],tri2[1],tri2[2],i1)){
      res.push_back(Eigen::Vector3d(i1[0],i1[1],i1[2]));
      //hppDout(notice,"first intersection : "<<"["<<i1[0]<<","<<i1[1]<<","<<i1[2]<<"]");
      if(ss)
        *ss<<"["<<i1[0]<<","<<i1[1]<<","<<i1[2]<<"],";                        
    }
    if(geom::insideTriangle(tri2[0],tri2[1],tri2[2],i2)){ 
      res.push_back(Eigen::Vector3d(i2[0],i2[1],i2[2]));      
      //hppDout(notice,"second intersection : "<<"["<<i2[0]<<","<<i2[1]<<","<<i2[2]<<"]");
      if(ss)
        *ss<<"["<<i2[0]<<","<<i2[1]<<","<<i2[2]<<"],";                    
    }
    return res;
  }
  /*
  T_Point intersectPolygonePlane(BVHModelOBConst_Ptr_t polygone, BVHModelOBConst_Ptr_t model2, fcl::Vec3f n , double t, fcl::CollisionResult result, bool useT, double epsilon){
    T_Point res,triRes,sortedRes;
    std::ostringstream ss;
    ss<<"[";
    
    
    for(size_t c = 0 ; c < result.numContacts() ; ++c){
    //  hppDout(info,"normal = "<<result.getContact(c).normal);
      if(result.getContact(c).normal.equal(-n,epsilon)){ // only compute intersection for contact with the plane
        // need the -n because .normal are oriented from o1 to o2
        int i = result.getContact(c).b1;  // triangle index
        int j = result.getContact(c).b2;
        
        
        fcl::Vec3f tri[3] = {polygone->vertices[polygone->tri_indices[i][0]],polygone->vertices[polygone->tri_indices[i][1]],polygone->vertices[polygone->tri_indices[i][2]]};
        fcl::Vec3f tri2[3] = {model2->vertices[model2->tri_indices[j][0]],model2->vertices[model2->tri_indices[j][1]],model2->vertices[model2->tri_indices[j][2]]}; 
        fcl::Vec3f n2=0;
        fcl::FCL_REAL t2=0;
        fcl::Intersect::buildTrianglePlane(tri2[0],tri2[1],tri2[2], &n2, &t2);
      //  hppDout(info,"n = "<<n2);
       // hppDout(info,"t = "<<t2);
        if(n2.equal(n,epsilon) && ((!useT) ||((t2 + EPSILON >= t ) && (t2-EPSILON <= t )))){
          triRes = intersectTriangles(tri,tri2);
          res.insert(res.end(),triRes.begin(),triRes.end());
          triRes = intersectTriangles(tri2,tri);
          res.insert(res.end(),triRes.begin(),triRes.end());

          
        }
      }
      
    } // for each contact point
    if(res.empty()){
      hppDout(notice,"~ Intersection between polygon and plane is empty");
      std::cout<<"~ Intersection between polygon and plane is empty"<<std::endl;
      return res;
    }
    sortedRes = convexHull(res.begin(),res.end());
    hppDout(notice,"clipped point : ");
    for(size_t i = 0; i < sortedRes.size() ; ++i){
      ss<<"["<<sortedRes[i][0]<<","<<sortedRes[i][1]<<","<<sortedRes[i][2]<<"]";
      if(i< (sortedRes.size() -1))
        ss<<",";
    }
    ss<<"]";
    std::cout<<"intersection : "<<std::endl;
    std::cout<<ss.str()<<std::endl;
    hppDout(notice,"area = "<<area(sortedRes.begin(),sortedRes.end()));
    return sortedRes;
  }
  */

  // cf http://geomalgorithms.com/a05-_intersect-1.html
  T_Point intersectSegmentPlane(Point s0, Point s1, Eigen::Vector3d pn, Point p0 ){
    T_Point res;
    Point u = s1 - s0;
    Point w = s0 - p0;

    double d = pn.dot(u);
    double n = - pn.dot(w);

    if(fabs(d) < EPSILON){ // segment parrallel to plane
      if(fabs(n) < EPSILON){ //segment in plane
        res.insert(res.end(),s0);
        res.insert(res.end(),s1);
        return res;
      }else
        return res; // no intersection
    }
    // there ONE intersection point :
    double di = n/d;
    if(di < 0 || di > 1)
      return res; // unknow error ?

    Point si = s0 + di*u;
    res.insert(res.end(),si);
    return res;
  }


  T_Point intersectPolygonePlane(BVHModelOBConst_Ptr_t polygone, BVHModelOBConst_Ptr_t plane, Eigen::Ref<Point> Pn){
    T_Point res, sortedRes;
    T_Point intersection;
    // compute plane equation (normal, point inside the plan)
    Point P0;
    TrianglePoints triPlane;
    triPlane.p1 = plane->vertices[plane->tri_indices[0][0]]; // FIXME : always use the first triangle, is it an issue ?
    triPlane.p2 = plane->vertices[plane->tri_indices[0][1]];
    triPlane.p3 = plane->vertices[plane->tri_indices[0][2]];
    Pn = TriangleNormal(triPlane);
    P0 = triPlane.p1; //FIXME : better point ?

    for(size_t i = 0 ; i < polygone->num_tris ; i++){ // FIXME : can test 2 times the same line (in both triangles), avoid this ?
      //hppDout(info,"triangle : "<<i);
      for(size_t j = 0 ; j < 3 ; j++){
       // hppDout(info,"couple : "<<j);
        intersection = intersectSegmentPlane(polygone->vertices[polygone->tri_indices[i][j]],
                                             polygone->vertices[polygone->tri_indices[i][((j == 2) ? 0 : (j+1))]], Pn,P0);
        if(intersection.size() > 0)
          res.insert(res.end(),intersection.begin(),intersection.end());
      }
    }



    // ordonate the point in the vector (clockwise) first point and last point are the same
    sortedRes = convexHull(res.begin(),res.end());
    hppDout(notice,"clipped point : ");
    std::ostringstream ss;
    ss<<"[";
    for(size_t i = 0; i < sortedRes.size() ; ++i){
      ss<<"["<<sortedRes[i][0]<<","<<sortedRes[i][1]<<","<<sortedRes[i][2]<<"]";
      if(i< (sortedRes.size() -1))
        ss<<",";
    }
    ss<<"]";
   // std::cout<<"intersection : "<<std::endl;
   // std::cout<<ss.str()<<std::endl;
    hppDout(notice,"area = "<<area(sortedRes.begin(),sortedRes.end()));
    return sortedRes;
  }

  T_Point convertBVH(BVHModelOBConst_Ptr_t obj){
    T_Point result;
    for(int i = 0 ; i < obj->num_vertices ; ++i)
    {
      result.push_back(Eigen::Vector3d(obj->vertices[i][0], obj->vertices[i][1], obj->vertices[i][2]));
    }

    return convexHull(result.begin(),result.end());
  }


} //namespace geom

