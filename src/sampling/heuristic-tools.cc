#include <hpp/rbprm/sampling/heuristic-tools.hh>

namespace hpp{
namespace rbprm{
namespace sampling{

HeuristicParam::HeuristicParam(const std::map<std::string, fcl::Vec3f> & cp, const fcl::Vec3f & comPos,
const fcl::Vec3f & comSp, const fcl::Vec3f & comAcc, const std::string & sln, const fcl::Transform3f & tf) : contactPositions_(cp),
                                                                                                             comPosition_(comPos),
                                                                                                             comSpeed_(comSp),
                                                                                                             comAcceleration_(comAcc),
                                                                                                             sampleLimbName_(sln),
                                                                                                             tfWorldRoot_(tf)
{}
HeuristicParam::HeuristicParam(const HeuristicParam & zhp) : contactPositions_(zhp.contactPositions_),
                                                             comPosition_(zhp.comPosition_),
                                                             comSpeed_(zhp.comSpeed_),
                                                             comAcceleration_(zhp.comAcceleration_),
                                                             sampleLimbName_(zhp.sampleLimbName_),
                                                             tfWorldRoot_(zhp.tfWorldRoot_)
{}
HeuristicParam & HeuristicParam::operator=(const HeuristicParam & zhp)
{
    if(this != &zhp)
    {
        this->contactPositions_.clear();
        this->contactPositions_.insert(zhp.contactPositions_.begin(), zhp.contactPositions_.end());
        this->comPosition_ = zhp.comPosition_;
        this->comSpeed_ = zhp.comSpeed_;
        this->comAcceleration_ = zhp.comAcceleration_;
        this->sampleLimbName_ = zhp.sampleLimbName_;
        this->tfWorldRoot_ = zhp.tfWorldRoot_;
    }
    return *this;
}

fcl::Vec3f transform(const fcl::Vec3f & p, const fcl::Vec3f & tr, const fcl::Matrix3f & ro)
{
    fcl::Vec3f res(
        p[0]*ro(0,0) + p[1]*ro(0,1) + p[2]*ro(0,2) + tr[0],
        p[0]*ro(1,0) + p[1]*ro(1,1) + p[2]*ro(1,2) + tr[1],
        p[0]*ro(2,0) + p[1]*ro(2,1) + p[2]*ro(2,2) + tr[2]
    );
    return res;
}

Vec2D & Vec2D::operator=(const Vec2D & c)
{
    if(this != &c)
    {
        this->x = c.x;
        this->y = c.y;
    }
    return *this;
}
double Vec2D::operator[](int idx) const
{
    idx = idx % 2;
    if(idx == 0)
        return this->x;
    else
        return this->y;
}
double & Vec2D::operator[](int idx)
{
    idx = idx % 2;
    if(idx == 0)
        return this->x;
    else
        return this->y;
}
double Vec2D::euclideanDist(const Vec2D & v1, const Vec2D & v2)
{
    return std::sqrt(std::pow(v1.x - v2.x, 2) + std::pow(v1.y - v2.y, 2));
}
bool operator==(const Vec2D & v1, const Vec2D & v2)
{
    //return ((v1.x == v2.x) && (v1.y == v2.y));
    return ((std::abs(v1.x - v2.x) < 1e-9) && (std::abs(v1.y - v2.y) < 1e-9));
}
bool operator!=(const Vec2D & v1, const Vec2D & v2)
{
    return !(v1 == v2);
}
std::ostream & operator<<(std::ostream & out, const Vec2D & v)
{
    out << "(" << v.x << ", " << v.y << ")";
    return out;
}
Plane & Plane::operator=(const Plane & pe)
{
    if(this != &pe)
    {
        this->a = pe.a;
        this->b = pe.b;
        this->c = pe.c;
        this->d = pe.d;
    }
    return *this;
}

fcl::Vec3f orthogonalProjection(const fcl::Vec3f & point, const Plane & plane)
{
    double k(((plane.a * point[0]) + (plane.b * point[1]) + (plane.c * point[2]) + plane.d) / (std::pow(plane.a, 2) + std::pow(plane.b, 2) + std::pow(plane.c, 2)));
    return fcl::Vec3f(point[0] - k*plane.a, point[1] - k*plane.b, point[2] - k*plane.c);
}

std::vector <Vec2D> computeSupportPolygon(const std::map <std::string, fcl::Vec3f> & contactPositions)
{
    Plane h_plane(0, 0, 1, 0); // horizontal plane
    std::vector <Vec2D> res;
    for(std::map<std::string, fcl::Vec3f>::const_iterator cit = contactPositions.begin(); cit != contactPositions.end(); ++cit)
    {
        fcl::Vec3f proj(orthogonalProjection(cit->second, h_plane));
        Vec2D vertex_2D(proj[0], proj[1]);
        res.push_back(vertex_2D);
        //res.push_back(Vec2D(cit->second[0], cit->second[1])); // because the plane is horizontal, we just have to remove the z (vertical) component
    }
    return res;
}

double computeAngle(const Vec2D & center, const Vec2D & end1, const Vec2D & end2)
{
    if(end1 == end2)
        return 0.0;

    // build vector1
    Vec2D vector1(end1.x - center.x, end1.y - center.y);
    double norm1(std::sqrt(std::pow(vector1.x, 2) + std::pow(vector1.y, 2)));

    // build vector2
    Vec2D vector2(end2.x - center.x, end2.y - center.y);
    double norm2(std::sqrt(std::pow(vector2.x, 2) + std::pow(vector2.y, 2)));

    // find the angle between the two vectors using their scalar product
    double sp((vector1.x*vector2.x) + (vector1.y*vector2.y));
    return std::acos(sp/(norm1*norm2));
}

void scanningProcess(const Vec2D & basePoint, std::vector <Vec2D> & subset, double & angle, const Vec2D & currentPoint, bool higher, bool direction)
{
    // higher == true --> currentPoint is above basePoint
    // higher == false --> currentPoint is below basePoint
    // direction == true --> scan to the right
    // direction == false --> scan to the left
    int higher_val = higher ? 1 : -1;
    int direction_val = direction ? 1 : -1;

    if(subset.size() == 1)
    {
        // init
        angle = computeAngle(basePoint, Vec2D(basePoint.x + direction_val, basePoint.y), currentPoint);
        subset.push_back(currentPoint);
    }
    else if((higher_val*currentPoint.y) >= (higher_val*subset.back().y))
    {
        double opening(computeAngle(subset.back(), Vec2D(subset.back().x + direction_val, subset.back().y), currentPoint));
        if(opening <= angle)
        {
            subset.push_back(currentPoint);
            angle = opening;
        }
        else
        {
            subset.pop_back();
            opening = computeAngle(subset.back(), Vec2D(subset.back().x + direction_val, subset.back().y), currentPoint);
            bool convex(false);
            if(subset.size() == 1)
                convex = true;
            while(!convex)
            {
                Vec2D base(subset[subset.size() - 2]);
                angle = computeAngle(base, Vec2D(base.x + direction_val, base.y), subset.back());
                if(angle < opening)
                {
                    subset.pop_back();
                    opening = computeAngle(subset.back(), Vec2D(subset.back().x + direction_val, subset.back().y), currentPoint);
                }
                else
                    convex = true;
                if(subset.size() == 1)
                    convex = true;
            }
            subset.push_back(currentPoint);
            angle = opening;
        }
    }
}

std::vector <Vec2D> convexHull(std::vector <Vec2D> set)
{
    std::vector <Vec2D> res_tmp, res;
    
    if(!set.empty())
    {
        /* sort the input set by x increasing */
        std::vector <Vec2D> sortedSet;
        while(!set.empty())
        {
            unsigned int index(0);
            double min(set[index].x);
            for(unsigned int i = 0; i < set.size(); ++i)
            {
                if(set[i].x < min)
                {
                    min = set[i].x;
                    index = i;
                }
            }
            sortedSet.push_back(set[index]);
            set.erase(set.begin()+index);
        }

        /* first scanning, to the right */
        Vec2D basePoint(sortedSet[0]);
        std::vector <Vec2D> tr_upper_set; tr_upper_set.push_back(basePoint);
        std::vector <Vec2D> tr_lower_set; tr_lower_set.push_back(basePoint);
        double upperAngle, lowerAngle;
        for(unsigned int i = 1; i < sortedSet.size(); ++i)
        {
            if(sortedSet[i].y >= basePoint.y) // if the point is upper than basePoint
            {
                scanningProcess(basePoint, tr_upper_set, upperAngle, sortedSet[i], true, true);
            }
            else // if the point is lower than basePoint
            {
                scanningProcess(basePoint, tr_lower_set, lowerAngle, sortedSet[i], false, true);
            }
        }

        /* second scanning, to the left */
        basePoint = sortedSet.back();
        std::vector <Vec2D> tl_upper_set; tl_upper_set.push_back(basePoint);
        std::vector <Vec2D> tl_lower_set; tl_lower_set.push_back(basePoint);
        for(int i = (int)sortedSet.size() - 2; i >= 0; --i)
        {
            if(sortedSet[i].y >= basePoint.y) // if the point is upper than basePoint
            {
                scanningProcess(basePoint, tl_upper_set, upperAngle, sortedSet[i], true, false);
            }
            else // if the point is lower than basePoint
            {
                scanningProcess(basePoint, tl_lower_set, lowerAngle, sortedSet[i], false, false);
            }
        }

        /* merge the four subsets without keeping the duplicates (subsets boundaries, ...) */
        for(unsigned int i = 0; i < tr_upper_set.size(); ++i)
        {
            res_tmp.push_back(tr_upper_set[i]);
        }
        for(int i = (int)tl_upper_set.size() - 1; i >= 0; --i)
        {
            res_tmp.push_back(tl_upper_set[i]);
        }
        for(unsigned int i = 0; i < tl_lower_set.size(); ++i)
        {
            res_tmp.push_back(tl_lower_set[i]);
        }
        for(int i = (int)tr_lower_set.size() - 1; i >= 0; --i)
        {
            res_tmp.push_back(tr_lower_set[i]);
        }
        for(unsigned int i = 0; i < res_tmp.size(); ++i)
        {
            if(!contains(res, res_tmp[i]))
                res.push_back(res_tmp[i]);
        }
    }

    return res;
}

Vec2D weightedCentroidConvex2D(const std::vector <Vec2D> & convexPolygon)
{
    if(convexPolygon.empty())
        throw std::string("Impossible to find the weighted centroid of nothing (the specified convex polygon has no vertices)");

    Vec2D res;
    if(convexPolygon.size() == 1)
        res = convexPolygon[0];
    else if(convexPolygon.size() == 2)
    {
        double resX((convexPolygon[0].x + convexPolygon[1].x) / 2.0);
        double resY((convexPolygon[0].y + convexPolygon[1].y) / 2.0);
        res = Vec2D(resX, resY);
    }
    else
    {
        // get the longest edge and define the minimum admissible threshold for counting a vertex as a single point
        double maxDist(Vec2D::euclideanDist(convexPolygon.back(), convexPolygon.front()));
        for(unsigned int i = 0; i < convexPolygon.size() - 1; ++i)
        {
            double dist(Vec2D::euclideanDist(convexPolygon[i], convexPolygon[i+1]));
            if(dist > maxDist)
                maxDist = dist;
        }
        double threshold(maxDist/10.0);

        // shift the list until starting from a lonely (to the rear) point
        std::vector <Vec2D> shifted(convexPolygon);
        while(Vec2D::euclideanDist(shifted.back(), shifted.front()) <= threshold)
        {
            shifted.push_back(shifted.front());
            shifted.erase(shifted.begin());
        }

        // look over the shifted set
        std::vector <Vec2D> finalSet, localSubset;
        bool subsetOngoing = false;
        shifted.push_back(shifted.front());

        for(unsigned int i = 0; i < shifted.size() - 1; ++i)
        {
            if(Vec2D::euclideanDist(shifted[i], shifted[i+1]) > threshold)
            {
                if(!subsetOngoing)
                    finalSet.push_back(shifted[i]);
                else
                {
                    localSubset.push_back(shifted[i]);
                    double moyX(0.0), moyY(0.0);
                    for(unsigned int j = 0; j < localSubset.size(); ++j)
                    {
                        moyX += localSubset[j].x;
                        moyY += localSubset[j].y;
                    }
                    moyX /= static_cast<double>(localSubset.size());
                    moyY /= static_cast<double>(localSubset.size());
                    finalSet.push_back(Vec2D(moyX, moyY));
                    localSubset.clear();
                    subsetOngoing = false;
                }
            }
            else
            {
                localSubset.push_back(shifted[i]);
                if(!subsetOngoing)
                    subsetOngoing = true;
            }
        }

        double resX(0.0), resY(0.0);
        for(unsigned int i = 0; i < finalSet.size(); ++i)
        {
            resX += finalSet[i].x;
            resY += finalSet[i].y;
        }
        resX /= static_cast<double>(finalSet.size());
        resY /= static_cast<double>(finalSet.size());
        res = Vec2D(resX, resY);
    }
    return res;
}

void removeNonGroundContacts(std::map<std::string, fcl::Vec3f> & contacts, double groundThreshold)
{
    std::map<std::string, fcl::Vec3f>::const_iterator cit = contacts.begin();
    double minZ(cit->second[2]);
    for(; cit != contacts.end(); ++cit)
    {
        if(cit->second[2] < minZ)
            minZ = cit->second[2];
    }
    std::vector <std::string> outOfTheGround;
    for(cit = contacts.begin(); cit != contacts.end(); ++cit)
    {
        if(std::abs(cit->second[2] - minZ) > std::abs(groundThreshold))
            outOfTheGround.push_back(cit->first);
    }
    for(unsigned int i = 0; i < outOfTheGround.size(); ++i)
        contacts.erase(outOfTheGround[i]);
}

} // namespace sampling
} // namespace rbprm
} // namespace hpp