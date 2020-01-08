 

#ifndef OPENCV_STITCHING_UTIL_INL_HPP
#define OPENCV_STITCHING_UTIL_INL_HPP

#include <queue>
#include "opencv2/core.hpp"
#include "util.hpp" // Make your IDE see declarations

//! @cond IGNORED

namespace chamo {
namespace panod {

template <typename B>
B Graph::forEach(B body) const
{
    for (int i = 0; i < numVertices(); ++i)
    {
        std::list<GraphEdge>::const_iterator edge = edges_[i].begin();
        for (; edge != edges_[i].end(); ++edge)
            body(*edge);
    }
    return body;
}


template <typename B>
B Graph::walkBreadthFirst(int from, B body) const
{
    std::vector<bool> was(numVertices(), false);
    std::queue<int> vertices;

    was[from] = true;
    vertices.push(from);

    while (!vertices.empty())
    {
        int vertex = vertices.front();
        vertices.pop();

        std::list<GraphEdge>::const_iterator edge = edges_[vertex].begin();
        for (; edge != edges_[vertex].end(); ++edge)
        {
            if (!was[edge->to])
            {
                body(*edge);
                was[edge->to] = true;
                vertices.push(edge->to);
            }
        }
    }

    return body;
}


//////////////////////////////////////////////////////////////////////////////
// Some auxiliary math functions

static inline
float normL2(const Point3f& a)
{
    return a.x * a.x + a.y * a.y + a.z * a.z;
}


static inline
float normL2(const Point3f& a, const Point3f& b)
{
    return normL2(a - b);
}


static inline
double normL2sq(const Mat &r)
{
    return r.dot(r);
}


static inline int sqr(int x) { return x * x; }
static inline float sqr(float x) { return x * x; }
static inline double sqr(double x) { return x * x; }

} // namespace panod
} // namespace cv

//! @endcond

#endif // OPENCV_STITCHING_UTIL_INL_HPP
