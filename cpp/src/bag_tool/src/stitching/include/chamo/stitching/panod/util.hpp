 

#ifndef OPENCV_STITCHING_UTIL_HPP
#define OPENCV_STITCHING_UTIL_HPP

#include <list>
#include "opencv2/core.hpp"
using namespace cv;
namespace chamo {
namespace panod {

//! @addtogroup stitching
//! @{

class CV_EXPORTS DisjointSets
{
public:
    DisjointSets(int elem_count = 0) { createOneElemSets(elem_count); }

    void createOneElemSets(int elem_count);
    int findSetByElem(int elem);
    int mergeSets(int set1, int set2);

    std::vector<int> parent;
    std::vector<int> size;

private:
    std::vector<int> rank_;
};


struct CV_EXPORTS GraphEdge
{
    GraphEdge(int from, int to, float weight);
    bool operator <(const GraphEdge& other) const { return weight < other.weight; }
    bool operator >(const GraphEdge& other) const { return weight > other.weight; }

    int from, to;
    float weight;
};

inline GraphEdge::GraphEdge(int _from, int _to, float _weight) : from(_from), to(_to), weight(_weight) {}


class CV_EXPORTS Graph
{
public:
    Graph(int num_vertices = 0) { create(num_vertices); }
    void create(int num_vertices) { edges_.assign(num_vertices, std::list<GraphEdge>()); }
    int numVertices() const { return static_cast<int>(edges_.size()); }
    void addEdge(int from, int to, float weight);
    template <typename B> B forEach(B body) const;
    template <typename B> B walkBreadthFirst(int from, B body) const;

private:
    std::vector< std::list<GraphEdge> > edges_;
};


//////////////////////////////////////////////////////////////////////////////
// Auxiliary functions

CV_EXPORTS_W bool overlapRoi(Point tl1, Point tl2, Size sz1, Size sz2, Rect &roi);
CV_EXPORTS_W Rect resultRoi(const std::vector<Point> &corners, const std::vector<UMat> &images);
CV_EXPORTS_W Rect resultRoi(const std::vector<Point> &corners, const std::vector<Size> &sizes);
CV_EXPORTS_W Rect resultRoiIntersection(const std::vector<Point> &corners, const std::vector<Size> &sizes);
CV_EXPORTS_W Point resultTl(const std::vector<Point> &corners);

// Returns random 'count' element subset of the {0,1,...,size-1} set
CV_EXPORTS_W void selectRandomSubset(int count, int size, std::vector<int> &subset);

CV_EXPORTS_W int& stitchingLogLevel();

//! @}

} // namespace panod
} // namespace cv

#include "util_inl.hpp"

#endif // OPENCV_STITCHING_UTIL_HPP
