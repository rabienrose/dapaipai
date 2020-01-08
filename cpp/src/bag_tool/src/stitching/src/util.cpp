 

#include "precomp.hpp"

namespace chamo {
namespace panod {

void DisjointSets::createOneElemSets(int n)
{
    rank_.assign(n, 0);
    size.assign(n, 1);
    parent.resize(n);
    for (int i = 0; i < n; ++i)
        parent[i] = i;
}


int DisjointSets::findSetByElem(int elem)
{
    int set = elem;
    while (set != parent[set])
        set = parent[set];
    int next;
    while (elem != parent[elem])
    {
        next = parent[elem];
        parent[elem] = set;
        elem = next;
    }
    return set;
}


int DisjointSets::mergeSets(int set1, int set2)
{
    if (rank_[set1] < rank_[set2])
    {
        parent[set1] = set2;
        size[set2] += size[set1];
        return set2;
    }
    if (rank_[set2] < rank_[set1])
    {
        parent[set2] = set1;
        size[set1] += size[set2];
        return set1;
    }
    parent[set1] = set2;
    rank_[set2]++;
    size[set2] += size[set1];
    return set2;
}


void Graph::addEdge(int from, int to, float weight)
{
    edges_[from].push_back(GraphEdge(from, to, weight));
}


bool overlapRoi(Point tl1, Point tl2, Size sz1, Size sz2, Rect &roi)
{
    int x_tl = std::max(tl1.x, tl2.x);
    int y_tl = std::max(tl1.y, tl2.y);
    int x_br = std::min(tl1.x + sz1.width, tl2.x + sz2.width);
    int y_br = std::min(tl1.y + sz1.height, tl2.y + sz2.height);
    if (x_tl < x_br && y_tl < y_br)
    {
        roi = Rect(x_tl, y_tl, x_br - x_tl, y_br - y_tl);
        return true;
    }
    return false;
}


Rect resultRoi(const std::vector<Point> &corners, const std::vector<UMat> &images)
{
    std::vector<Size> sizes(images.size());
    for (size_t i = 0; i < images.size(); ++i)
        sizes[i] = images[i].size();
    return resultRoi(corners, sizes);
}


Rect resultRoi(const std::vector<Point> &corners, const std::vector<Size> &sizes)
{
    CV_Assert(sizes.size() == corners.size());
    Point tl(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
    Point br(std::numeric_limits<int>::min(), std::numeric_limits<int>::min());
    for (size_t i = 0; i < corners.size(); ++i)
    {
        tl.x = std::min(tl.x, corners[i].x);
        tl.y = std::min(tl.y, corners[i].y);
        br.x = std::max(br.x, corners[i].x + sizes[i].width);
        br.y = std::max(br.y, corners[i].y + sizes[i].height);
    }
    return Rect(tl, br);
}

Rect resultRoiIntersection(const std::vector<Point> &corners, const std::vector<Size> &sizes)
{
    CV_Assert(sizes.size() == corners.size());
    Point tl(std::numeric_limits<int>::min(), std::numeric_limits<int>::min());
    Point br(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
    for (size_t i = 0; i < corners.size(); ++i)
    {
        tl.x = std::max(tl.x, corners[i].x);
        tl.y = std::max(tl.y, corners[i].y);
        br.x = std::min(br.x, corners[i].x + sizes[i].width);
        br.y = std::min(br.y, corners[i].y + sizes[i].height);
    }
    return Rect(tl, br);
}


Point resultTl(const std::vector<Point> &corners)
{
    Point tl(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
    for (size_t i = 0; i < corners.size(); ++i)
    {
        tl.x = std::min(tl.x, corners[i].x);
        tl.y = std::min(tl.y, corners[i].y);
    }
    return tl;
}


void selectRandomSubset(int count, int size, std::vector<int> &subset)
{
    subset.clear();
    for (int i = 0; i < size; ++i)
    {
        if (randu<int>() % (size - i) < count)
        {
            subset.push_back(i);
            count--;
        }
    }
}

int& stitchingLogLevel()
{
    static int _log_level=1;
    return _log_level;
}

} // namespace panod
} // namespace cv
