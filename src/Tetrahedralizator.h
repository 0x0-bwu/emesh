#pragma once
#include "generic/geometry/Tetrahedralization.hpp"
#include "MeshCommon.h"
#include <list>
namespace emesh {

using namespace generic::geometry;
using namespace generic::geometry::tet;

class Tetrahedralizator
{
public:
    using Point = Point3D<coor_t>;
    using Edge = tet::IndexEdge;
    using Face = std::vector<size_t>;
    using Vertex = tet::IndexVertex;
    using Tetrahedron = IndexTetrahedron;

    Tetrahedralization<Point> & tet;
    Tetrahedralizator(Tetrahedralization<Point> & t) : tet(t) {}

    bool Tetrahedralize(const std::vector<Point> & points, const std::list<Edge> & edges);
    bool Tetrahedralize(const std::vector<Point> & points, const std::list<Face> & faces, const std::list<Edge> & edges);
};
}//namespace emesh