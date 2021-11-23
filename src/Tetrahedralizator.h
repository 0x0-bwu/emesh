#ifndef EMESH_TETRAHEDRALIZATOR_H
#define EMESH_TETRAHEDRALIZATOR_H
#include "generic/geometry/Tetrahedralization.hpp"
#include "MeshCommon.h"
namespace emesh {

using namespace generic::geometry;
using namespace generic::geometry::tet;

class Tetrahedralizator
{
public:
    using Point = Point3D<coor_t>;
    using Edge = tet::IndexEdge;
    using Vertex = tet::IndexVertex;
    using Tetrahedron = IndexTetrahedron;

    Tetrahedralization<Point> & tet;
    Tetrahedralizator(Tetrahedralization<Point> & t) : tet(t) {}

    bool Tetrahedralize(const std::vector<Point> & points, const std::list<Edge> & edges);
};
}//namespace emesh
#endif//EMESH_TETRAHEDRALIZATOR_H