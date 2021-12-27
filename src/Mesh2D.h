#ifndef EMESH_MESH2D_H
#define EMESH_MESH2D_H
#include <boost/core/noncopyable.hpp>
#include "generic/geometry/TriangleEvaluator.hpp"
#include "generic/geometry/Triangulator.hpp"
#include "generic/math/MathUtility.hpp"
#include "MeshCommon.h"
namespace emesh {
using namespace generic;
using namespace generic::geometry;
using namespace generic::geometry::tri;
using generic::common::float_type;
struct Mesh2Options
{
    size_t threads;
    Mesh2Ctrl meshCtrl;
    std::string workPath;
    std::string projName;
    FileFormat iFileFormat = FileFormat::DomDmc;
    FileFormat oFileFormat = FileFormat::MSH;
};

struct Mesh2DB : private boost::noncopyable
{
    Mesh2DB() = default;

    template<typename T>
    using Data = std::unique_ptr<T>;

    Data<IndexEdgeList>      edges;
    Data<Point2DContainer>   points;
    Data<PolygonContainer>   inGeoms;
    Data<Segment2DContainer> segments;
    Data<TriangulationData>  triangulation;
};

using MeshEvaluation2D = typename TriangleEvaluator<Point2D<coor_t> >::Evaluation;
}//namespace emesh
#endif//EMESH_MESH2D_H