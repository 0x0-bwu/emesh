#ifndef EMESH_MESHCOMMON_H
#define EMESH_MESHCOMMON_H
#include "generic/geometry/TriangleEvaluator.hpp"
#include "generic/geometry/Triangulator.hpp"
#include "generic/math/MathUtility.hpp"
#include "generic/tools/Log.hpp"
#include <memory>
#include <string>
#include <queue>
namespace emesh {

using namespace generic;
using namespace generic::geometry;
using namespace generic::geometry::tri;
using generic::common::float_type;
using coor_t = int64_t;
using float_t = float_type<coor_t>;

enum class FileFormat { DomDmc, WKT, MSH };
enum class MeshTask { MeshGeneration, MeshEvaluation };

struct MeshCtrl2D
{
    coor_t tolerance = 0;
    coor_t maxEdgeLen = 0;
    coor_t minEdgeLen = 100;
    float_t scale2Int = 1e3;
    float_t minAlpha = math::Rad(15);
};

struct MeshCtrl3D
{
    size_t threads = 4;
    coor_t tolerance = 100;
    coor_t maxEdgeLenH = 0;
    float_t smartZRatio = 0.0; 
};

struct StackLayerInfo
{
    coor_t elevation;
    coor_t thickness;
};

using MeshTasks = std::queue<MeshTask>;
using IndexEdgeList = std::list<IndexEdge>;
using Point2DContainer = std::vector<Point2D<coor_t> >;
using Point3DContainer = std::vector<Point3D<coor_t> >;
using PolygonContainer = std::list<Polygon2D<coor_t> >;
using TriangulationData = Triangulation<Point2D<coor_t> >;
using Segment2DContainer = std::vector<Segment2D<coor_t> >;
using Segment3DContainer = std::vector<Segment3D<coor_t> >;

using StackLayerInfos = std::vector<StackLayerInfo>;
using StackLayerPolygons = std::vector<PolygonContainer>;
using InterfaceIntersections = std::vector<Segment2DContainer>;

}//namespace emesh
#endif//EMESH_MESHCOMMON_H