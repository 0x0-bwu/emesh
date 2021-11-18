#ifndef FEM_MESH_MESHCOMMON_H
#define FEM_MESH_MESHCOMMON_H
#include "generic/geometry/TriangleEvaluator.hpp"
#include "generic/geometry/Triangulator.hpp"
#include "generic/math/MathUtility.hpp"
#include "generic/tools/Log.hpp"
#include <memory>
#include <string>
#include <queue>
namespace fem {
namespace mesh {

using namespace generic;
using namespace generic::geometry;
using namespace generic::geometry::tri;
using generic::common::float_type;

enum class FileFormat { DomDmc, WKT, MSH };
enum class MeshTask { MeshGeneration, MeshEvaluation };

struct MeshCtrl
{
    using coor_t = int64_t;
    using float_t = float_type<coor_t>;
    coor_t tolerance = 0;
    coor_t maxEdgeLen = 0;
    coor_t minEdgeLen = 100;
    float_t scale2Int = 1e3;
    float_t minAlpha = math::Rad(15);
};

struct StackLayerInfo
{
    using coor_t = int64_t;
    coor_t elevation;
    coor_t thickness;
};

using MeshTasks = std::queue<MeshTask>;
using IndexEdgeList = std::list<IndexEdge>;
using Point2DContainer = std::vector<Point2D<typename MeshCtrl::coor_t> >;
using Point3DContainer = std::vector<Point3D<typename MeshCtrl::coor_t> >;
using PolygonContainer = std::list<Polygon2D<typename MeshCtrl::coor_t> >;
using TriangulationData = Triangulation<Point2D<typename MeshCtrl::coor_t> >;
using Segment2DContainer = std::vector<Segment2D<typename MeshCtrl::coor_t> >;
using Segment3DContainer = std::vector<Segment3D<typename MeshCtrl::coor_t> >;

using StackLayerInfos = std::vector<StackLayerInfo>;
using StackLayerPolygons = std::vector<PolygonContainer>;
using InterfaceIntersections = std::vector<Segment2DContainer>;

}//namespace mesh
}//namespace fem
#endif//FEM_MESH_MESHCOMMON_H