#pragma once
#include <boost/core/noncopyable.hpp>
#include "generic/geometry/TriangleEvaluator.hpp"
#include "generic/geometry/Triangulator.hpp"
#include "generic/math/MathUtility.hpp"
#include "generic/tools/Log.hpp"
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

    Data<MeshTasks>          tasks;
    Data<IndexEdgeList>      edges;
    Data<Point2DContainer>   points;
    Data<PolygonContainer>   inGeoms;
    Data<Segment2DContainer> segments;
    Data<TriangulationData>  triangulation;
};

class Mesher2D
{
public:
    using float_t = float_type<coor_t>;
    using MeshEvaluation = typename TriangleEvaluator<Point2D<coor_t> >::Evaluation;

    Mesher2D();
    ~Mesher2D();
    bool Run();

    Mesh2DB db;
    Mesh2Options options;
private:
    void InitLogger();
    void CloseLogger();
    bool RunTasks();
    bool RunGenerateMesh();
    bool RunMeshEvaluation();
};

class MeshFlow2D
{
    friend class Mesher2D;
    using float_t = typename Mesher2D::float_t;
public:
    static bool LoadGeometryFiles(const std::string & filename, FileFormat format, float_t scale, PolygonContainer & polygons);
    static bool ExtractIntersections(const PolygonContainer & polygons, std::vector<Segment2D<coor_t> > & segments);
    static bool ExtractTopology(const std::vector<Segment2D<coor_t> > & segments, std::vector<Point2D<coor_t> > & points, std::list<IndexEdge> & edges);
    static bool MergeClosePointsAndRemapEdge(std::vector<Point2D<coor_t> > & points, std::list<IndexEdge> & edges, coor_t tolorance);
    static bool SplitOverlengthEdges(std::vector<Point2D<coor_t> > & points, std::list<IndexEdge> & edges, coor_t maxLength);
    static bool AddPointsFromBalancedQuadTree(const Polygon2D<coor_t> & outline, std::vector<Point2D<coor_t> > & points, size_t threshold);
    static bool TriangulatePointsAndEdges(const std::vector<Point2D<coor_t> > & points, const std::list<IndexEdge> & edges, Triangulation<Point2D<coor_t> > & triangulation);
    static bool TriangulationRefinement(Triangulation<Point2D<coor_t> > & triangulation, float_t minAlpha, coor_t minLen, coor_t maxLen, size_t iteration);
    static bool ExportMeshResult(const std::string & filename, FileFormat format, const Triangulation<Point2D<coor_t> > & triangulation, float_t scale);
    static bool GenerateReport(const std::string & filename, const TriangulationData & triangulation, float_t scale);
};
}//namespace emesh