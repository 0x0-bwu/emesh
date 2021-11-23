#ifndef EMESH_MESHER2D_H
#define EMESH_MESHER2D_H
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

struct Mesh2DFlowDB
{
    Mesh2DFlowDB()
     : meshCtrl(new MeshCtrl),
       inFormat(new FileFormat(FileFormat::DomDmc)),
       outFormat(new FileFormat(FileFormat::MSH))
    {}
    Mesh2DFlowDB(Mesh2DFlowDB && ) = delete;
    Mesh2DFlowDB(const Mesh2DFlowDB & ) = delete;
    Mesh2DFlowDB & operator= (Mesh2DFlowDB && ) = delete;
    Mesh2DFlowDB & operator= (const Mesh2DFlowDB & ) = delete;

    template<typename T>
    using Data = std::unique_ptr<T>;

    Data<MeshTasks>   tasks;
    Data<std::string> workPath;
    Data<std::string> projName;
    Data<MeshCtrl>    meshCtrl;
    Data<FileFormat>  inFormat;
    Data<FileFormat>  outFormat;
    Data<IndexEdgeList>      edges;
    Data<Point2DContainer>   points;
    Data<PolygonContainer>   inGoems;
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
    Mesh2DFlowDB db;
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
    static bool LoadGeometryFiles(const std::string & filename, FileFormat format, float_t scale2Int, std::list<Polygon2D<coor_t> > & polygons);
    static bool ExtractIntersections(const std::list<Polygon2D<coor_t> > & polygons, std::vector<Segment2D<coor_t> > & segments);
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
 #endif//EMESHER2D_H