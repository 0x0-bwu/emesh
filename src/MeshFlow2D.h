#ifndef EMESH_MESHFLOW2D_H
#define EMESH_MESHFLOW2D_H
#include "Mesh2D.h"
namespace emesh {
class MeshFlow2D
{
public:
    static bool LoadGeometryFiles(const std::string & filename, FileFormat format, PolygonContainer & polygons);
    static bool ExtractIntersections(const PolygonContainer & polygons, std::vector<Segment2D<coor_t> > & segments);
    static bool ExtractTopology(const std::vector<Segment2D<coor_t> > & segments, std::vector<Point2D<coor_t> > & points, std::list<IndexEdge> & edges);
    static bool MergeClosePointsAndRemapEdge(std::vector<Point2D<coor_t> > & points, std::list<IndexEdge> & edges, coor_t tolorance);
    static bool SplitOverlengthEdges(std::vector<Point2D<coor_t> > & points, std::list<IndexEdge> & edges, coor_t maxLength);
    static bool AddPointsFromBalancedQuadTree(const Polygon2D<coor_t> & outline, std::vector<Point2D<coor_t> > & points, size_t threshold);
    static bool TriangulatePointsAndEdges(const std::vector<Point2D<coor_t> > & points, const std::list<IndexEdge> & edges, Triangulation<Point2D<coor_t> > & triangulation);
    static bool TriangulationRefinement(Triangulation<Point2D<coor_t> > & triangulation, float_t minAlpha, coor_t minLen, coor_t maxLen, size_t iteration);
    static bool ExportMeshResult(const std::string & filename, FileFormat format, const Triangulation<Point2D<coor_t> > & triangulation);
    static bool GenerateReport(const std::string & filename, const TriangulationData & triangulation);
};
}//namespace emesh
 #endif//EMESH_MESHFLOW2D_H