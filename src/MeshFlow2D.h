#ifndef EMESH_MESHFLOW2D_H
#define EMESH_MESHFLOW2D_H
#include "Mesh2D.h"
namespace emesh {
class MeshFlow2D
{
public:
    static bool LoadGeometryFiles(const std::string & filename, FileFormat format, PolygonContainer & polygons);
    static bool ExtractIntersections(const PolygonContainer & polygons, Segment2DContainer & segments);
    static bool RemoveDuplicatedSegments(Segment2DContainer & segments);
    static bool MergeClosestParallelSegmentsIteratively(Segment2DContainer & segments, coor_t tolerance);
    static bool ExtractTopology(const Segment2DContainer & segments, Point2DContainer & points, std::list<IndexEdge> & edges);
    static bool MergeClosePointsAndRemapEdge(Point2DContainer & points, std::list<IndexEdge> & edges, coor_t tolerance);
    static bool SplitOverlengthEdges(Point2DContainer & points, std::list<IndexEdge> & edges, coor_t maxLength);
    static bool AddPointsFromBalancedQuadTree(Point2DContainer & points, std::list<IndexEdge> & edges, size_t maxLevel);
    static bool TriangulatePointsAndEdges(const Point2DContainer & points, const std::list<IndexEdge> & edges, Triangulation<Point2D<coor_t> > & triangulation);
    static bool TriangulationRefinement(Triangulation<Point2D<coor_t> > & triangulation, const Mesh2Ctrl & ctrl);
    static bool ExportMeshResult(const std::string & filename, FileFormat format, const Triangulation<Point2D<coor_t> > & triangulation);
    static bool GenerateReport(const std::string & filename, const TriangulationData & triangulation);
private:    
    static bool MergeClosestParallelSegments(Segment2DContainer & segments, coor_t tolerance);
    static void MergeClosestParallelSegments(const Segment2D<coor_t> & seg1, const Segment2D<coor_t> & seg2, Segment2DContainer & added);
};
}//namespace emesh
 #endif//EMESH_MESHFLOW2D_H