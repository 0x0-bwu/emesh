#ifndef EMESH_MESHFLOW3D_H
#define EMESH_MESHFLOW3D_H
// #include "generic/geometry/Tetrahedralization.hpp"
#include "Mesh3D.h"
#include <memory>
namespace emesh {

class MeshFlow3D
{
    friend class Mesher3D;
    struct PointExtent
    {
        Box2D<coor_t> operator()(const Point2D<coor_t> & point) const
        {
            return Box2D<coor_t>(point, point);
        }
    };
public:
    static bool LoadGeometryFiles(const std::string & workPath, const std::string & projName, StackLayerPolygons & polygons, StackLayerInfos & infos);
    static bool CleanGeometries(StackLayerPolygons & polygons, coor_t distance);
    static bool CleanLayerGeometries(PolygonContainer & polygons, coor_t distance);
    static bool ExtractInterfaceIntersections(const StackLayerPolygons & polygons, InterfaceIntersections & intersections);
    static bool ExtractInterfaceIntersection(const PolygonContainer & layer1, const PolygonContainer & layer2, Segment2DContainer & intersection);
    static bool SplitOverlengthEdges(StackLayerPolygons & polygons, InterfaceIntersections & intersections, coor_t maxLength);
    static bool BuildMeshSketchLayers(const StackLayerPolygons & polygons, const InterfaceIntersections & intersections, const StackLayerInfos & infos, MeshSketchLayers3D & meshSktLyrs);
    static bool AddGradePointsForMeshLayers(MeshSketchLayers3D & meshSktLyrs, size_t threshold);
    static bool AddGradePointsForMeshLayer(MeshSketchLayer3D & meshSktLyr, size_t threshold);
    static bool SliceOverheightLayers(MeshSketchLayers3D & meshSktLyrs, float_t ratio);
    static bool GenerateTetrahedronsFromSketchLayers(const MeshSketchLayers3D & meshSktLyrs, TetrahedronDataVec & tetVec);
    static bool GenerateTetrahedronsFromSketchLayer(const MeshSketchLayer3D & meshSktLyr, TetrahedronData & tet);
    static bool ExtractTopology(const MeshSketchLayer3D & meshSktLyr, Point3DContainer & points, std::list<IndexEdge> & edges);
    static bool SplitOverlengthEdges(Point3DContainer & points, std::list<IndexEdge> & edges, coor_t maxLength, bool surfaceOnly = true);
    static bool WriteNodeAndEdgeFiles(const std::string & filename, const Point3DContainer & points, const std::list<IndexEdge> & edges);
    static bool LoadLayerStackInfos(const std::string & filename, StackLayerInfos & infos);
    static bool Tetrahedralize(const Point3DContainer & points, const std::list<IndexEdge> & edges, const Point3DContainer * addin, TetrahedronData & tet);
    static bool MergeTetrahedrons(TetrahedronData & master, TetrahedronDataVec & tetVec);
    static bool ExportVtkFile(const std::string & filename, const TetrahedronData & tet);

    static bool SliceOverheightLayers(std::list<MeshSketchLayer3D> & meshSktLyrs, float_t ratio);
    static void SplitOverlengthIntersections(InterfaceIntersections & intersections, coor_t maxLength);
    static void SplitOverlengthSegments(Segment2DContainer & segments, coor_t maxLength);
    static void SplitOverlengthPolygons(PolygonContainer & polygons, coor_t maxLength);
    static void SplitOverlengthPolygon(Polygon2D<coor_t> & polygon, coor_t maxLength);
    static void SplitOverlengthEdges(Point2DContainer & points, std::list<IndexEdge> & edges, coor_t maxLength);
    static std::unique_ptr<Point2DContainer> AddPointsFromBalancedQuadTree(const Segment2DContainer & segments, size_t threshold);
    static std::unique_ptr<Point2DContainer> AddPointsFromBalancedQuadTree(const PolygonContainer & polygons, size_t threshold);
    static std::unique_ptr<Point2DContainer> AddPointsFromBalancedQuadTree(std::list<Point2D<coor_t> * > points, size_t threshold);
};

}//namespace emesh
#endif//EMESH_MESHFLOW3D_H