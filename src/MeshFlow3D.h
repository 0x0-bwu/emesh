#ifndef EMESH_MESHFLOW3D_H
#define EMESH_MESHFLOW3D_H
#include "Mesh3D.h"
namespace emesh {

class MeshFlow3D
{
public:
    static bool LoadGeometryFiles(const std::string & filename, FileFormat format, StackLayerPolygons & polygons, StackLayerInfos & infos);
    static bool CleanGeometries(StackLayerPolygons & polygons, coor_t distance);
    static bool CleanLayerGeometries(PolygonContainer & polygons, coor_t distance);
    static bool ExtractModelsIntersections(std::vector<Ptr<StackLayerModel> > & models);
    static bool ExtractModelIntersections(StackLayerModel & model);
    static bool ExtractInterfaceIntersections(const StackLayerPolygons & polygons, InterfaceIntersections & intersections);
    static bool ExtractInterfaceIntersection(const PolygonContainer & layer, Segment2DContainer & intersection);
    static bool ExtractInterfaceIntersection(const PolygonContainer & layer1, const PolygonContainer & layer2, Segment2DContainer & intersection);
    // static bool SplitOverlengthEdges(const StackLayerModel & model, coor_t maxLength);
    // static bool SplitOverlengthEdges(StackLayerPolygons & polygons, InterfaceIntersections & intersections, coor_t maxLength);
    static bool BuildMeshSketchModels(StackLayerModel & model, std::vector<MeshSketchModel> & models);
    static bool BuildMeshSketchModel(const StackLayerPolygons & polygons, const InterfaceIntersections & intersections, const StackLayerInfos & infos, MeshSketchModel & model);
    static bool AddGradePointsForMeshModels(std::vector<MeshSketchModel> & models, size_t threshold);
    static bool AddGradePointsForMeshModel(MeshSketchModel & model, size_t threshold);
    static bool AddGradePointsForMeshLayer(MeshSketchLayer & layer, size_t threshold);
    static bool SliceOverheightModels(std::vector<MeshSketchModel> & models, float_t ratio);
    static bool SliceOverheightLayers(MeshSketchModel & model, float_t ratio);
    static bool GenerateTetrahedronVecFromSketchModels(std::vector<MeshSketchModel> & models, TetrahedronDataVec & tetVec);
    static bool GenerateTetrahedronVecFromSketchModel(MeshSketchModel & model, TetrahedronDataVec & tetVec);
    static bool GenerateTetrahedronDataFromSketchModel(MeshSketchModel & model, TetrahedronData & tet);
    static bool GenerateTetrahedronDataFromSketchLayer(const MeshSketchLayer & layer, TetrahedronData & tet);
    static bool ExtractTopology(const MeshSketchLayer & layer, Point3DContainer & points, std::list<IndexFace> & faces, std::list<IndexEdge> & edges);
    static bool SplitOverlengthEdges(Point3DContainer & points, std::list<IndexEdge> & edges, coor_t maxLength, bool surfaceOnly = true);
    static bool WriteNodeAndEdgeFiles(const std::string & filename, const Point3DContainer & points, const std::list<IndexEdge> & edges);
    static bool LoadLayerStackInfos(const std::string & filename, StackLayerInfos & infos);
    static bool Tetrahedralize(const Point3DContainer & points, const std::list<std::vector<size_t> > & faces, const std::list<IndexEdge> & edges, TetrahedronData & tet);
    static bool MergeTetrahedrons(TetrahedronData & master, TetrahedronDataVec & tetVec);
    static bool ExportResultFile(const std::string & filename,  FileFormat format, const TetrahedronData & tet);

    static bool SliceOverheightLayers(std::list<MeshSketchLayer> & layers, float_t ratio);
    // static void SplitOverlengthIntersections(InterfaceIntersections & intersections, coor_t maxLength);
    // static void SplitOverlengthSegments(Segment2DContainer & segments, coor_t maxLength);
    // static void SplitOverlengthPolygons(PolygonContainer & polygons, coor_t maxLength);
    // static void SplitOverlengthPolygon(Polygon2D<coor_t> & polygon, coor_t maxLength);
    // static void SplitOverlengthEdges(Point2DContainer & points, std::list<IndexEdge> & edges, coor_t maxLength);
    static void Polygons2Segments(const PolygonContainer & layer, std::list<Segment2D<coor_t> > & segments);
    static std::unique_ptr<Point2DContainer> AddPointsFromBalancedQuadTree(const Segment2DContainer & segments, size_t threshold);
    static std::unique_ptr<Point2DContainer> AddPointsFromBalancedQuadTree(const PolygonContainer & polygons, size_t threshold);
    static std::unique_ptr<Point2DContainer> AddPointsFromBalancedQuadTree(std::list<Point2D<coor_t> * > points, size_t threshold);
};

class MeshFlow3DMT
{
public:
    static bool CleanGeometries(StackLayerPolygons & polygons, coor_t distance, size_t threads = maxThreads);
    static bool ExtractModelsIntersections(std::vector<StackLayerModel * > & models, size_t threads = maxThreads);
    // static bool ExtractInterfaceIntersections(StackLayerModel & model, size_t threads = maxThreads);
    // static bool ExtractInterfaceIntersections(const StackLayerPolygons & polygons, InterfaceIntersections & intersections, size_t threads = maxThreads);
    static bool SplitOverlengthEdges(const StackLayerModel & model, coor_t maxLength, size_t threads = maxThreads); 
    static bool SplitOverlengthEdges(StackLayerPolygons & polygons, InterfaceIntersections & intersections, coor_t maxLength, size_t threads = maxThreads);
    static bool GenerateTetrahedronVecFromSketchModels(std::vector<MeshSketchModel> & models, TetrahedronDataVec & tetVec, size_t threads = maxThreads);
    // static bool AddGradePointsForMeshModel(MeshSketchModel & model, size_t threshold, size_t threads = maxThreads);
    // static bool GenerateTetrahedronsFromSketchModel(const MeshSketchModel & model, TetrahedronDataVec & tetVec, size_t threads = maxThreads);
};

}//namespace emesh
#endif//EMESH_MESHFLOW3D_H