#ifndef EMESH_MESHER3D_H
#define EMESH_MESHER3D_H
#include "generic/geometry/Tetrahedralization.hpp"
#include "generic/geometry/HashFunction.hpp"
#include "generic/math/MathUtility.hpp"
#include "generic/tools/Log.hpp"
#include "MeshCommon.h"
namespace emesh {
struct MeshSketchLayer3D
{
    using coor_t = int64_t;
    coor_t topH = 0, botH = 0;
    const PolygonContainer & polygons;
    const Segment2DContainer * constrains[2] = { nullptr, nullptr };
    MeshSketchLayer3D(const PolygonContainer & p)
     : polygons(p) {}
    
    void SetConstrains(const Segment2DContainer * top, const Segment2DContainer * bot)
    {
        constrains[0] = top;
        constrains[1] = bot;
    }

    void SetTopBotHeight(coor_t tH, coor_t bH)
    {
        topH = tH; botH = bH;
    }
};

using MeshSketchLayers3D = std::vector<MeshSketchLayer3D>;
using TetrahedronData = generic::geometry::tet::Tetrahedralization<Point3D<coor_t> >;
using TetrahedronDataVec = std::vector<TetrahedronData>;

class TetrahedronDataMerger
{
    using PointIndexMap = std::unordered_map<Point2D<coor_t>, size_t, PointHash<coor_t> >;
    using LayerPointIndexMap = std::unordered_map<coor_t, PointIndexMap>;
public:
    explicit TetrahedronDataMerger(TetrahedronData & data);

    void Merge(const TetrahedronData & data);

private:
    void BuildIndexMap();

private:
    TetrahedronData & m_data;
    LayerPointIndexMap m_indexMap;
};

struct Mesh3DFlowDB
{
    Mesh3DFlowDB()
     : meshCtrl(new MeshCtrl)
    {}
    Mesh3DFlowDB(Mesh3DFlowDB && ) = delete;
    Mesh3DFlowDB(const Mesh3DFlowDB & ) = delete;
    Mesh3DFlowDB & operator= (Mesh3DFlowDB && ) = delete;
    Mesh3DFlowDB & operator= (const Mesh3DFlowDB & ) = delete;

    template<typename T>
    using Data = std::unique_ptr<T>;

    Data<MeshTasks>   tasks;
    Data<std::string> workPath;
    Data<std::string> projName;
    Data<MeshCtrl>    meshCtrl;
    Data<IndexEdgeList>      edges;
    Data<StackLayerInfos>    sInfos;
    Data<TetrahedronData>    tetras;
    Data<Point3DContainer>   points;
    Data<StackLayerPolygons> inGoems;
    Data<MeshSketchLayers3D> meshSktLyrs;
    Data<InterfaceIntersections> intersections;
};

class Mesher3D
{
public:
    using float_t = float_type<coor_t>;

    Mesher3D();
    ~Mesher3D();
    bool Run();
    bool RunTest();
    Mesh3DFlowDB db;
private:
    void InitLogger();
    void CloseLogger();
    bool RunGenerateMesh();
};

class MeshFlow3D
{
    friend class Mesher3D;
    using float_t = typename Mesher3D::float_t;
public:
    static bool LoadGeometryFiles(const std::string & workPath, const std::string & projName, StackLayerPolygons & polygons, StackLayerInfos & infos);
    static bool CleanGeometries(StackLayerPolygons & polygons, coor_t distance);
    static bool ExtractInterfaceIntersections(const StackLayerPolygons & polygons, InterfaceIntersections & intersections);
    static bool ExtractInterfaceIntersection(const PolygonContainer & layer1, const PolygonContainer & layer2, Segment2DContainer & intersection);
    static bool BuildMeshSketchLayers(const StackLayerPolygons & polygons, const InterfaceIntersections & intersections, const StackLayerInfos & infos, MeshSketchLayers3D & meshSktLyrs);
    static bool GenerateTetrahedronsFromSketchLayers(const MeshSketchLayers3D & meshSktLyrs, TetrahedronDataVec & tetVec);
    static bool GenerateTetrahedronsFromSketchLayer(const MeshSketchLayer3D & meshSktLyr, TetrahedronData & tet);
    static bool ExtractTopology(const MeshSketchLayer3D & meshSktLyr, std::vector<Point3D<coor_t> > & points, std::list<IndexEdge> & edges);
    static bool ExtractTopology(StackLayerPolygons & polygons, const StackLayerInfos & infos, const InterfaceIntersections & intersections, 
                                std::vector<Point3D<coor_t> > & points, std::list<IndexEdge> & edges);
    static bool SplitOverlengthEdges(std::vector<Point3D<coor_t> > & points, std::list<IndexEdge> & edges, coor_t maxLength);
    static bool WriteNodeAndEdgeFiles(const std::string & filename, const std::vector<Point3D<coor_t> > & points, const std::list<IndexEdge> & edges);
    static bool LoadLayerStackInfos(const std::string & filename, StackLayerInfos & infos);
    static bool Tetrahedralize(const std::vector<Point3D<coor_t> > & points, const std::list<IndexEdge> & edges, TetrahedronData & tet);
    static bool MergeTetrahedrons(TetrahedronData & master, TetrahedronDataVec & tetVec);
    static bool ExportVtkFile(const std::string & filename, const TetrahedronData & tet);
};
}//namespace emesh
 #endif//EMESH_MESHER3D_H