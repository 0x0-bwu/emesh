#ifndef FEM_MESH_MESHER3D_H
#define FEM_MESH_MESHER3D_H
#include "generic/math/MathUtility.hpp"
#include "generic/tools/Log.hpp"
#include "MeshCommon.h"

namespace fem {
namespace mesh {

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
    Data<Point3DContainer>   points;
    Data<StackLayerPolygons> inGoems;
    Data<InterfaceIntersections> intersections;
};

class Mesher3D
{
public:
    using coor_t = int64_t;
    using float_t = float_type<coor_t>;

    Mesher3D();
    ~Mesher3D();
    bool Run();
    Mesh3DFlowDB db;
private:
    void InitLogger();
    void CloseLogger();
    bool RunGenerateMesh();
};

class MeshFlow3D
{
    friend class Mesher3D;
    using coor_t = typename Mesher3D::coor_t;
    using float_t = typename Mesher3D::float_t;
public:
    static bool LoadGeometryFiles(const std::string & workPath, const std::string & projName, StackLayerPolygons & polygons, std::vector<StackLayerInfo> & infos);
    static bool CleanGeometries(StackLayerPolygons & polygons, coor_t distance);
    static bool ExtractInterfaceIntersections(const StackLayerPolygons & polygons, InterfaceIntersections & intersections);
    static bool ExtractInterfaceIntersection(const PolygonContainer & layer1, const PolygonContainer & layer2, Segment2DContainer & intersection);
    static bool ExtractTopology(StackLayerPolygons & polygons, const std::vector<StackLayerInfo> & infos, const InterfaceIntersections & intersections, 
                                std::vector<Point3D<coor_t> > & points, std::list<IndexEdge> & edges);
    static bool SplitOverlengthEdges(std::vector<Point3D<coor_t> > & points, std::list<IndexEdge> & edges, coor_t maxLength);
    static bool WriteNodeAndEdgeFiles(const std::string & filename, const std::vector<Point3D<coor_t> > & points, const std::list<IndexEdge> & edges);
    static bool LoadLayerStackInfos(const std::string & filename, StackLayerInfos & infos);
};
}//namespace mesh
}//namespace fem
 #endif//FEM_MESH_MESHER3D_H