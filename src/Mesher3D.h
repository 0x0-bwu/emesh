#ifndef EMESH_MESHER3D_H
#define EMESH_MESHER3D_H
#include "generic/geometry/Tetrahedralization.hpp"
#include "generic/geometry/HashFunction.hpp"
#include "generic/math/MathUtility.hpp"
#include "generic/tools/Log.hpp"
#include "MeshCommon.h"
#include <memory>
namespace emesh {
struct MeshSketchLayer3D
{
    using coor_t = int64_t;
    coor_t topH = 0, botH = 0;
    const PolygonContainer & polygons;
    const Segment2DContainer * constrains[2] = { nullptr, nullptr };
    std::shared_ptr<Point2DContainer> addPoints[2] = { nullptr, nullptr};
    MeshSketchLayer3D(const PolygonContainer & p)
     : polygons(p) {}
    
    MeshSketchLayer3D(const MeshSketchLayer3D & other) = default;
    MeshSketchLayer3D & operator= (const MeshSketchLayer3D & other) = default;

    void SetConstrains(const Segment2DContainer * top, const Segment2DContainer * bot)
    {
        constrains[0] = top;
        constrains[1] = bot;
    }

    void SetTopBotHeight(coor_t tH, coor_t bH)
    {
        topH = tH; botH = bH;
    }

    coor_t GetHeight() const
    {
        return topH - botH;
    }

    std::unique_ptr<Point3DContainer> GetAdditionalPoints() const
    {
        Point3DContainer points;
        if(addPoints[0]){
            points.reserve(addPoints[0]->size());
            for(const auto & p : *addPoints[0])
                points.push_back(Point3D<coor_t>(p[0], p[1], topH));
        }
        if(addPoints[1]){
            points.reserve(points.size() + addPoints[1]->size());
            for(const auto & p : *addPoints[1])
                points.push_back(Point3D<coor_t>(p[0], p[1], botH));
        }
        if(0 == points.size()) return nullptr;
        return std::make_unique<Point3DContainer>(points);   
    }

    std::pair<MeshSketchLayer3D, MeshSketchLayer3D> Split() const
    {
        MeshSketchLayer3D top = *this;
        MeshSketchLayer3D bot = *this;
        auto mid = (topH + botH) / 2;
        top.botH = mid;
        bot.topH = mid;
        bot.addPoints[0] = addPoints[1];
        bot.constrains[0] = constrains[1];//C0-C1 -> C0-C1-C1
        return std::make_pair(top, bot);
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
     : meshCtrl(new MeshCtrl3D)
    {}
    Mesh3DFlowDB(Mesh3DFlowDB && ) = delete;
    Mesh3DFlowDB(const Mesh3DFlowDB & ) = delete;
    Mesh3DFlowDB & operator= (Mesh3DFlowDB && ) = delete;
    Mesh3DFlowDB & operator= (const Mesh3DFlowDB & ) = delete;

    template<typename T>
    using Data = std::unique_ptr<T>;

    Data<std::string> workPath;
    Data<std::string> projName;
    Data<MeshCtrl3D>       meshCtrl;
    Data<StackLayerInfos>    sInfos;
    Data<TetrahedronData>    tetras;
    Data<StackLayerPolygons> inGoems;
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
    static bool ExtractInterfaceIntersections(const StackLayerPolygons & polygons, InterfaceIntersections & intersections);
    static bool ExtractInterfaceIntersection(const PolygonContainer & layer1, const PolygonContainer & layer2, Segment2DContainer & intersection);
    static bool BuildMeshSketchLayers(const StackLayerPolygons & polygons, const InterfaceIntersections & intersections, const StackLayerInfos & infos, MeshSketchLayers3D & meshSktLyrs);
    static bool AddPointsFromBalancedQuadTree(MeshSketchLayers3D & meshSktLyrs, size_t threshold);
    static bool AddPointsFromBalancedQuadTree(MeshSketchLayer3D & meshSktLyr, size_t threshold);
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

private:
    static bool SliceOverheightLayers(std::list<MeshSketchLayer3D> & meshSktLyrs, float_t ratio);
    static std::unique_ptr<Point2DContainer> AddPointsFromBalancedQuadTree(const Segment2DContainer & segments, size_t threshold);
    static std::unique_ptr<Point2DContainer> AddPointsFromBalancedQuadTree(const PolygonContainer & polygons, size_t threshold);
    static std::unique_ptr<Point2DContainer> AddPointsFromBalancedQuadTree(std::list<Point2D<coor_t> * > points, size_t threshold);
};
}//namespace emesh
 #endif//EMESH_MESHER3D_H