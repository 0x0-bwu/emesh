#ifndef EMESH_MESH3D_H
#define EMESH_MESH3D_H
#include "generic/geometry/Tetrahedralization.hpp"
#include "generic/geometry/HashFunction.hpp"
#include "MeshCommon.h"
#include <array>
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

    void SetConstrains(const Segment2DContainer * top, const Segment2DContainer * bot);
    void SetTopBotHeight(coor_t tH, coor_t bH);
    coor_t GetHeight() const;

    std::unique_ptr<Point3DContainer> GetAdditionalPoints() const;
    std::pair<MeshSketchLayer3D, MeshSketchLayer3D> Split() const;
};

inline std::array<Box2D<coor_t>, 4> Split(const Box2D<coor_t> & bbox)
{   
    std::array<Box2D<coor_t>, 4> sub;
    auto ct = bbox.Center().template Cast<coor_t>();
    auto ll = Point2D<coor_t>(bbox[0][0], bbox[0][1]);
    auto lm = Point2D<coor_t>(ct[0], bbox[0][1]);
    auto mr = Point2D<coor_t>(bbox[1][0], ct[1]);
    auto ur = Point2D<coor_t>(bbox[1][0], bbox[1][1]);
    auto um = Point2D<coor_t>(ct[0], bbox[1][1]);
    auto ml = Point2D<coor_t>(bbox[0][0], ct[1]);
    sub[0] = Box2D<coor_t>(ll, ct);
    sub[1] = Box2D<coor_t>(lm, mr);
    sub[2] = Box2D<coor_t>(ct, ur);
    sub[3] = Box2D<coor_t>(ml, um);
    return sub;
}

struct MeshSketchModel3D;
struct IntersectionModel
{
    MeshSketchModel3D * models[2] = {nullptr, nullptr};
    Segment2DContainer segments;
    std::array<std::shared_ptr<IntersectionModel>, 4> Split(const Box2D<coor_t> & box);
};


struct MeshSketchModel3D
{
    using coor_t = int64_t;
    Box2D<coor_t> bbox;
    coor_t height[2] = {0, 0};//0 - top, 1 - bot
    std::shared_ptr<PolygonContainer> polygons = nullptr;
    std::shared_ptr<IntersectionModel> constrains[2] = { nullptr, nullptr };
    std::unique_ptr<MeshSketchModel3D> subModels[4] = { nullptr, nullptr, nullptr, nullptr };

    void CreateSubModels()
    {
        auto sub = Split(bbox);
        for(size_t i = 0; i < 4; ++i){
            subModels[i] = std::make_unique<MeshSketchModel3D>();
            subModels[i]->bbox = sub[i];
            subModels[i]->height[0] = height[0];
            subModels[i]->height[1] = height[1];
            subModels[i]->polygons = std::make_shared<PolygonContainer>();
        }
        //todo
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

}//namespace emesh
#endif//EMESH_MESH3D_H