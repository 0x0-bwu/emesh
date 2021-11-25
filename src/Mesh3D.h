#ifndef EMESH_MESH3D_H
#define EMESH_MESH3D_H
#include "generic/geometry/Tetrahedralization.hpp"
#include "generic/geometry/HashFunction.hpp"
#include "MeshCommon.h"
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