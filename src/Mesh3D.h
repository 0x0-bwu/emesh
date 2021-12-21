#ifndef EMESH_MESH3D_H
#define EMESH_MESH3D_H
#include "generic/geometry/Tetrahedralization.hpp"
#include "generic/geometry/BooleanOperation.hpp"
#include "generic/geometry/HashFunction.hpp"
#include "generic/geometry/Utility.hpp"
#include "MeshCommon.h"
#include <array>
namespace emesh {

class MeshSketchLayer
{
public:
    Polygon2D<coor_t> outline;//wbtest
    coor_t height[2] = {0, 0};//0 - top, 1 - bot
    SPtr<PolygonContainer> polygons = nullptr;
    SPtr<Segment2DContainer> constrains[2] = { nullptr, nullptr };
    SPtr<Point2DContainer> addPoints[2] = { nullptr, nullptr };

    MeshSketchLayer() = default;
    MeshSketchLayer(const MeshSketchLayer & other) = default;
    MeshSketchLayer & operator= (const MeshSketchLayer & other) = default;

    void SetConstrains(SPtr<Segment2DContainer> top, SPtr<Segment2DContainer> bot);
    void SetTopBotHeight(coor_t tH, coor_t bH);
    coor_t GetHeight() const;
    UPtr<Point3DContainer> GetAdditionalPoints() const;
    std::pair<MeshSketchLayer, MeshSketchLayer> Slice() const;
};

class MeshSketchModel
{
public:
    Box2D<coor_t> bbox;
    std::vector<MeshSketchLayer> layers;
    
    MeshSketchModel() = default;
    MeshSketchModel(const MeshSketchModel & other) = default;
    MeshSketchModel & operator= (const MeshSketchModel & other) = default;
};

class StackLayerModel
{
public:
    Box2D<coor_t> bbox;
    SPtr<StackLayerInfos> sInfos = nullptr;
    SPtr<StackLayerPolygons> inGeoms = nullptr;
    SPtr<InterfaceIntersections> intersections = nullptr;
    UPtr<StackLayerModel> subModels[4] = { nullptr, nullptr, nullptr, nullptr };

    StackLayerModel() = default;
    virtual ~StackLayerModel() = default;

    inline static void GetAllLeafModels(StackLayerModel * model, std::vector<StackLayerModel *> & subModels)
    {
        if(model->hasSubModels()){
            for(size_t i = 0; i < 4; ++i)
                GetAllLeafModels(model->subModels[i].get(), subModels);
        }
        else subModels.push_back(model);
    }

    inline static void CreateSubModels(StackLayerModel * model, size_t depth)
    {
        while(depth > 0){
            std::vector<StackLayerModel *> subModels;
            GetAllLeafModels(model, subModels);
            for(auto * subModel : subModels)
                subModel->CreateSubModels();
            depth--;
        }
    }

    bool hasSubModels() const { return subModels[0] != nullptr; }

    void CreateSubModels()
    {
        auto subBoxes = SplitBox();
        for(size_t i = 0; i < 4; ++i){
            subModels[i] = std::make_unique<StackLayerModel>();
            subModels[i]->bbox = subBoxes[i];
            subModels[i]->sInfos = sInfos;
        }

        if(inGeoms){
            for(size_t i = 0; i < 4; ++i){
                subModels[i]->inGeoms = std::make_shared<StackLayerPolygons>(inGeoms->size(), nullptr);
                for(auto & inGeom : *(subModels[i]->inGeoms))
                    inGeom = std::make_shared<PolygonContainer>();
            }

            for(size_t i = 0; i < inGeoms->size(); ++i){
                for(size_t j = 0; j < 4; ++j){
                    subModels[j]->inGeoms->at(i)->emplace_back(toPolygon(subBoxes[j]));
                }//boundary
                auto & polygons = *(inGeoms->at(i));
                while(!polygons.empty()){
                    bool contains = false;
                    auto & polygon = polygons.back();
                    for(size_t j = 0; j < 4; ++j){
                        if(Contains(subBoxes[j], Extent(polygon), true)){
                            subModels[j]->inGeoms->at(i)->emplace_back(std::move(polygon));
                            contains = true;
                            break;
                        }
                    }
                    if(!contains){
                        for(size_t j = 0; j < 4; ++j){
                            std::list<Polygon2D<coor_t> > results;
                            boolean::Intersect(polygon, subBoxes[j], results);
                            for(auto & result : results){
                                subModels[j]->inGeoms->at(i)->emplace_back(std::move(result));
                            }
                        }
                    }
                    polygons.pop_back();
                }
            }
            inGeoms.reset();
        }
    }

private:
    std::array<Box2D<coor_t>, 4> SplitBox() const
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
};

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