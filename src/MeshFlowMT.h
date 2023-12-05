#pragma once
#include "generic/thread/ThreadPool.hpp"
#include "MeshFlow3D.h"
#include <limits>
namespace emesh {

inline constexpr size_t maxThreads = std::numeric_limits<size_t>::max();
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