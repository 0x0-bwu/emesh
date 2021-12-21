#include "MeshFlowMT.h"
#include "generic/thread/ThreadPool.hpp"
using namespace generic;
using namespace emesh;

bool MeshFlow3DMT::CleanGeometries(StackLayerPolygons & polygons, coor_t distance, size_t threads)
{
    thread::ThreadPool pool(threads);
    for(size_t i = 0; i < polygons.size(); ++i)
        pool.Submit(std::bind(&MeshFlow3D::CleanLayerGeometries, std::ref(*polygons[i]), distance));        
    
    return true;
}

bool MeshFlow3DMT::ExtractModelsIntersections(std::vector<StackLayerModel * > & models, size_t threads)
{
    thread::ThreadPool pool(threads);
    for(size_t i = 0; i < models.size(); ++i)
        pool.Submit(std::bind(&MeshFlow3D::ExtractModelIntersections, std::ref(*(models[i]))));
    return true;
}

// bool MeshFlow3DMT::ExtractInterfaceIntersections(StackLayerModel & model, size_t threads)
// {
//     if(model.hasSubModels()){
//         bool res = true;
//         for(size_t i = 0; i < 4; ++i)
//             res = res && ExtractInterfaceIntersections((*model.subModels[i]), threads);
//         return res;
//     }
//     else{
//         if(nullptr == model.inGeoms) return false;
//         model.intersections.reset(new InterfaceIntersections);
//         return ExtractInterfaceIntersections(*model.inGeoms, *model.intersections, threads);
//     }
// }

// bool MeshFlow3DMT::ExtractInterfaceIntersections(const StackLayerPolygons & polygons, InterfaceIntersections & intersections, size_t threads)
// {
//     intersections.clear();
//     if(polygons.size() <= 1) return false;

//     auto size = polygons.size() - 1;
//     intersections.resize(size);

//     thread::ThreadPool pool(threads);
//     std::vector<std::future<bool> > futures(size);
//     for(size_t i = 0; i < size; ++i){
//         futures[i] = pool.Submit(std::bind(&MeshFlow3D::ExtractInterfaceIntersection,
//                                  std::ref(polygons[i]), std::ref(polygons[i + 1]), std::ref(intersections[i])));        
//     }      
    
//     bool res = true;
//     for(size_t i = 0; i < futures.size(); ++i)
//         res = res && futures[i].get();

//     return res;
// }

bool MeshFlow3DMT::SplitOverlengthEdges(const StackLayerModel & model, coor_t maxLength, size_t threads)
{
    if(model.hasSubModels()){
        bool res = true;
        for(size_t i = 0; i < 4; ++i)
            res = res && SplitOverlengthEdges((*model.subModels[i]), maxLength, threads);
        return res;
    }
    else return SplitOverlengthEdges(*model.inGeoms, *model.intersections, maxLength, threads);
}

bool MeshFlow3DMT::SplitOverlengthEdges(StackLayerPolygons & polygons, InterfaceIntersections & intersections, coor_t maxLength, size_t threads)
{
    if(0 == maxLength) return true;
    thread::ThreadPool pool(threads);

    pool.Submit(std::bind(&MeshFlow3D::SplitOverlengthPolygons, std::ref(polygons.front()), maxLength));
    pool.Submit(std::bind(&MeshFlow3D::SplitOverlengthPolygons, std::ref(polygons.back()), maxLength));

    for(size_t i = 0; i < intersections.size(); ++i){
        pool.Submit(std::bind(&MeshFlow3D::SplitOverlengthSegments, std::ref(intersections[i]), maxLength));
    }        
    return true;
}

bool MeshFlow3DMT::GenerateTetrahedronVecFromSketchModels(std::vector<MeshSketchModel> & models, TetrahedronDataVec & tetVec, size_t threads)
{
    auto size = models.size();
    tetVec.resize(size);
    thread::ThreadPool pool(threads);
    for(size_t i = 0; i < size; ++i)
        pool.Submit(std::bind(&MeshFlow3D::GenerateTetrahedronDataFromSketchModel, std::ref(models[i]), std::ref(tetVec[i])));        
    
    return true;
}


// bool MeshFlow3DMT::AddGradePointsForMeshModel(MeshSketchModel & model, size_t threshold, size_t threads)
// {
//     thread::ThreadPool pool(threads);
//     std::vector<std::future<bool> > futures(model.layers.size());
//     for(size_t i = 0; i < model.layers.size(); ++i){
//         futures[i] = pool.Submit(std::bind(&MeshFlow3D::AddGradePointsForMeshLayer, std::ref(model.layers[i]), threshold));
//     }

//     bool res = true;
//     for(size_t i = 0; i < futures.size(); ++i)
//         res = res && futures[i].get();

//     return res;
// }

// bool MeshFlow3DMT::GenerateTetrahedronsFromSketchModel(const MeshSketchModel & model, TetrahedronDataVec & tetVec, size_t threads)
// {
//     thread::ThreadPool pool(threads);
    
//     tetVec.resize(model.layers.size());
//     std::vector<std::future<bool> > futures(model.layers.size());
//     for(size_t i = 0; i < model.layers.size(); ++i){
//         futures[i] = pool.Submit(std::bind(&MeshFlow3D::GenerateTetrahedronsFromSketchLayer,
//                                  std::ref(model.layers[i]), std::ref(tetVec[i])));
//     }    

//     bool res = true;
//     for(size_t i = 0; i < futures.size(); ++i)
//         res = res && futures[i].get();

//     return res;
// }
