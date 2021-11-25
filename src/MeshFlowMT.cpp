#include "MeshFlowMT.h"
#include "generic/thread/ThreadPool.hpp"
using namespace generic;
using namespace emesh;

bool MeshFlow3DMT::CleanGeometries(StackLayerPolygons & polygons, coor_t distance, size_t threads)
{
    thread::ThreadPool pool(threads);
    std::vector<std::future<bool> > futures(polygons.size());
    for(size_t i = 0; i < polygons.size(); ++i)
        futures[i] = pool.Submit(std::bind(&MeshFlow3D::CleanLayerGeometries, std::ref(polygons[i]), distance));        
    
    bool res = true;
    for(size_t i = 0; i < futures.size(); ++i)
        res = res && futures[i].get();

    return res;
}

bool MeshFlow3DMT::ExtractInterfaceIntersections(const StackLayerPolygons & polygons, InterfaceIntersections & intersections, size_t threads)
{
    intersections.clear();
    if(polygons.size() <= 1) return false;

    auto size = polygons.size() - 1;
    intersections.resize(size);

    thread::ThreadPool pool(threads);
    std::vector<std::future<bool> > futures(size);
    for(size_t i = 0; i < size; ++i){
        futures[i] = pool.Submit(std::bind(&MeshFlow3D::ExtractInterfaceIntersection,
                                 std::ref(polygons[i]), std::ref(polygons[i + 1]), std::ref(intersections[i])));        
    }      
    
    bool res = true;
    for(size_t i = 0; i < futures.size(); ++i)
        res = res && futures[i].get();

    return res;
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

bool MeshFlow3DMT::AddGradePointsForMeshLayers(MeshSketchLayers3D & meshSktLyrs, size_t threshold, size_t threads)
{
    thread::ThreadPool pool(threads);
    std::vector<std::future<bool> > futures(meshSktLyrs.size());
    for(size_t i = 0; i < meshSktLyrs.size(); ++i){
        futures[i] = pool.Submit(std::bind(&MeshFlow3D::AddGradePointsForMeshLayer, std::ref(meshSktLyrs[i]), threshold));
    }

    bool res = true;
    for(size_t i = 0; i < futures.size(); ++i)
        res = res && futures[i].get();

    return res;
}

bool MeshFlow3DMT::GenerateTetrahedronsFromSketchLayers(const MeshSketchLayers3D & meshSktLyrs, TetrahedronDataVec & tetVec, size_t threads)
{
    thread::ThreadPool pool(threads);
    
    tetVec.resize(meshSktLyrs.size());
    std::vector<std::future<bool> > futures(meshSktLyrs.size());
    for(size_t i = 0; i < meshSktLyrs.size(); ++i){
        futures[i] = pool.Submit(std::bind(&MeshFlow3D::GenerateTetrahedronsFromSketchLayer,
                                 std::ref(meshSktLyrs[i]), std::ref(tetVec[i])));
    }    

    bool res = true;
    for(size_t i = 0; i < futures.size(); ++i)
        res = res && futures[i].get();

    return res;
}
