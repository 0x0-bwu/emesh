#include "Mesher2D.h"
#include "generic/tools/FileSystem.hpp"
#include "generic/tools/Log.hpp"
#include "MeshFlow2D.h"
#include "MeshIO.h"
using namespace generic;
using namespace emesh;
bool Mesher2D::Run()
{ 
    if(GetProjFileName().empty()) return false;

    InitLogger();
    
    bool res = true;
    res = res && RunGenerateMesh();
    //res = res && RunMeshEvaluation();

    CloseLogger();
    return res;
}

bool Mesher2D::RunTest()
{
    return false;
}

std::string Mesher2D::GetProjFileName() const
{
    if(options.workPath.empty() || options.projName.empty()) return std::string{};
    return options.workPath + GENERIC_FOLDER_SEPS + options.projName;
}

bool Mesher2D::RunGenerateMesh()
{
    bool res = true;
    std::string filename = GetProjFileName();
    size_t threads = std::max<size_t>(1, options.threads);
    log::Info("2d mesh config:");
    log::Info("path: %1%", filename);
    log::Info("threads: %1%", threads);

    //
    log::Info("start load geometries from file...");
    db.inGeoms.reset(new PolygonContainer);
    res = MeshFlow2D::LoadGeometryFiles(filename, options.iFileFormat, *db.inGeoms);
    if(!res){
        log::Error("failed to load geometry from input file");
        return false;
    }

    if(db.inGeoms->empty()){
        log::Error("no geometry loaded from input file");
        return false;
    }
    else{
        log::Info("total input geometries: %1%", db.inGeoms->size());
    }

    //
    log::Info("start calculate convex hull...");
    auto outline = ConvexHull(*db.inGeoms);
    // db.inGeoms->push_back(outline);//wbtest
    Box2D<coor_t> bbox = Extent(outline);


    coor_t maxLength = std::max(bbox.Length(), bbox.Width()) / 10;
    coor_t minLength = std::min(bbox.Length(), bbox.Width()) / 1000;
    options.meshCtrl.maxEdgeLen = std::min(maxLength, options.meshCtrl.maxEdgeLen);
    options.meshCtrl.maxEdgeLen = std::max(minLength, options.meshCtrl.maxEdgeLen);
    options.meshCtrl.minEdgeLen = minLength;

    //
    log::Info("start extract intersections...");
    db.segments.reset(new Segment2DContainer);
    res = MeshFlow2D::ExtractIntersections(*db.inGeoms, *db.segments);
    if(!res){
        log::Error("failed to extract intersections!");
        return false;
    }

    db.inGeoms.reset();
    db.edges.reset(new IndexEdgeList);
    db.points.reset(new Point2DContainer);
    
    coor_t tolerance = 0;//wbtest
    log::Info("start merge near edges..., tolerance: %1%", tolerance);
    res = MeshFlow2D::MergeClosestParallelSegmentsIteratively(*db.segments, tolerance);
    if(!res){
        log::Error("failed to merge near edges!");
        return false;
    }
    
    //
    log::Info("start extract connection topology...");
    res = MeshFlow2D::ExtractTopology(*db.segments, *db.points, *db.edges);
    if(!res){
        log::Error("failed to extract connection topology!");
        return false;
    }

    db.segments.reset();
    
    //
    if(options.meshCtrl.tolerance > 0){
        log::Info("start merge near points and remap edges..., tolerance: %1%", options.meshCtrl.tolerance);
        res = MeshFlow2D::MergeClosePointsAndRemapEdge(*db.points, *db.edges, options.meshCtrl.tolerance);
        if(!res){
            log::Error("failed to merge closed points and remap edges!");
            return false;
        }
    }

    if(options.meshCtrl.maxEdgeLen > 0){
        log::Info("start split overlength edges..., length: %1%", options.meshCtrl.maxEdgeLen);
        res = MeshFlow2D::SplitOverlengthEdges(*db.points, *db.edges, options.meshCtrl.maxEdgeLen);
        if(!res){
            log::Error("failed to split overlength edges!");
            return false;
        }
    }

    if(options.maxGradeLvl > 0){
        size_t size = db.points->size();
        log::Info("grade points option enabled, input geometries should have only one closure!");
        log::Info("start insert grade points..., level: %1%", options.maxGradeLvl);
        res = MeshFlow2D::AddPointsFromBalancedQuadTree(*db.points, *db.edges, options.maxGradeLvl);
        if(!res){
            log::Error("failed to insert grade points!");
            return false;
        }
        else{
            log::Info("total added grade points: %1%", db.points->size() - size);
        }
    }
    
    log::Info("start generate mesh...");
    db.triangulation.reset(new TriangulationData);
    res = MeshFlow2D::TriangulatePointsAndEdges(*db.points, *db.edges, *db.triangulation);
    if(!res){
        log::Error("failed to generate mesh!");
        return false;
    }
    log::Info("total nodes: %1%", db.triangulation->vertices.size());
    log::Info("total elements: %1%", db.triangulation->triangles.size());

    db.edges.reset();
    db.points.reset();

    if(options.meshCtrl.refineIte > 0){
        log::Info("start refine mesh...");
        MeshFlow2D::TriangulationRefinement(*db.triangulation, options.meshCtrl);
    }

    log::Info("start write mesh result..., output file format: %1%", toString(options.oFileFormat));
    res = MeshFlow2D::ExportMeshResult(filename, options.oFileFormat, *db.triangulation);
    if(!res){
        log::Error("failed to write mesh result!");
        return false;
    }
    return true;
}

bool Mesher2D::RunMeshEvaluation()
{
    if(!db.triangulation){
        if(options.iFileFormat == FileFormat::MSH){
            std::string mshFile = GetProjFileName() + ".msh";
            log::Info("loading mesh data from %1%", mshFile);
            if(!filesystem::FileExists(mshFile)){
                log::Error("file %1% not exist!", mshFile);
                return false;
            }

            db.triangulation.reset(new TriangulationData);
            auto res = io::ImportMshFile(mshFile, *db.triangulation);
            if(!res){
                log::Error("fail to load mesh from %1%", mshFile);
                return false;
            }
        }
    }

    if(!db.triangulation){
        log::Fatal("no input mesh data!");
        return false;
    }

    log::Info("start generate mesh report...");
    std::string rptFile = GetProjFileName() + ".rpt";
    auto res = MeshFlow2D::GenerateReport(rptFile, *db.triangulation);
    if(!res){
        log::Error("failed to generate mesh report");
        return false;
    }

    log::Info("report generated: %1%", rptFile);
    return true;
}