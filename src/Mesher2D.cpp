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
    res = res && RunMeshEvaluation();

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
    bool res(true);
    std::string filename = GetProjFileName();
    //
    log::Info("start to load geometries from file...");
    db.inGeoms.reset(new PolygonContainer);
    res = MeshFlow2D::LoadGeometryFiles(filename, options.iFileFormat, *db.inGeoms);
    if(!res){
        log::Error("fail to load geometry from %1%", filename);
        return false;
    }

    if(db.inGeoms->empty()){
        log::Error("no geometry load from file %1%", filename);
        return false;
    }

    //
    log::Info("start to calculate convex hull...");
    auto outline = ConvexHull(*db.inGeoms);
    // db.inGeoms->push_back(outline);//wbtest
    Box2D<coor_t> bbox = Extent(outline);
    // db.inGeoms->push_back(toPolygon(bbox));/wbtest

    //wbtest
    options.meshCtrl.minEdgeLen = std::min(bbox.Length(), bbox.Width()) / 3000;
    options.meshCtrl.maxEdgeLen = std::max(bbox.Length(), bbox.Width()) / 3;
    options.meshCtrl.minAlpha = math::Rad(15);

    //
    log::Info("start to extract intersections...");
    db.segments.reset(new Segment2DContainer);
    res = MeshFlow2D::ExtractIntersections(*db.inGeoms, *db.segments);
    if(!res){
        log::Error("fail to extract intersections!");
        return false;
    }

    db.inGeoms.reset();
    db.edges.reset(new IndexEdgeList);
    db.points.reset(new Point2DContainer);
    
    //
    log::Info("start to extract topology...");
    res = MeshFlow2D::ExtractTopology(*db.segments, *db.points, *db.edges);
    if(!res){
        log::Error("fail to extract topology!");
        return false;
    }

    db.segments.reset();
    
    //
    log::Info("start to merge closed points and remap edges...");
    res = MeshFlow2D::MergeClosePointsAndRemapEdge(*db.points, *db.edges, options.meshCtrl.tolerance);
    if(!res){
        log::Error("fail to merge closed points and remap edges!");
        return false;
    }

    // if(db.meshCtrl->maxEdgeLen > 0){
    //     log::Info("start to split overlength edges...");
    //     res = MeshFlow2D::SplitOverlengthEdges(*db.points, *db.edges, db.meshCtrl->maxEdgeLen);
    //     if(!res){
    //         log::Error("fail to split overlength edges!");
    //         return false;
    //     }
    // }

    // {
    //     log::Info("start to add grade points...");
    //     size_t threshold = 10;
    //     res = MeshFlow2D::AddPointsFromBalancedQuadTree(outline, *db.points, threshold);
    //     if(!res){
    //         log::Error("fail to add grade points!");
    //         return false;
    //     }
    // }

    db.triangulation.reset(new TriangulationData);
    
    log::Info("start to generate mesh...");
    res = MeshFlow2D::TriangulatePointsAndEdges(*db.points, *db.edges, *db.triangulation);
    if(!res){
        log::Error("fail to generate mesh!");
        return false;
    }

    db.edges.reset();
    db.points.reset();

    log::Info("start to refine mesh...");
    size_t iteration = 5000;//wbtest
    res = MeshFlow2D::TriangulationRefinement(*db.triangulation, options.meshCtrl.minAlpha, options.meshCtrl.minEdgeLen, options.meshCtrl.maxEdgeLen, iteration);
    if(!res){
        log::Error("fail to refine mesh!");
        return false;
    }
    
    log::Info("start to write mesh result...");
    res = MeshFlow2D::ExportMeshResult(filename, options.oFileFormat, *db.triangulation);
    if(!res){
        log::Error("fail to write mesh result!");
        return false;
    }
    else { log::Info("write mesh result to %1%", filename); }

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

    log::Info("start to generate mesh report...");
    std::string rptFile = GetProjFileName() + ".rpt";
    auto res = MeshFlow2D::GenerateReport(rptFile, *db.triangulation);
    if(!res){
        log::Error("fail to generate mesh report");
        return false;
    }

    log::Info("report generated: %1%", rptFile);
    return true;
}