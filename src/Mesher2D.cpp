#include "Mesher2D.h"
#include "generic/geometry/TriangulationRefinement.hpp"
#include "generic/geometry/BoostPolygonRegister.hpp"
#include "generic/tree/QuadTreeUtilityMT.hpp"
#include "generic/geometry/HashFunction.hpp"
#include "generic/geometry/Transform.hpp"
#include "generic/geometry/Topology.hpp"
#include "generic/geometry/Utility.hpp"
#include "generic/tools/FileSystem.hpp"
#include "generic/tools/Tools.hpp"
#include "MeshFileUtility.h"
#include <memory>
#include <ctime>
using namespace generic;
using namespace emesh;
Mesher2D::Mesher2D()
{
}

Mesher2D::~Mesher2D()
{
}

bool Mesher2D::Run()
{
    if(!db.workPath || !db.projName) return false;
    
    InitLogger();
    
    auto res = RunTasks();

    CloseLogger();
    return res;
}

bool Mesher2D::RunTasks()
{
    if(!db.tasks) {
        log::Info("no task need to do!");
        return false;
    }

    bool res(false);
    while(!db.tasks->empty()){
        auto task = db.tasks->front();
        db.tasks->pop();
        if(task == MeshTask::MeshGeneration) res = RunGenerateMesh();
        if(task == MeshTask::MeshEvaluation) res = RunMeshEvaluation();
        if(!res) return false;
    }
    return true;
}

bool Mesher2D::RunGenerateMesh()
{
    bool res(true);
    //
    std::string ctrlFile = *db.workPath + GENERIC_FOLDER_SEPS + "mesh_input.txt";
    if(filesystem::FileExists(ctrlFile)){
        log::Info("find input mesh ctrl file %1%", ctrlFile);

        res = MeshFileUtility::LoadMeshCtrlFile(ctrlFile, *db.meshCtrl);
        if(res){ log::Info("load mesh ctrl file successfully!"); }
        else { log::Info("fail to load paras from mesh ctrl file, use default paras instead."); }
    }

    //
    log::Info("start to load geometries from file...");
    db.inGoems.reset(new PolygonContainer);
    std::string filename = *db.workPath + GENERIC_FOLDER_SEPS + *db.projName;
    res = MeshFlow2D::LoadGeometryFiles(filename, *db.inFormat, db.meshCtrl->scale2Int, *db.inGoems);
    if(!res){
        log::Error("fail to load geometry from %1%", filename);
        return false;
    }

    if(db.inGoems->empty()){
        log::Error("no geometry load from file %1%", filename);
        return false;
    }

    //
    log::Info("start to calculate convex hull...");
    auto outline = ConvexHull(*db.inGoems);
    // db.inGoems->push_back(outline);//wbtest
    Box2D<coor_t> bbox = Extent(outline);
    // db.inGoems->push_back(toPolygon(bbox));/wbtest

    //wbtest
    db.meshCtrl->minEdgeLen = std::min(bbox.Length(), bbox.Width()) / 3000;
    db.meshCtrl->maxEdgeLen = std::max(bbox.Length(), bbox.Width()) / 3;
    db.meshCtrl->minAlpha = math::Rad(15);

    //
    log::Info("start to extract intersections...");
    db.segments.reset(new Segment2DContainer);
    res = MeshFlow2D::ExtractIntersections(*db.inGoems, *db.segments);
    if(!res){
        log::Error("fail to extract intersections!");
        return false;
    }

    db.inGoems.reset();
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
    res = MeshFlow2D::MergeClosePointsAndRemapEdge(*db.points, *db.edges, db.meshCtrl->tolerance);
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
    res = MeshFlow2D::TriangulationRefinement(*db.triangulation, db.meshCtrl->minAlpha, db.meshCtrl->minEdgeLen, db.meshCtrl->maxEdgeLen, iteration);
    if(!res){
        log::Error("fail to refine mesh!");
        return false;
    }
    
    log::Info("start to write mesh result...");
    res = MeshFlow2D::ExportMeshResult(filename, FileFormat::MSH, *db.triangulation, 1.0 / db.meshCtrl->scale2Int);
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
        if(*db.inFormat == FileFormat::MSH){
            std::string mshFile = *db.workPath + GENERIC_FOLDER_SEPS + *db.projName + ".msh";
            log::Info("loading mesh data from %1%", mshFile);
            if(!filesystem::FileExists(mshFile)){
                log::Error("file %1% not exist!", mshFile);
                return false;
            }

            db.triangulation.reset(new TriangulationData);
            auto res = MeshFileUtility::ImportMshFile(mshFile, *db.triangulation, db.meshCtrl->scale2Int);
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
    std::string filename = *db.workPath + GENERIC_FOLDER_SEPS + *db.projName + ".rpt";
    auto res = MeshFlow2D::GenerateReport(filename, *db.triangulation, 1.0 / db.meshCtrl->scale2Int);
    if(!res){
        log::Error("fail to generate mesh report");
        return false;
    }

    log::Info("report generated: %1%", filename);
    return true;
}

void Mesher2D::InitLogger()
{
    std::string dbgFile = *db.workPath + GENERIC_FOLDER_SEPS + *db.projName + ".dbg";
    std::string logFile = *db.workPath + GENERIC_FOLDER_SEPS + *db.projName + ".log";

    auto traceSink = std::make_shared<log::StreamSinkMT>(std::cout);
    auto debugSink = std::make_shared<log::FileSinkMT>(dbgFile);
    auto infoSink  = std::make_shared<log::FileSinkMT>(logFile);
    traceSink->SetLevel(log::Level::Trace);
    debugSink->SetLevel(log::Level::Debug);
    infoSink->SetLevel(log::Level::Info);

    auto logger = log::MultiSinksLogger("Mesh Log", {traceSink, debugSink, infoSink});
    logger->SetLevel(log::Level::Trace);
    log::SetDefaultLogger(logger);
}

void Mesher2D::CloseLogger()
{
    log::ShutDown();
}

bool MeshFlow2D::LoadGeometryFiles(const std::string & filename, FileFormat format, float_t scale2Int, std::list<Polygon2D<coor_t> > & polygons)
{
    try {
        switch (format) {
            case FileFormat::DomDmc : {
                std::string dom = filename + ".dom";
                std::string dmc = filename + ".dmc";
                return MeshFileUtility::LoadDomDmcFiles(dom, dmc, scale2Int, polygons);
            }
            case FileFormat::WKT : {
                std::string wkt = filename + ".wkt";
                return MeshFileUtility::LoadWktFile(wkt, scale2Int, polygons);
            }
            default : return false;
        }
    }
    catch (...) { return false; }
    return true;
}

bool MeshFlow2D::ExtractIntersections(const std::list<Polygon2D<coor_t> > & polygons, std::vector<Segment2D<coor_t> > & segments)
{
    segments.clear();
    std::list<Segment2D<coor_t> > temp;
    for(const auto & polygon : polygons){
        size_t size = polygon.Size();
        for(size_t i = 0; i < size; ++i){
            size_t j = (i + 1) % size;
            temp.emplace_back(Segment2D<coor_t>(polygon[i], polygon[j]));
        }
    }
    boost::polygon::intersect_segments(segments, temp.begin(), temp.end());
    return true;
}

bool MeshFlow2D::ExtractTopology(const std::vector<Segment2D<coor_t> > & segments, std::vector<Point2D<coor_t> > & points, std::list<IndexEdge> & edges)
{
    points.clear();
    edges.clear();
    using EdgeSet = topology::UndirectedIndexEdgeSet;
    using PointIdxMap = std::unordered_map<Point2D<coor_t>, size_t, PointHash<coor_t> >;
    
    EdgeSet edgeSet;
    PointIdxMap pointIdxMap;

    auto getIndex = [&pointIdxMap, &points](const Point2D<coor_t> & p) mutable
    {
        if(!pointIdxMap.count(p)){
            pointIdxMap.insert(std::make_pair(p, points.size()));
            points.push_back(p);
        }
        return pointIdxMap.at(p);
    };

    points.reserve(2 * segments.size());
    for(const auto & segment : segments){
        IndexEdge e(getIndex(segment[0]), getIndex(segment[1]));
        if(edgeSet.count(e)) continue;
        edgeSet.insert(e);
        edges.emplace_back(std::move(e));
    }
    return true;   
}

bool MeshFlow2D::MergeClosePointsAndRemapEdge(std::vector<Point2D<coor_t> > & points, std::list<IndexEdge> & edges, coor_t tolerance)
{
    if(tolerance > 0) RemoveDuplicatesAndRemapEdges(points, edges, tolerance);
    return true;
}

bool MeshFlow2D::SplitOverlengthEdges(std::vector<Point2D<coor_t> > & points, std::list<IndexEdge> & edges, coor_t maxLength)
{
    if(0 == maxLength) return true;
    coor_t maxLenSq = maxLength * maxLength;

    auto lenSq = [&points](const IndexEdge & e) { return DistanceSq(points[e.v1()], points[e.v2()]); };
    auto split = [&points](const IndexEdge & e) mutable
    {
        size_t index = points.size();
        points.push_back((points[e.v1()] + points[e.v2()]) * 0.5);
        return std::make_pair(IndexEdge(e.v1(), index), IndexEdge(index, e.v2()));
    };

    std::list<IndexEdge> tmp;
    while(edges.size()){
        IndexEdge e = edges.front();
        edges.pop_front();
        if(maxLenSq < lenSq(e)){
            auto added = split(e);
            edges.emplace_front(std::move(added.first));
            edges.emplace_front(std::move(added.second));
        }
        else tmp.emplace_back(std::move(e));
    }

    std::swap(edges, tmp);
    return true;
}

bool MeshFlow2D::AddPointsFromBalancedQuadTree(const Polygon2D<coor_t> & outline, std::vector<Point2D<coor_t> > & points, size_t threshold)
{

    struct PointExtent
    {
        Box2D<coor_t> operator()(const Point2D<coor_t> & point) const
        {
            return Box2D<coor_t>(point, point);
        }
    };
    
    std::list<Point2D<coor_t> * > objs;
    for(size_t i = 0; i < points.size(); ++i)
        objs.push_back(&points[i]);
    
    threshold = std::max(size_t(1), threshold);
    Box2D<coor_t> bbox = Extent(outline);
    using Tree = tree::QuadTree<coor_t, Point2D<coor_t>, PointExtent>;
    using Node = typename Tree::QuadNode;
    using TreeBuilder = tree::QuadTreeBuilderMT<Point2D<coor_t>, Tree>;
    Tree tree(bbox);
    TreeBuilder builder(tree);
    builder.Build(objs, threshold);

    tree.Balance();

    std::list<Node * > leafNodes;
    Tree::GetAllLeafNodes(&tree, leafNodes);

    for(auto node : leafNodes){
        if(node->GetObjs().size() > 0) continue;
        const auto & box = node->GetBBox();
        Point2D<coor_t> ct = box.Center().Cast<coor_t>();
        if(Contains(outline, ct))
            points.emplace_back(std::move(ct));
    }
    return true;
}

bool MeshFlow2D::TriangulatePointsAndEdges(const std::vector<Point2D<coor_t> > & points, const std::list<IndexEdge> & edges, Triangulation<Point2D<coor_t> > & tri)
{
    tri.Clear();
    try {
        Triangulator2D<coor_t> triangulator(tri);
        triangulator.InsertVertices(points.begin(), points.end(), [](const Point2D<coor_t> & p){ return p[0]; }, [](const Point2D<coor_t> & p){ return p[1]; });
        triangulator.InsertEdges(edges.begin(), edges.end(), [](const IndexEdge & e){ return e.v1(); }, [](const IndexEdge & e){ return e.v2(); });
        triangulator.EraseOuterTriangles();
    }
    catch (...) {
        tri.Clear();
        return false;
    }
    return true;
}

bool MeshFlow2D::TriangulationRefinement(Triangulation<Point2D<coor_t> > & triangulation, float_t minAlpha, coor_t minLen, coor_t maxLen, size_t iteration)
{
    // ChewSecondRefinement2D<coor_t> refinement(triangulation);
    JonathanRefinement2D<coor_t> refinement(triangulation);
    refinement.SetParas(minAlpha, minLen, maxLen);
    refinement.Refine(iteration);
    refinement.ReallocateTriangulation();
    return true;
}

bool MeshFlow2D::ExportMeshResult(const std::string & filename, FileFormat format, const Triangulation<Point2D<coor_t> > & triangulation, float_t scale)
{
    switch (format) {
        case FileFormat::MSH : {
            std::string msh = filename + ".msh";
            return MeshFileUtility::ExportMshFile(msh, triangulation, scale);
        }
        default : return false;
    }
}

bool MeshFlow2D::GenerateReport(const std::string & filename, const Triangulation<Point2D<coor_t> > & triangulation, float_t scale)
{
    TriangleEvaluator<Point2D<coor_t> > evaluator(triangulation);
    auto evaluation = evaluator.Report();
    return MeshFileUtility::ExportReportFile(filename, evaluation, scale, false);
}