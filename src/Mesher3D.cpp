#include "Mesher3D.h"
#include "generic/geometry/BoostGeometryRegister.hpp"
#include "generic/geometry/BoostPolygonRegister.hpp"
#include "generic/geometry/TetrahedralizationIO.hpp"
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
Mesher3D::Mesher3D()
{
    std::string workPath = filesystem::CurrentPath() + GENERIC_FOLDER_SEPS + "test" + GENERIC_FOLDER_SEPS + "subgds";
    std::string projName = "SubGDS_DIE1";
    
    db.workPath.reset(new std::string(std::move(workPath)));
    db.projName.reset(new std::string(std::move(projName)));
}

Mesher3D::~Mesher3D()
{
}

bool Mesher3D::Run()
{
    if(!db.workPath || !db.projName) return false;

    InitLogger();
    
    auto res = RunGenerateMesh();

    CloseLogger();
    return res;
}

bool Mesher3D::RunGenerateMesh()
{
    bool res(true);

    //
    log::Info("start to load geometries from file...");
    db.sInfos.reset(new StackLayerInfos);
    db.inGoems.reset(new StackLayerPolygons);
    res = MeshFlow3D::LoadGeometryFiles(*db.workPath, *db.projName, *db.inGoems, *db.sInfos);
    if(!res){
        log::Error("fail to load geometries");
        return false;
    }
    else log::Info("finish loading geometries from file");

    size_t geomCount = 0;
    geometry::Box2D<coor_t> bbox;
    for(const auto & geoms : *db.inGoems){
        geomCount += geoms.size();
        for(const auto & geom : geoms)
            bbox |= geometry::Extent(geom);
    }
    log::Info("total geometries: %1%", geomCount);

    //
    log::Info("start simplify geometries...");
    coor_t tolerance = 100;
    res = MeshFlow3D::CleanGeometries(*db.inGoems, tolerance);
    if(!res){
        log::Error("fail to simplify geometries, tolerance may be to large");
        return false;
    }
    else log::Info("finish simplifying geometries, tolerance: %1%", tolerance);

    //
    db.intersections.reset(new InterfaceIntersections);
    res = MeshFlow3D::ExtractInterfaceIntersections(*db.inGoems, *db.intersections);
    if(!res){
        log::Error("fail to extract interface intersections");
        return false;
    }
    else log::Info("finish extracting interface intersections");

    //
    db.edges.reset(new IndexEdgeList);
    db.points.reset(new Point3DContainer);
    // std::vector<StackLayerInfo> infos { {0, 3048}, {-3048, 20320}, {-23368, 3048}, {-26416, 20320}, {-46736, 3048}, {-49784, 20320}, {-70104, 3048} };

    log::Info("start to extract topology...");
    res = MeshFlow3D::ExtractTopology(*db.inGoems, *db.sInfos, *db.intersections, *db.points, *db.edges);
    if(!res){
        log::Error("fail to extract topology!");
        return false;
    }
    else log::Info("finish extracting topology");

    //
    log::Info("start to split overlength edges...");
    coor_t threshold = std::max(bbox.Length(), bbox.Width()) / 10;
    res = MeshFlow3D::SplitOverlengthEdges(*db.points, *db.edges, threshold);
    if(!res){
        log::Error("fail to split overlength edges!");
        return false;
    }
    else log::Info("finish splitting overlength edges, threshold: %1%", threshold);

    //
    std::string filename = *db.workPath + GENERIC_FOLDER_SEPS + *db.projName;
    log::Info("start to write node and edge files...");
    res = MeshFlow3D::WriteNodeAndEdgeFiles(filename, *db.points, *db.edges);
    if(!res){
        log::Error("fail to write node and edge files!");
        return false;
    }
    else log::Info("finish writing node and edge files"); 
    return true;
}

void Mesher3D::InitLogger()
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

void Mesher3D::CloseLogger()
{
    log::ShutDown();
}

bool MeshFlow3D::LoadGeometryFiles(const std::string & workPath, const std::string & projName, StackLayerPolygons & polygons, std::vector<StackLayerInfo> & infos)
{
    std::string baseName = workPath + GENERIC_FOLDER_SEPS + projName;
    std::string stackFile = baseName + "_stack.info";
    if(!LoadLayerStackInfos(stackFile, infos)) return false;

    polygons.clear();

    bool res = true;
    polygons.resize(infos.size());
    for(size_t i = 0; i < infos.size(); ++i){
        std::string filename = baseName + "_" + std::to_string(i + 1) + ".wkt";
        std::list<PolygonWithHoles2D<coor_t> > pwhs;
        res = res && MeshFileUtility::LoadWktFile(filename, pwhs);
        for(auto & pwh : pwhs){
            polygons[i].push_back(std::move(pwh.outline));
            for(auto & hole : pwh.holes)
                polygons[i].push_back(std::move(hole));
        }
    }
    return res;
}

bool MeshFlow3D::CleanGeometries(StackLayerPolygons & polygons, coor_t distance)
{
    for(auto & layer : polygons){
        for(auto & polygon : layer){
            Polygon2D<coor_t> out;
            if(polygon.Front() == polygon.Back()) polygon.PopBack();
            boost::geometry::simplify(polygon, out, distance);
            if(out.Size() < 3) return false;
            if(out.Size() != polygon.Size())
                polygon = std::move(out);
        }
    }
    return true;
}

bool MeshFlow3D::ExtractInterfaceIntersections(const StackLayerPolygons & polygons, InterfaceIntersections & intersections)
{
    intersections.clear();
    if(polygons.size() <= 1) return false;

    bool res = true;
    intersections.resize(polygons.size() - 1);
    for(size_t i = 0; i < polygons.size() -1; ++i){
        res = res && ExtractInterfaceIntersection(polygons[i], polygons[i + 1], intersections[i]);
    }
    return res;
}

bool MeshFlow3D::ExtractInterfaceIntersection(const PolygonContainer & layer1, const PolygonContainer & layer2, Segment2DContainer & intersection)
{
    intersection.clear();
    std::list<Segment2D<coor_t> > temp;
    auto toSegments = [&temp](const Polygon2D<coor_t> & polygon) mutable
    {
        size_t size = polygon.Size();
        for(size_t i = 0; i < size; ++i){
            size_t j = (i + 1) % size;
            temp.emplace_back(Segment2D<coor_t>(polygon[i], polygon[j]));
        }
    };
    for(const auto & polygon : layer1) toSegments(polygon);
    for(const auto & polygon : layer2) toSegments(polygon);
    boost::polygon::intersect_segments(intersection, temp.begin(), temp.end());
    return true;
}

bool MeshFlow3D::ExtractTopology(StackLayerPolygons & polygons, const std::vector<StackLayerInfo> & infos, const InterfaceIntersections & intersections,
                                 std::vector<Point3D<coor_t> > & points, std::list<IndexEdge> & edges)
{
    points.clear();
    edges.clear();
    if(polygons.size() != infos.size()) return false;

    using EdgeSet = topology::UndirectedIndexEdgeSet;
    using LayerIdxMap = std::unordered_map<coor_t, size_t>;
    using PointIdxMap = std::unordered_map<Point2D<coor_t>, size_t, PointHash<coor_t> >;
    using LayerPointIdxMap = std::vector<PointIdxMap>;
    
    EdgeSet edgeSet;
    std::vector<coor_t> heights(infos.size() + 1);
    LayerPointIdxMap lyrPtIdxMap(infos.size() + 1);
    size_t lyrIdx = 0;
    for(const auto & info : infos){
        heights[lyrIdx] = info.elevation;
        lyrIdx++;
    }
    heights[lyrIdx] = infos.back().elevation - infos.back().thickness;

    // for(size_t i = 0 ; i < heights.size(); ++i){
    //     heights[i] *= 100;
    // }//wbtest

    auto getIndex = [&lyrPtIdxMap, &heights, &points](const Point2D<coor_t> & p, size_t layer) mutable
    {
        auto & ptIdxMap = lyrPtIdxMap[layer];
        if(!ptIdxMap.count(p)){
            auto index = points.size();
            ptIdxMap.insert(std::make_pair(p, index));
            points.push_back(Point3D<coor_t>(p[0], p[1], heights[layer]));
        }
        return ptIdxMap.at(p);
    };

    lyrIdx = 0;
    const auto & top = polygons.front();
    for(const auto & polygon : top){
        auto size = polygon.Size();
        for(size_t i = 0; i < size; ++i){
            size_t j = (i + 1) % size;
            IndexEdge e(getIndex(polygon[i], lyrIdx), getIndex(polygon[j], lyrIdx));
            if(edgeSet.count(e)) continue;
            edgeSet.insert(e);
            edges.emplace_back(std::move(e)); 
        }
    }
    
    for(const auto & intersection : intersections){
        lyrIdx++;
        for(const auto & segment : intersection){
            IndexEdge e(getIndex(segment[0], lyrIdx), getIndex(segment[1], lyrIdx));
            if(edgeSet.count(e)) continue;
            edgeSet.insert(e);
            edges.emplace_back(std::move(e));
        }
    }

    lyrIdx++;
    const auto & bot = polygons.back();
    for(const auto & polygon : bot){
        auto size = polygon.Size();
        for(size_t i = 0; i < size; ++i){
            size_t j = (i + 1) % size;
            IndexEdge e(getIndex(polygon[i], lyrIdx), getIndex(polygon[j], lyrIdx));
            if(edgeSet.count(e)) continue;
            edgeSet.insert(e);
            edges.emplace_back(std::move(e)); 
        }
    }

    lyrIdx = 0;
    for(const auto & layer : polygons){
        for(const auto & polygon : layer){
            size_t size = polygon.Size();
            for(size_t i = 0; i < size; ++i){
                IndexEdge e(getIndex(polygon[i], lyrIdx), getIndex(polygon[i], lyrIdx + 1));
                if(edgeSet.count(e)) continue;
                edgeSet.insert(e);
                edges.emplace_back(std::move(e)); 
            }
        }
        lyrIdx++;
    }
    return true;
}

bool MeshFlow3D::SplitOverlengthEdges(std::vector<Point3D<coor_t> > & points, std::list<IndexEdge> & edges, coor_t maxLength)
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

bool MeshFlow3D::WriteNodeAndEdgeFiles(const std::string & filename, const std::vector<Point3D<coor_t> > & points, const std::list<IndexEdge> & edges)
{

    geometry::tet::PiecewiseLinearComplex<Point3D<coor_t> >  plc;
    plc.points = points;
    plc.surfaces.resize(edges.size());
    size_t index = 0;

    using IdxPolyline = typename geometry::tet::PiecewiseLinearComplex<Point3D<coor_t> >::IdxPolyLine;
    for(const auto & edge : edges){
        plc.surfaces[index].faces.push_back(IdxPolyline{ edge.v1(), edge.v2()});
        index++;
    }
    // std::string plcFile = filename + ".poly";
    // geometry::tet::WritePlcToPolyFile(plcFile, plc);//wbtest
    return geometry::tet::WritePlcToNodeAndEdgeFiles(filename, plc);
}

bool MeshFlow3D::LoadLayerStackInfos(const std::string & filename, StackLayerInfos & infos)
{
    return MeshFileUtility::LoadLayerStackInfos(filename, infos);
}

