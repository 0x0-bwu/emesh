#include "MeshFlow3D.h"
#include "generic/geometry/TetrahedralizationIO.hpp"
#include "generic/geometry/Utility.hpp"
#include "generic/tree/QuadTreeUtilityMT.hpp"
#include "Tetrahedralizator.h"
#include "MeshFileUtility.h"
using namespace generic;
using namespace emesh;

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
    bool res = true;
    for(auto & layer : polygons){
        res = res && CleanLayerGeometries(layer, distance);
    }
    return res;
}

bool MeshFlow3D::CleanLayerGeometries(PolygonContainer & polygons, coor_t distance)
{
    for(auto & polygon : polygons){
        Polygon2D<coor_t> out;
        if(polygon.Front() == polygon.Back()) polygon.PopBack();
        boost::geometry::simplify(polygon, out, distance);
        if(out.Size() < 3) return false;
        if(out.Size() != polygon.Size())
            polygon = std::move(out);
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

bool MeshFlow3D::SplitOverlengthEdges(StackLayerPolygons & polygons, InterfaceIntersections & intersections, coor_t maxLength)
{
    SplitOverlengthIntersections(intersections, maxLength);
    //for stacklayer polygons, only need split top and bot
    SplitOverlengthPolygons(polygons.front(), maxLength);
    SplitOverlengthPolygons(polygons.back(), maxLength);
    return true;
}

bool MeshFlow3D::BuildMeshSketchLayers(const StackLayerPolygons & polygons, const InterfaceIntersections & intersections, const StackLayerInfos & infos, MeshSketchLayers3D & meshSktLyrs)
{
    if(polygons.size() != (intersections.size() + 1)) return false;
    if(polygons.size() != infos.size()) return false;

    meshSktLyrs.clear();
    for(size_t i = 0; i < polygons.size(); ++i){
        MeshSketchLayer3D layer(polygons[i]);
        layer.SetTopBotHeight(infos[i].elevation, infos[i].elevation - infos[i].thickness);
        if(i == 0) layer.SetConstrains(nullptr, &(intersections[i]));
        else if(i == (polygons.size() - 1)) layer.SetConstrains(&(intersections[i - 1]), nullptr);
        else layer.SetConstrains(&(intersections[i - 1]), &(intersections[i]));
        meshSktLyrs.emplace_back(std::move(layer));
    }
    return true;
}

bool MeshFlow3D::AddGradePointsForMeshLayers(MeshSketchLayers3D & meshSktLyrs, size_t threshold)
{
    bool res = true;
    for(size_t i = 0; i < meshSktLyrs.size(); ++i)
        res = res && AddGradePointsForMeshLayer(meshSktLyrs[i], threshold);
    return res;
}

bool MeshFlow3D::AddGradePointsForMeshLayer(MeshSketchLayer3D & meshSktLyr, size_t threshold)
{
    for(size_t i = 0; i < 2; ++i){
        if(meshSktLyr.addPoints[i]) continue;
        if(meshSktLyr.constrains[i])
            meshSktLyr.addPoints[i] = AddPointsFromBalancedQuadTree(*(meshSktLyr.constrains[i]), threshold);
        else meshSktLyr.addPoints[i] = AddPointsFromBalancedQuadTree(meshSktLyr.polygons, threshold);
    }
}

bool MeshFlow3D::SliceOverheightLayers(MeshSketchLayers3D & meshSktLyrs, float_t ratio)
{
    std::list<MeshSketchLayer3D> layers(meshSktLyrs.begin(), meshSktLyrs.end());
    bool sliced = true;
    while(sliced){
        sliced = SliceOverheightLayers(layers, ratio);
    }
    meshSktLyrs.clear();
    meshSktLyrs.reserve(layers.size());
    for(auto & layer : layers)
        meshSktLyrs.emplace_back(std::move(layer));
    return true;
}

bool MeshFlow3D::GenerateTetrahedronsFromSketchLayers(const MeshSketchLayers3D & meshSktLyrs, TetrahedronDataVec & tetVec)
{
    bool res = true;
    tetVec.resize(meshSktLyrs.size());
    for(size_t i = 0; i < meshSktLyrs.size(); ++i)
        res = res && GenerateTetrahedronsFromSketchLayer(meshSktLyrs[i], tetVec[i]);
    return res;
}

bool MeshFlow3D::GenerateTetrahedronsFromSketchLayer(const MeshSketchLayer3D & meshSktLyr, TetrahedronData & tet)
{
    auto edges = std::make_unique<std::list<IndexEdge> >();
    auto points = std::make_unique<std::vector<Point3D<coor_t> > >();
    if(!ExtractTopology(meshSktLyr, *points, *edges)) return false;

    // auto bbox = Extent(points->begin(), points->end());
    // coor_t threshold = std::max(bbox.Length(), bbox.Width()) / 10;//wbtest
    // if(!SplitOverlengthEdges(*points, *edges, threshold)) return false;

    auto addin = meshSktLyr.GetAdditionalPoints();
    if(addin){
        points->reserve(points->size() + addin->size());
        points->insert(points->end(), addin->begin(), addin->end());
    }

    if(!Tetrahedralize(*points, *edges, nullptr, tet)) return false;

    return true;
}

bool MeshFlow3D::ExtractTopology(const MeshSketchLayer3D & meshSktLyr, Point3DContainer & points, std::list<IndexEdge> & edges)
{
    edges.clear(); 
    points.clear();

    using EdgeSet = topology::UndirectedIndexEdgeSet;
    using LayerIdxMap = std::unordered_map<coor_t, size_t>;
    using PointIdxMap = std::unordered_map<Point2D<coor_t>, size_t, PointHash<coor_t> >;

    EdgeSet edgeSet;
    PointIdxMap ptIdxMap[2];

    auto getH = [&meshSktLyr](size_t layer){ return layer == 0 ? meshSktLyr.topH : meshSktLyr.botH; };//layer: 0-top, 1-bot
    auto getIndex = [&getH, &ptIdxMap, &points](const Point2D<coor_t> & p, size_t layer) mutable //layer: 0-top, 1-bot
    {
        if(!ptIdxMap[layer].count(p)){
            auto index = points.size();
            ptIdxMap[layer].insert(std::make_pair(p, index));
            points.push_back(Point3D<coor_t>(p[0], p[1], getH(layer)));
        }
        return ptIdxMap[layer].at(p);
    };

    auto addEdge = [&edges, &edgeSet](IndexEdge && e) mutable
    {
        if(e.v1() == e.v2()) return;
        if(edgeSet.count(e)) return;
        edges.emplace_back(e);
        edgeSet.insert(e);
    };

    for(size_t i = 0; i < 2; ++i){
        if(meshSktLyr.constrains[i]){
            for(const auto & segment : *(meshSktLyr.constrains[i])){
                IndexEdge e(getIndex(segment[0], i), getIndex(segment[1], i));
                addEdge(std::move(e));
            }
        }
        else{
            for(const auto & polygon : meshSktLyr.polygons){
                size_t size = polygon.Size();
                 for(size_t j = 0; j < size; ++j){
                    size_t k = (j + 1) % size;
                    IndexEdge e(getIndex(polygon[j], i), getIndex(polygon[k], i));
                    addEdge(std::move(e));
                }
            } 
        }
    }

    for(const auto & polygon : meshSktLyr.polygons){
        size_t size = polygon.Size();
        for(size_t i = 0; i < size; ++i){
            IndexEdge e(getIndex(polygon[i], 0), getIndex(polygon[i], 1));
            addEdge(std::move(e));
        }
    }
    return true;
}

bool MeshFlow3D::SplitOverlengthEdges(Point3DContainer & points, std::list<IndexEdge> & edges, coor_t maxLength, bool surfaceOnly)
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
        if(surfaceOnly){
            const auto & p1 = points[e.v1()];
            const auto & p2 = points[e.v2()];
            if(p1[0] == p2[0] && p1[1] == p2[1])
                continue;
        }
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

bool MeshFlow3D::WriteNodeAndEdgeFiles(const std::string & filename, const Point3DContainer & points, const std::list<IndexEdge> & edges)
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
    return geometry::tet::WritePlcToNodeAndEdgeFiles(filename, plc);
}

bool MeshFlow3D::LoadLayerStackInfos(const std::string & filename, StackLayerInfos & infos)
{
    return MeshFileUtility::LoadLayerStackInfos(filename, infos);
}

bool MeshFlow3D::Tetrahedralize(const Point3DContainer & points, const std::list<IndexEdge> & edges, const Point3DContainer * addin, TetrahedronData & t)
{
    if(addin){
        std::cout << "additional points: " << addin->size() << std::endl;//wbtest
    }
    Tetrahedralizator tetrahedralizator(t);
    tetrahedralizator.Tetrahedralize(points, edges, addin);
}

bool MeshFlow3D::MergeTetrahedrons(TetrahedronData & master, TetrahedronDataVec & tetVec)
{
    size_t count = 0;
    TetrahedronDataMerger merger(master);
    for(size_t i = 0; i < tetVec.size(); ++i){
        count += tetVec[i].tetrahedrons.size();
        merger.Merge(tetVec[i]);
        tetVec[i].Clear();
    }
    GENERIC_ASSERT(count == master.tetrahedrons.size())
    return true;
}

bool MeshFlow3D::ExportVtkFile(const std::string & filename, const TetrahedronData & tet)
{
    return MeshFileUtility::ExportVtkFile(filename, tet);
}

bool MeshFlow3D::SliceOverheightLayers(std::list<MeshSketchLayer3D> & meshSktLyrs, float_t ratio)
{
    bool sliced = false;
    auto curr = meshSktLyrs.begin();
    for(;curr != meshSktLyrs.end();){
        auto currH = curr->GetHeight();
        if(curr != meshSktLyrs.begin()){
            auto prev = curr; prev--;
            auto prevH = prev->GetHeight();
            auto r = currH / (float_t)prevH;
            if(math::GT(r, ratio)){
                auto [top, bot] = curr->Split();
                curr = meshSktLyrs.erase(curr);
                curr = meshSktLyrs.insert(curr, bot);
                curr = meshSktLyrs.insert(curr, top);
                sliced = true;
                curr++;
            }
        }
        auto next = curr; next++;
        if(next != meshSktLyrs.end()){
            auto nextH = next->GetHeight();
            auto r = currH / (float_t)nextH;
            if(math::GT(r, ratio)){
                auto [top, bot] = curr->Split();
                curr = meshSktLyrs.erase(curr);
                curr = meshSktLyrs.insert(curr, bot);
                curr = meshSktLyrs.insert(curr, top);
                sliced = true;
                curr++;
            }
        }
        curr++;
    }
    return sliced;
}

void MeshFlow3D::SplitOverlengthIntersections(InterfaceIntersections & intersections, coor_t maxLength)
{
    if(0 == maxLength) return;
    for(auto & intersection : intersections)
        SplitOverlengthSegments(intersection, maxLength);
}

void MeshFlow3D::SplitOverlengthSegments(Segment2DContainer & segments, coor_t maxLength)
{
    if(0 == maxLength) return;
    coor_t maxLenSq = maxLength * maxLength;

    auto overLen = [&maxLenSq](const Segment2D<coor_t> & segment)
    {
        return DistanceSq(segment[0], segment[1]) > maxLenSq;
    };

    auto split = [](const Segment2D<coor_t> & segment)
    {
        auto ct = (segment[0] + segment[1]) * 0.5;
        return std::make_pair(Segment2D<coor_t>(segment[0], ct), Segment2D<coor_t>(ct, segment[1]));
    };

    auto it = segments.begin();
    while(it != segments.end()){
        if(overLen(*it)){
            auto [s1, s2] = split(*it);
            it = segments.erase(it);
            it = segments.insert(it, s1);
            it = segments.insert(it, s2);
        }
        else it++;
    }
}

void MeshFlow3D::SplitOverlengthPolygons(PolygonContainer & polygons, coor_t maxLength)
{
    for(auto & polygon : polygons)
        SplitOverlengthPolygon(polygon, maxLength);
}

void MeshFlow3D::SplitOverlengthPolygon(Polygon2D<coor_t> & polygon, coor_t maxLength)
{
    if(0 == maxLength) return;
    coor_t maxLenSq = maxLength * maxLength;

    auto overLen = [&maxLenSq](const Point2D<coor_t> & p1, const Point2D<coor_t> & p2)
    {
        return DistanceSq(p1, p2) > maxLenSq;
    };
        
    std::list<Point2D<coor_t> > points(polygon.ConstBegin(), polygon.ConstEnd());
    auto curr = points.begin();
    while(curr != points.end()){
        auto next = curr; next++;
        if(next == points.end())
            next = points.begin();
        if(overLen(*curr, *next)){
            auto ct = (*curr + *next) * 0.5;
            points.insert(next, ct);
        }
        else curr++;
    }

    polygon.Clear();
    polygon.Insert(polygon.End(), points.begin(), points.end());
}

std::unique_ptr<Point2DContainer> MeshFlow3D::AddPointsFromBalancedQuadTree(const Segment2DContainer & segments, size_t threshold)
{    
    std::list<Point2D<coor_t> * > objs;
    for(const auto & segment : segments){
        objs.push_back(const_cast<Point2D<coor_t> * >(&segment[0]));
        objs.push_back(const_cast<Point2D<coor_t> * >(&segment[1]));
    }
    return AddPointsFromBalancedQuadTree(std::move(objs), threshold);
}

std::unique_ptr<Point2DContainer> MeshFlow3D::AddPointsFromBalancedQuadTree(const PolygonContainer & polygons, size_t threshold)
{
    std::list<Point2D<coor_t> * > objs;
    for(const auto & polygon : polygons){
        for(size_t i = 0; i < polygon.Size(); ++i)
            objs.push_back(const_cast<Point2D<coor_t> * >(&polygon[i]));
    }
    return AddPointsFromBalancedQuadTree(std::move(objs), threshold);
}

std::unique_ptr<Point2DContainer> MeshFlow3D::AddPointsFromBalancedQuadTree(std::list<Point2D<coor_t> * > points, size_t threshold)
{    
    Box2D<coor_t> bbox;
    for(auto * point : points) bbox |= *point;

    float_t minLen = std::min(bbox.Length(), bbox.Width()) / 10.0;
    threshold = std::max(size_t(1), threshold);

    using Tree = tree::QuadTree<coor_t, Point2D<coor_t>, PointExtent>;
    using Node = typename Tree::QuadNode;
    using TreeBuilder = tree::QuadTreeBuilderMT<Point2D<coor_t>, Tree>;
    Tree tree(bbox);
    TreeBuilder builder(tree);
    builder.Build(points, threshold);

    auto condition = [minLen](Node * node)
    {
        auto bbox = node->GetBBox();
        return node->GetObjs().size() > 0 && std::min(bbox.Length(), bbox.Width()) > minLen; 
    };

    Tree::CreateSubNodesIf(&tree, condition);

    tree.Balance();
    std::list<Node * > leafNodes;
    Tree::GetAllLeafNodes(&tree, leafNodes);

    auto addin = std::make_unique<Point2DContainer>();
    for(auto node : leafNodes){
        if(node->GetObjs().size() > 0) continue;
        const auto & box = node->GetBBox();
        Point2D<coor_t> ct = box.Center().Cast<coor_t>();
        if(Contains(bbox, ct))
            addin->emplace_back(std::move(ct));
    }
    return addin;
}