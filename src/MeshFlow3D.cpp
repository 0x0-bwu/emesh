#include "MeshFlow3D.h"
#include "generic/tree/QuadTreeUtilityMT.hpp"
#include "generic/thread/ThreadPool.hpp"
#include "generic/geometry/Utility.hpp"
#include "generic/tools/Log.hpp"
#include "Tetrahedralizator.h"
#include "MeshFlow2D.h"
#include "MeshIO.h"
#include <boost/geometry/index/rtree.hpp>
using namespace generic;
using namespace emesh;
bool MeshFlow3D::LoadGeometryFiles(const std::string & filename, FileFormat format, StackLayerPolygons & polygons, StackLayerInfos & infos)
{
    try {
        std::string stackFile = filename + ".stack";
        if(!io::ImportLayerStackFile(stackFile, infos)) return false;

        polygons.assign(infos.size(), nullptr);

        switch (format) {
            case FileFormat::DomDmc : {
                std::string dom = filename + ".dom";
                std::string dmc = filename + ".dmc";
                auto results = std::make_shared<std::map<int, SPtr<PolygonContainer> > >(); 
                if(!io::ImportDomDmcFiles(dom, dmc, *results)) return false;
                if(results->size() != infos.size()) return false;

                size_t i = 0;
                for(auto result : *results)
                    polygons[i++] = result.second;
                
                return true;
            }
            case FileFormat::WKT : {
                bool res = true;
                auto pwhs = std::make_shared<PolygonWithHolesContainer>();
                for(size_t i = 0; i < infos.size(); ++i){
                    std::string wkt = filename + "_" + std::to_string(i + 1) + ".wkt";
                    res = res && io::ImportWktFile(wkt, *pwhs);
                    polygons[i] = std::make_shared<PolygonContainer>();
                    for(auto & pwh : *pwhs){
                        polygons[i]->emplace_back(std::move(pwh.outline));
                        for(auto & hole : pwh.holes)
                            polygons[i]->emplace_back(std::move(hole));
                    }
                }

                return true;
            }
            default : return false;
        }
    }
    catch (...) {
        polygons.clear();
        infos.clear();
        return false;
    }
    return true;
}

bool MeshFlow3D::CleanGeometries(StackLayerPolygons & polygons, coor_t distance)
{
    bool res = true;
    for(auto & layer : polygons){
        if(nullptr == layer) continue;
        res = res && CleanLayerGeometries(*layer, distance);
    }
    return res;
}

bool MeshFlow3D::CleanLayerGeometries(PolygonContainer & polygons, coor_t distance)
{
    for(auto & polygon : polygons){
        Polygon2D<coor_t> out;
        boost::geometry::simplify(polygon, out, distance);
        if(polygon.Front() == polygon.Back()) polygon.PopBack();
        if(out.Size() < 3) return false;
        if(out.Size() != polygon.Size())
            std::swap(polygon, out);
    }
    return true;
}

bool MeshFlow3D::ExtractModelsIntersections(std::vector<StackLayerModel * > & models)
{
    for(size_t i = 0; i < models.size(); ++i)
        ExtractModelIntersections(*(models[i]));
    return true;
}

bool MeshFlow3D::ExtractModelIntersections(StackLayerModel & model)
{
    if(model.hasSubModels()){
        bool res = true;
        for(size_t i = 0; i < 4; ++i)
            res = res && ExtractModelIntersections((*model.subModels[i]));
        return res;
    }
    else{
        if(nullptr == model.inGeoms) return false;
        model.intersections.reset(new InterfaceIntersections);
        return ExtractInterfaceIntersections(*model.inGeoms, *model.intersections);
    }
}

bool MeshFlow3D::ExtractInterfaceIntersections(const StackLayerPolygons & polygons, InterfaceIntersections & intersections)
{
    intersections.clear();
    if(polygons.size() == 0) return false;

    intersections.resize(polygons.size() + 1, nullptr);
    for(auto & intersection : intersections)
        intersection = std::make_shared<Segment2DContainer>();
    
    ExtractInterfaceIntersection(*(polygons.front()), *(intersections.front()));
    if(polygons.size() > 1) ExtractInterfaceIntersection(*(polygons.back()), *(intersections.back()));    
    for(size_t i = 0; i < polygons.size() - 1; ++i){
        ExtractInterfaceIntersection(*(polygons[i]), *(polygons[i + 1]), *(intersections[i + 1]));
    }
    return true;
}

bool MeshFlow3D::ExtractInterfaceIntersection(const PolygonContainer & layer, Segment2DContainer & intersection)
{
    intersection.clear();
    Polygons2Segments(layer, intersection);
    std::vector<Segment2D<coor_t> > results;
    boost::polygon::intersect_segments(results, intersection.begin(), intersection.end());
    intersection.clear();
    intersection.insert(intersection.end(), results.begin(), results.end());
    return true;
}

bool MeshFlow3D::ExtractInterfaceIntersection(const PolygonContainer & layer1, const PolygonContainer & layer2, Segment2DContainer & intersection)
{
    intersection.clear();
    Polygons2Segments(layer1, intersection);
    Polygons2Segments(layer2, intersection);
    std::vector<Segment2D<coor_t> > results;
    boost::polygon::intersect_segments(results, intersection.begin(), intersection.end());
    intersection.clear();
    intersection.insert(intersection.end(), results.begin(), results.end());
    return true;
}

bool MeshFlow3D::SplitOverlengthModelsIntersections(std::vector<Ptr<StackLayerModel> > & models, coor_t maxLength)
{
    for(auto model : models){
        GENERIC_ASSERT(model)
        if(model->intersections){
            GENERIC_ASSERT(false == model->hasSubModels())
            SplitOverlengthInterfaceIntersections(*(model->intersections), maxLength);
        }
        else{
            std::vector<Ptr<StackLayerModel> > subModels;
            StackLayerModel::GetAllLeafModels(*model, subModels);
            SplitOverlengthModelsIntersections(subModels, maxLength);
        }
    }
    return true;
}

bool MeshFlow3D::SplitOverlengthInterfaceIntersections(const InterfaceIntersections & intersections, coor_t maxLength)
{
    for(const auto & intersection : intersections)
        SplitOverlengthSegments(*intersection, maxLength);
    return true;
}

bool MeshFlow3D::SplitOverlengthSegments(Segment2DContainer & segments, coor_t maxLength)
{
    Segment2DContainer tmp;
    auto maxLenSq = maxLength * maxLength;
    auto lenSq = [](const Segment2D<coor_t> & seg)
    {
        return DistanceSq(seg[0], seg[1]);
    };

    auto split = [](const Segment2D<coor_t> & seg)
    {
        auto mid = (seg[0] + seg[1]) * 0.5;
        return std::make_pair(Segment2D<coor_t>(seg[0], mid), Segment2D<coor_t>(mid, seg[1]));
    };

    auto iter = segments.begin();
    while(iter != segments.end()){
        if(lenSq(*iter) <= maxLenSq){
            ++iter; continue;
        }
        else{
            auto [s1, s2] = split(*iter);
            iter = segments.erase(iter);
            iter = segments.insert(iter, s2);
            iter = segments.insert(iter, s1);
        }
    }
    return true;
}

bool MeshFlow3D::AddLayerModelsGradePoints(std::vector<Ptr<StackLayerModel> > & ins, size_t maxLevel)
{
    bool res = true;
    for(auto in : ins)
        res = res && AddLayerModelGradePoints(*in, maxLevel);
    return res;
}

bool MeshFlow3D::AddLayerModelGradePoints(StackLayerModel & in, size_t maxLevel)
{
    GENERIC_ASSERT(in.intersections)
    in.gradePoints.reset(new std::vector<SPtr<Point2DContainer> >(in.intersections->size(), nullptr));
    for(size_t i = 0; i < in.intersections->size(); ++i){
        GENERIC_ASSERT(in.intersections->at(i))
        in.gradePoints->at(i) = AddPointsFromBalancedQuadTree(*(in.intersections->at(i)), maxLevel);
        if(in.gradePoints->at(i))
            log::Info("add %1% grade points at interface layer %2%", in.gradePoints->at(i)->size(), i);
    }
    return true;
}

bool MeshFlow3D::BuildMeshSketchModels(std::vector<Ptr<StackLayerModel> > & ins, std::vector<MeshSketchModel> & outs)
{
    outs.resize(ins.size());
    for(size_t i = 0; i < ins.size(); ++i)
        BuildMeshSketchModel(*(ins[i]), outs[i]);
    return true;
}

bool MeshFlow3D::BuildMeshSketchModel(const StackLayerModel & in, MeshSketchModel & out)
{
    if((in.inGeoms->size() + 1) != in.intersections->size()) return false;
    if(in.inGeoms->size() != in.sInfos->size()) return false;
    
    out.bbox = in.bbox;
    out.layers.clear();
    for(size_t i = 0; i < in.inGeoms->size(); ++i){
        MeshSketchLayer layer(i);
        layer.SetTopBotHeight(in.sInfos->at(i).elevation, in.sInfos->at(i).elevation - in.sInfos->at(i).thickness);
        layer.polygons = in.inGeoms->at(i);
        layer.SetConstrains(in.intersections->at(i), in.intersections->at(i + 1));
        if(in.gradePoints){
            layer.SetAdditionalPoints(in.gradePoints->at(i), in.gradePoints->at(i + 1));
        }
        out.layers.emplace_back(std::move(layer));
        GENERIC_ASSERT(layer.GetHeight() > 0)
    }
    return true;
}

bool MeshFlow3D::SliceOverheightLayers(MeshSketchModel & model, float_t ratio)
{
    std::list<MeshSketchLayer> layers(model.layers.begin(), model.layers.end());
    bool sliced = true;
    while(sliced){
        sliced = SliceOverheightLayers(layers, ratio);
    }
    model.layers.clear();
    model.layers.reserve(layers.size());
    for(auto & layer : layers)
        model.layers.emplace_back(std::move(layer));
    model.MakeLayerIndexOrdered();
    return true;
}

bool MeshFlow3D::GenerateTetrahedronVecFromSketchModels(std::vector<MeshSketchModel> & models, TetrahedronDataVec & tetVec, const Mesh3Ctrl & ctrl)
{
    auto size = models.size();
    tetVec.resize(size);
    for(size_t i = 0; i < size; ++i){
        log::Info("remain models: %1%/%2%", size - i, size);
        GenerateTetrahedronDataFromSketchModel(models, i, tetVec[i], ctrl);
        models[i].layers.clear();
    }
    return true;   
}

bool MeshFlow3D::GenerateTetrahedronVecFromSketchModel(std::vector<MeshSketchModel> & models, size_t index, TetrahedronDataVec & tetVec, const Mesh3Ctrl & ctrl)
{
    if(index >= models.size()) return false;

    const auto & model = models[index];
    size_t layers = model.layers.size();
    tetVec.resize(layers);
    for(size_t i = 0; i < layers; ++i){
        log::Info("model %1%: remain layers: %2%/%3%", index + 1, layers - i, layers);
        if(!GenerateTetrahedronDataFromSketchLayer(model.layers[i], tetVec[i], ctrl)){
            log::Error("model %1%: failed to generate mesh of layer %2%", index + 1, i + 1);
            return false;
        }
    }
    return true;
}

bool MeshFlow3D::GenerateTetrahedronDataFromSketchModel(std::vector<MeshSketchModel> & models, size_t index, TetrahedronData & tet, const Mesh3Ctrl & ctrl)
{
    tet.Clear();
    auto tetVec = std::make_unique<TetrahedronDataVec>();
    bool res = GenerateTetrahedronVecFromSketchModel(models, index, *tetVec, ctrl);
    res = res && MergeTetrahedrons(tet, *tetVec);
    return res;
}

bool MeshFlow3D::GenerateTetrahedronDataFromSketchLayer(const MeshSketchLayer & layer, TetrahedronData & tet, const Mesh3Ctrl & ctrl)
{
    auto faces = std::make_unique<std::list<IndexFace> >();
    auto edges = std::make_unique<std::list<IndexEdge> >();
    auto points = std::make_unique<std::vector<Point3D<coor_t> > >();
    if(!ExtractTopology(layer, *points, *faces, *edges))
        return false;

    auto addin = layer.GetAdditionalPoints();
    if(addin){
        points->reserve(points->size() + addin->size());
        points->insert(points->end(), addin->begin(), addin->end());
    }

    if(!Tetrahedralize(*points, *faces, *edges, tet)){
        std::string ne = "./debug.ne";
        io::ExportNodeAndEdges(ne, *points, *edges);
        log::Error("dump debug node edge file to %1%", ne);
        return false;
    }
    
    // log::Info("finish generate mesh of layer %1%", layer.index + 1);
    return true;
}

bool MeshFlow3D::ExtractTopology(const MeshSketchLayer & layer, Point3DContainer & points, std::list<IndexFace> & faces, std::list<IndexEdge> & edges)
{
    edges.clear();
    faces.clear(); 
    points.clear();

    using EdgeSet = topology::UndirectedIndexEdgeSet;
    using LayerIdxMap = std::unordered_map<coor_t, size_t>;
    using PointIdxMap = std::unordered_map<Point2D<coor_t>, size_t, PointHash<coor_t> >;

    EdgeSet edgeSet;
    PointIdxMap ptIdxMap[2];

    auto getH = [&layer](size_t lyrIdx){ return lyrIdx == 0 ? layer.height[0] : layer.height[1]; };//lyrIdx: 0-top, 1-bot
    auto getIndex = [&getH, &ptIdxMap, &points](const Point2D<coor_t> & p, size_t lyrIdx) mutable //lyrIdx: 0-top, 1-bot
    {
        if(!ptIdxMap[lyrIdx].count(p)){
            auto index = points.size();
            ptIdxMap[lyrIdx].insert(std::make_pair(p, index));
            points.push_back(Point3D<coor_t>(p[0], p[1], getH(lyrIdx)));
        }
        return ptIdxMap[lyrIdx].at(p);
    };

    auto addEdge = [&edges, &edgeSet](IndexEdge && e) mutable
    {
        if(e.v1() == e.v2()) return;
        if(edgeSet.count(e)) return;
        edges.emplace_back(e);
        edgeSet.insert(e);
    };

    for(size_t i = 0; i < 2; ++i){
        GENERIC_ASSERT(layer.constrains[i])
        for(const auto & segment : *(layer.constrains[i])){
            IndexEdge e(getIndex(segment[0], i), getIndex(segment[1], i));
            addEdge(std::move(e));
        }
    }

    for(const auto & polygon : *layer.polygons){
        size_t size = polygon.Size();
        for(size_t i = 0; i < size; ++i){
            IndexEdge e(getIndex(polygon[i], 0), getIndex(polygon[i], 1));
            addEdge(std::move(e));
        }
    }

    //boundary
    // std::vector<Point2D<coor_t> > outline(4);
    // outline[0] = layer.bbox[0];
    // outline[1] = Point2D<coor_t>(layer.bbox[1][0], layer.bbox[0][1]);
    // outline[2] = layer.bbox[1];
    // outline[3] = Point2D<coor_t>(layer.bbox[0][0], layer.bbox[1][1]);

    // for(size_t i = 0; i < 4; ++i){
    //     size_t j = (i + 1) % 4;
    //     IndexEdge top(getIndex(outline[i], 0), getIndex(outline[j], 0));
    //     IndexEdge bot(getIndex(outline[i], 1), getIndex(outline[j], 1));
    //     IndexEdge side(getIndex(outline[i], 0), getIndex(outline[i], 1));
    //     addEdge(std::move(top));
    //     addEdge(std::move(bot));
    //     addEdge(std::move(side));
    // }
    return true;
}

// bool MeshFlow3D::ExtractTopology(const MeshSketchLayer & layer, Point3DContainer & points, std::list<IndexFace> & faces, std::list<IndexEdge> & edges)
// {
//     edges.clear();
//     faces.clear(); 
//     points.clear();

//     using EdgeSet = topology::UndirectedIndexEdgeSet;
//     using LayerIdxMap = std::unordered_map<coor_t, size_t>;
//     using PointIdxMap = std::unordered_map<Point2D<coor_t>, size_t, PointHash<coor_t> >;

//     EdgeSet edgeSet;
//     PointIdxMap ptIdxMap[2];

//     auto getH = [&layer](size_t lyrIdx){ return lyrIdx == 0 ? layer.height[0] : layer.height[1]; };//lyrIdx: 0-top, 1-bot
//     auto getIndex = [&getH, &ptIdxMap, &points](const Point2D<coor_t> & p, size_t lyrIdx) mutable //lyrIdx: 0-top, 1-bot
//     {
//         if(!ptIdxMap[lyrIdx].count(p)){
//             auto index = points.size();
//             ptIdxMap[lyrIdx].insert(std::make_pair(p, index));
//             points.push_back(Point3D<coor_t>(p[0], p[1], getH(lyrIdx)));
//         }
//         return ptIdxMap[lyrIdx].at(p);
//     };

//     auto addEdge = [&edges, &edgeSet](IndexEdge && e) mutable
//     {
//         if(e.v1() == e.v2()) return;
//         if(edgeSet.count(e)) return;
//         edges.emplace_back(e);
//         edgeSet.insert(e);
//     };


//     for(size_t i = 0; i < 2; ++i){
//         for(const auto & segment : *(layer.constrains[i])){
//             IndexEdge e(getIndex(segment[0], i), getIndex(segment[1], i));
//             addEdge(std::move(e));
//         }
//     }
//     //boundary
//     size_t size = layer.outline.Size();
//     IndexFace topFace, botFace;
//     for(size_t i = 0; i < size; ++i){
//         size_t j = (i + 1) % size;
//         // IndexFace side {getIndex(layer.outline[i], 0), getIndex(layer.outline[j], 0),
//         //                 getIndex(layer.outline[j], 1), getIndex(layer.outline[i], 1)};
//         // faces.emplace_back(std::move(side));
//         topFace.emplace_back(getIndex(layer.outline[i], 0));
//         botFace.emplace_back(getIndex(layer.outline[i], 1));
//     }
//     faces.emplace_back(std::move(topFace));
//     faces.emplace_back(std::move(botFace));

//     //side inside
//     for(const auto & polygon : *layer.polygons){
//         size_t size = polygon.Size();
//         for(size_t i = 0; i < size; ++i){
//             size_t j = (i + 1) % size;
//             IndexFace side {getIndex(polygon[i], 0), getIndex(polygon[j], 0),
//                             getIndex(polygon[j], 1), getIndex(polygon[i], 1)};
//             faces.emplace_back(std::move(side));
//         }
//     }
//     return true;
// }

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
    auto isVertical = [&points](const IndexEdge & e)
    {
        const auto & p1 = points[e.v1()];
        const auto & p2 = points[e.v2()];
        return p1[0] == p2[0] && p1[1] == p2[1];
    };

    auto iter = edges.begin();
    while(iter != edges.end()){
        if(surfaceOnly && isVertical(*iter)){
            ++iter; continue;
        }
        else if(lenSq(*iter) <= maxLenSq){
            ++iter; continue;
        }
        else{
            auto [e1, e2] = split(*iter);
            iter = edges.erase(iter);
            iter = edges.insert(iter, e2);
            iter = edges.insert(iter, e1);
        }
    }
    return true;
}

bool MeshFlow3D::WriteNodeAndEdgeFiles(const std::string & filename, const Point3DContainer & points, const std::list<IndexEdge> & edges)
{
    // geometry::tet::PiecewiseLinearComplex<Point3D<coor_t> >  plc;
    // plc.points = points;
    // plc.surfaces.resize(edges.size());
    // size_t index = 0;

    // using IdxPolyline = typename geometry::tet::PiecewiseLinearComplex<Point3D<coor_t> >::IdxPolyLine;
    // for(const auto & edge : edges){
    //     plc.surfaces[index].faces.push_back(IdxPolyline{edge.v1(), edge.v2()});
    //     index++;
    // }
    // return geometry::tet::WritePlcToNodeAndEdgeFiles(filename, plc);
    return io::ExportNodeAndEdges(filename, points, edges);
}

bool MeshFlow3D::Tetrahedralize(const Point3DContainer & points, const std::list<std::vector<size_t> > & faces, const std::list<IndexEdge> & edges, TetrahedronData & tet)
{
    Tetrahedralizator tetrahedralizator(tet);
    return tetrahedralizator.Tetrahedralize(points, faces, edges);
}

bool MeshFlow3D::MergeTetrahedrons(TetrahedronData & master, TetrahedronDataVec & tetVec)
{
    size_t count = master.tetrahedrons.size();
    TetrahedronDataMerger merger(master);
    for(size_t i = 0; i < tetVec.size(); ++i){
        count += tetVec[i].tetrahedrons.size();
        merger.Merge(tetVec[i]);
        tetVec[i].Clear();
    }
    GENERIC_ASSERT(count == master.tetrahedrons.size())
    return true;
}

bool MeshFlow3D::ExportResultFile(const std::string & filename,  FileFormat format, const TetrahedronData & tet)
{
    switch (format) {
        case FileFormat::VTK : {
            std::string vtk = filename + ".vtk";
            return io::ExportVtkFile(vtk, tet);
        }
        case FileFormat::MSH : {
            std::string msh = filename + ".msh";
            return io::ExportMshFile(msh, tet);
        }
        default : return false;
    }
}

bool MeshFlow3D::SliceOverheightModels(std::vector<MeshSketchModel> & models, float_t ratio)
{
    for(auto & model : models)
        SliceOverheightLayers(model, ratio);
    return true;
}

bool MeshFlow3D::SliceOverheightLayers(std::list<MeshSketchLayer> & layers, float_t ratio)
{
    bool sliced = false;
    auto curr = layers.begin();
    for(;curr != layers.end();){
        auto currH = curr->GetHeight();
        if(curr != layers.begin()){
            auto prev = curr; prev--;
            auto prevH = prev->GetHeight();
            auto r = currH / (float_t)prevH;
            if(math::GT(r, ratio)){
                auto [top, bot] = curr->Slice();
                curr = layers.erase(curr);
                curr = layers.insert(curr, bot);
                curr = layers.insert(curr, top);
                sliced = true;
                curr++;
            }
        }
        auto next = curr; next++;
        if(next != layers.end()){
            auto nextH = next->GetHeight();
            auto r = currH / (float_t)nextH;
            if(math::GT(r, ratio)){
                auto [top, bot] = curr->Slice();
                curr = layers.erase(curr);
                curr = layers.insert(curr, bot);
                curr = layers.insert(curr, top);
                sliced = true;
                curr++;
            }
        }
        curr++;
    }
    return sliced;
}

void MeshFlow3D::Polygons2Segments(const PolygonContainer & polygons, std::list<Segment2D<coor_t> > & segments)
{
    for(const auto & polygon : polygons){
        size_t size = polygon.Size();
        size_t end = polygon.ConstFront() == polygon.ConstBack() ? size - 1 : size;
        for(size_t i = 0; i < end; ++i){
            size_t j = (i + 1) % size;
            segments.emplace_back(Segment2D<coor_t>(polygon[i], polygon[j]));
        }
    }
}

std::unique_ptr<Point2DContainer> MeshFlow3D::AddPointsFromBalancedQuadTree(const Segment2DContainer & segments, size_t maxLevel)
{    
    std::list<Point2D<coor_t> * > points;
    for(const auto & segment : segments){
        points.push_back(const_cast<Point2D<coor_t> * >(&segment[0]));
        points.push_back(const_cast<Point2D<coor_t> * >(&segment[1]));
    }

    Box2D<coor_t> bbox;
    boost::polygon::envelope_segments(bbox, segments.begin(), segments.end());

    using QuadTree = tree::QuadTree<coor_t, Point2D<coor_t>, PointExtent>;
    using QuadNode = typename QuadTree::QuadNode;
    using RectNode = std::pair<Box2D<coor_t>, CPtr<Segment2D<coor_t> > >;
    using RectTree = boost::geometry::index::rtree<RectNode, boost::geometry::index::dynamic_rstar>;
    
    QuadTree quadTree(bbox);
    quadTree.Build(points, 0);
    auto condition = [&maxLevel](QuadNode * node)
    {
        auto bbox = node->GetBBox();
        return node->GetLevel() < maxLevel && node->GetObjs().size() > 0 && std::min(bbox.Length(), bbox.Width()) > 1; 
    };

    QuadTree::CreateSubNodesIf(&quadTree, condition);

    quadTree.Balance();
    std::list<QuadNode * > leafNodes;
    QuadTree::GetAllLeafNodes(&quadTree, leafNodes);

    boost::geometry::index::dynamic_rstar para(16);
    RectTree rectTree(para);
    for(const auto & segment : segments){
        auto box = Extent(segment);
        rectTree.insert(std::make_pair(box, &segment));
    }

    std::vector<RectNode> rectNodes;
    auto addin = std::make_unique<Point2DContainer>();
    for(auto node : leafNodes){
        if(node->GetObjs().size() > 0) continue;
        const auto & box = node->GetBBox();
        Point2D<coor_t> ct = box.Center().Cast<coor_t>();

        rectNodes.clear();
        Box2D<coor_t> searchBox(ct, ct);
        rectTree.query(boost::geometry::index::intersects(searchBox), std::back_inserter(rectNodes));

        if(rectNodes.empty()) addin->emplace_back(std::move(ct));
        else{
            bool flag = true;
            float_t distSq = std::min(box.Width(), box.Length());
            distSq = 0.7 * distSq * distSq;
            for(const auto & rectNode : rectNodes){
                if(PointSegmentDistanceSq(ct, *rectNode.second) < distSq){
                    flag = false; break;
                }
            }
            if(flag) addin->emplace_back(std::move(ct)); 
        }            
    }
    if(addin->size()) return addin;
    return nullptr;
}

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

// bool MeshFlow3DMT::GenerateTetrahedronVecFromSketchModels(std::vector<MeshSketchModel> & models, TetrahedronDataVec & tetVec, const Mesh3Ctrl & ctrl, size_t threads)
// {
//     //multi-threading by model
//     auto size = models.size();
//     tetVec.resize(size);
//     thread::ThreadPool pool(threads);
//     std::vector<std::future<bool> > futures(size);
//     for(size_t i = 0; i < size; ++i)
//         futures[i] = pool.Submit(std::bind(&MeshFlow3D::GenerateTetrahedronDataFromSketchModel, std::ref(models), i, std::ref(tetVec[i]), std::ref(ctrl)));        
    
//     bool res = true;
//     for(auto & future : futures)
//         res = res && future.get();
//     return res;
// }

bool MeshFlow3DMT::GenerateTetrahedronVecFromSketchModels(std::vector<MeshSketchModel> & models, TetrahedronDataVec & tetVec, const Mesh3Ctrl & ctrl, size_t threads)
{
    //multi-threading by layer
    bool res = true;
    auto size = models.size();
    tetVec.resize(size);
    for(size_t i = 0; i < size; ++i)
        res = res && MeshFlow3DMT::GenerateTetrahedronDataFromSketchModel(models, i, tetVec[i], ctrl, threads);            
    return res;
}

bool MeshFlow3DMT::GenerateTetrahedronDataFromSketchModel(std::vector<MeshSketchModel> & models, size_t index, TetrahedronData & tet, const Mesh3Ctrl & ctrl, size_t threads)
{
    tet.Clear();
    auto tetVec = std::make_unique<TetrahedronDataVec>();
    bool res = GenerateTetrahedronVecFromSketchModel(models, index, *tetVec, ctrl);
    res = res && MeshFlow3D::MergeTetrahedrons(tet, *tetVec);
    return res;
}

bool MeshFlow3DMT::GenerateTetrahedronVecFromSketchModel(std::vector<MeshSketchModel> & models, size_t index, TetrahedronDataVec & tetVec, const Mesh3Ctrl & ctrl, size_t threads)
{
    if(index >= models.size()) return false;

    const auto & model = models[index];
    size_t layers = model.layers.size();
    tetVec.resize(layers);
    
    thread::ThreadPool pool(threads);
    std::vector<std::future<bool> > futures(layers);
    for(size_t i = 0; i < layers; ++i)
        futures[i] = pool.Submit(std::bind(&MeshFlow3D::GenerateTetrahedronDataFromSketchLayer, std::ref(model.layers[i]), std::ref(tetVec[i]), std::ref(ctrl)));        
    
    bool res = true;
    for(auto & future : futures)
        res = res && future.get();
    return res;
}