#include "MeshFlow2D.h"
#include "generic/geometry/TriangulationRefinement.hpp"
#include "generic/tree/QuadTreeUtilityMT.hpp"
#include "generic/geometry/HashFunction.hpp"
#include "generic/geometry/Transform.hpp"
#include "generic/geometry/Topology.hpp"
#include "generic/geometry/Utility.hpp"
#include "generic/tools/FileSystem.hpp"
#include "generic/tools/Tools.hpp"
#include "MeshIO.h"
#include <memory>
#include <ctime>
using namespace generic;
using namespace emesh;
bool MeshFlow2D::LoadGeometryFiles(const std::string & filename, FileFormat format, PolygonContainer & polygons)
{
    polygons.clear();
    try {
        switch (format) {
            case FileFormat::DomDmc : {
                std::string dom = filename + ".dom";
                std::string dmc = filename + ".dmc";
                auto results = std::make_unique<std::map<int, SPtr<PolygonContainer> > >();
                if(!io::ImportDomDmcFiles(dom, dmc, *results)) return false;
                for(const auto & result : *results){
                    if(result.second){
                        for(auto & polygon : *result.second)
                            polygons.emplace_back(std::move(polygon));
                    }
                }
                return true;
            }
            case FileFormat::WKT : {
                std::string wkt = filename + ".wkt";
                auto pwhs = std::make_unique<PolygonWithHolesContainer>();
                if(!io::ImportWktFile(wkt, *pwhs)) return false;
                for(auto & pwh : *pwhs){
                    polygons.emplace_back(std::move(pwh.outline));
                    for(auto & hole : pwh.holes)
                        polygons.emplace_back(std::move(hole));
                }
                return true;
            }
            default : return false;
        }
    }
    catch (...) { return false; }
    return true;
}

bool MeshFlow2D::ExtractIntersections(const PolygonContainer & polygons, std::vector<Segment2D<coor_t> > & segments)
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

bool MeshFlow2D::ExportMeshResult(const std::string & filename, FileFormat format, const Triangulation<Point2D<coor_t> > & triangulation)
{
    switch (format) {
        case FileFormat::MSH : {
            std::string msh = filename + ".msh";
            return io::ExportMshFile(msh, triangulation);
        }
        default : return false;
    }
}

bool MeshFlow2D::GenerateReport(const std::string & filename, const Triangulation<Point2D<coor_t> > & triangulation)
{
    TriangleEvaluator<Point2D<coor_t> > evaluator(triangulation);
    auto evaluation = evaluator.Report();
    auto rpt = filename + ".rpt";
    return io::ExportReportFile(filename, evaluation, false);
}