#include "MeshFileUtility.h"
#include "generic/geometry/TetrahedralizationIO.hpp"
#include "generic/geometry/Transform.hpp"
#include "generic/tools/FileSystem.hpp"
#include "generic/tools/Tools.hpp"
#include "generic/tools/Parser.hpp"
#include <memory>
using namespace generic;
using namespace emesh;
bool MeshFileUtility::LoadMeshCtrlFile(const std::string & txt, Mesh2Ctrl & ctrl)
{
    std::ifstream in(txt);
    if(!in.is_open()) return false;

    std::string line, tmp;
    float_t scale(0), tolerance(0), maxEdge(0), minEdge(0), minAlpha(0);
    while(!in.eof()){
        line.clear();
        std::getline(in, line);
        if(line.empty()) continue;
        if(0 == line.rfind("Tolerance", 0)){
            std::stringstream ss(line);
            ss.ignore(std::string("Tolerance (um) = ").size());
            ss >> tolerance;
        }
        if(0 == line.rfind("Max Edge Length")){
            std::stringstream ss(line);
            ss.ignore(std::string("Max Edge Length (um) = ").size());
            ss >> maxEdge;
        }
        if(0 == line.rfind("QualityTarget")){
            std::stringstream ss(line);
            ss.ignore(std::string("QualityTarget = ").size());
            ss >> minAlpha >> minEdge;
        }
    }

    //validation
    if(0 < tolerance) {
        ctrl.scale2Int = 10 / tolerance;
        ctrl.tolerance = tolerance * ctrl.scale2Int;

        if(0 < minAlpha) ctrl.minAlpha = math::Rad(minAlpha);
        if(0 < maxEdge) ctrl.maxEdgeLen = maxEdge * ctrl.scale2Int;
        if(0 < minEdge) ctrl.minEdgeLen = minEdge * ctrl.scale2Int;
    }
    in.close();
    return true;
}

bool MeshFileUtility::LoadWktFile(const std::string & wkt, float_t scale, PolygonContainer & polygons)
{
    polygons.clear();
    std::list<Polygon2D<float_t> > tmp;
    if(!geometry::GeometryIO::Read<Polygon2D<float_t> >(wkt, std::back_inserter(tmp))) return false;

    auto trans = makeScaleTransform2D(scale);
    for(const auto & t : tmp){
        auto ts = trans * t;
        polygons.emplace_back(std::move(ts.Cast<coor_t>()));
    }
    return true;
}

bool MeshFileUtility::LoadDomDmcFiles(const std::string & dom, const std::string & dmc, float_t scale2Int, PolygonContainer & polygons)
{
    std::ifstream f_dom(dom);
    if(!f_dom.is_open()) return false;
    
    std::ifstream f_dmc(dmc);
    if(!f_dmc.is_open()) return false;

    polygons.clear();
    std::string tmp, netName, layerName;
    float_t x, y, llx, urx, lly, ury;
    int ptSize, bSolid, layerId, netId, signalLayer;

    while(!f_dmc.eof()){
        tmp.clear();
        std::getline(f_dmc, tmp);
        if(tmp.empty()) continue;

        std::stringstream ss(tmp);
        ss >> ptSize >> bSolid >> layerId >> llx >> urx >> lly >> ury >> netId >> netName >> signalLayer >> layerName;

        Polygon2D<coor_t> polygon;
        polygon.GetPoints().reserve(ptSize);
        for(size_t i = 0; i < ptSize; ++i){
            f_dom >> x >> y;
            polygon << Point2D<coor_t>(scale2Int * x, scale2Int * y);
        }
        polygons.emplace_back(std::move(polygon));
    }
    f_dom.close();
    f_dmc.close();
    return true;
}

bool MeshFileUtility::ImportMshFile(const std::string & msh, Triangulation<Point2D<coor_t> > & triangulation, float_t scale2Int)
{
    std::ifstream in(msh);
    if(!in.is_open()) return false;
    
    triangulation.Clear();
    auto & points = triangulation.points;
    auto & vertices = triangulation.vertices;
    auto & triangles = triangulation.triangles;

    std::string line;
    size_t nodes, elements;
    in >> nodes >> elements;
    
    auto loadNodes = [&triangulation, &in, nodes, scale2Int]() mutable
    {
        triangulation.points.reserve(nodes);
        triangulation.vertices.reserve(nodes);

        size_t index;
        float_t x, y;
        std::string line, tmp;
        for(size_t i = 0; i < nodes; ++i){
            line.clear();
            std::getline(in, line);
            std::stringstream ss(line);
            ss >> tmp >> index >> x >> y >> tmp;
            triangulation.points.emplace_back(Point2D<coor_t>(scale2Int * x, scale2Int * y));
            triangulation.vertices.emplace_back(IndexVertex());
            triangulation.vertices.back().index = i;
        }
    };

    auto loadElements = [&triangulation, &in, elements]() mutable
    {
        std::unordered_map<IndexEdge, size_t, IndexEdgeHash, IndexEdgeCompare> edgeTriIdxMap;

        triangulation.triangles.reserve(elements);
        size_t index, v[4];
        std::string line, tmp;
        for(size_t i = 0; i < elements; ++i){
            line.clear();
            std::getline(in, line);
            std::stringstream ss(line);
            ss >> tmp >> index >> v[0] >> v[1] >> v[2] >> v[3] >> tmp;
            triangulation.triangles.emplace_back(IndexTriangle());
            triangulation.triangles.back().vertices = std::array<VerIdx, 3>{v[0] - 1, v[1] - 1, v[2] - 1};

            for(size_t j = 0; j < 3; ++j){
                triangulation.vertices[v[j] - 1].triangles.insert(i);

                IndexEdge e(v[j] - 1, v[(j + 1) % 3] - 1);
                if(edgeTriIdxMap.count(e)){
                    TriIdx it = edgeTriIdxMap.at(e);
                    triangulation.triangles.back().neighbors[j] = it;
                    index_t in = triangulation.triangles[it].iEg(e);
                    triangulation.triangles[it].neighbors[in] = i;
                }
                else edgeTriIdxMap.insert(std::make_pair(e, i));
            }
        }
    };

    while(!in.eof()){
        line.clear();
        std::getline(in, line);
        if(line.empty()) continue;
        if(line.compare("NODES") == 0){
            loadNodes();
            line.clear();
            std::getline(in, line);
            if(line.compare("END_OF_NODES") != 0){
                triangulation.Clear();
                in.close();
                return false;
            }
        }
        if(line.compare("ELEMENTS") == 0){
            loadElements();
            line.clear();
            std::getline(in, line);
            if(line.compare("END_OF_ELEMENTS") != 0){
                triangulation.Clear();
                in.close();
                return false;
            }
        }
    }

    //make ccw;
    for(size_t it = 0; it < triangulation.triangles.size(); ++it){
        if(!TriangulationUtility<Point2D<coor_t> >::isCCW(triangulation, it))
            triangulation.triangles[it].Reverse();
    }

    in.close();
    return true;
}

bool MeshFileUtility::ExportMshFile(const std::string & msh, const Triangulation<Point2D<coor_t> > & triangulation, float_t scale)
{
    std::ofstream out(msh);
    if(!out.is_open()) return false;

    const auto & points = triangulation.points;
    const auto & vertices = triangulation.vertices;   
    const auto & triangles = triangulation.triangles;

    char sp(32);
    out << sp << vertices.size() << sp << triangles.size() << std::endl;

    out << std::setiosflags(std::ios::right);
    out << std::setiosflags(std::ios::fixed) << std::setprecision(12);

    size_t index = 0;
    out << "NODES" << std::endl;
    for(const auto & vertex : vertices){
        index++;
        const auto & point = points[vertex.index];
        out << "Node:" << std::setw(10) << index << sp;
        for(size_t i = 0; i < Point2D<coor_t>::dim; ++i) {
            out << std::setw(20) << (point[i] * scale) << sp;
        }
        out << "P" << index << std::endl;
    }
    out << "END_OF_NODES" << std::endl;

    index = 0;
    out << "ELEMENTS" << std::endl;
    for(const auto & triangle : triangles){
        index++;
        const auto & vs = triangle.vertices;
        out << "Element:" << std::setw(10) << index << sp;
        for(size_t i = 0; i < 3; ++i){
            out << std::setw(10) << vs[i] + 1;
        }
        out << std::setw(10) << vs[2] + 1;
        out << sp << "R1" << std::endl;
    }
    out << "END_OF_ELEMENTS" << std::endl;
    out << "END";

    out.close();
    return true;
}

bool MeshFileUtility::ExportReportFile(const std::string & rpt, const Mesher2D::MeshEvaluation & evaluation, float_t scale, bool pureText)
{
    std::ofstream out(rpt);
    if(!out.is_open()) return false;

    auto currentTime = std::time(nullptr);
    out << std::asctime(std::localtime(&currentTime)) << std::endl;

    auto printProgress = [&out](float_t progress, size_t len) mutable { for(size_t i = 0; i < progress * len; ++i) out << '#'; };

    out << std::setiosflags(std::ios::left);
    out << std::fixed << std::setprecision(2);
    out << std::setw(16) << "Nodes:" << evaluation.nodes << std::endl;
    out << std::setw(16) << "Elements:" << evaluation.elements << std::endl;
    out << std::endl;

    out << std::setw(16) << "Minimum Angle:" << math::Deg(evaluation.minAngle) << std::endl;
    out << std::setw(16) << "Maximum Angle:" << math::Deg(evaluation.maxAngle) << std::endl;
    out << std::setw(16) << "Distribution:" << std::endl;

    size_t colums = evaluation.triAngleHistogram.size();
    float_t step = math::Deg(math::pi / 3 / colums);
    float_t start = 0;
    for(size_t i = 0; i < colums; ++i){
        std::stringstream label;
        label << std::setiosflags(std::ios::right);
        label << std::setw(2) << start << '-' << std::setw(2) << (start + step) << ':';
        out << std::setw(16) << label.str();
        if(pureText) out << (evaluation.triAngleHistogram.at(i) * 100) << "%";
        else printProgress(evaluation.triAngleHistogram.at(i), 100);
        out << std::endl;
        start += step;
    }
    out << std::endl;

    out << std::setw(16) << "Minimum Edge:" << evaluation.minEdgeLen * scale << std::endl;
    out << std::setw(16) << "Maximum Edge:" << evaluation.maxEdgeLen * scale << std::endl;
    out << std::setw(16) << "Distribution:" << std::endl;

    colums = evaluation.triEdgeLenHistogram.size();
    step = (evaluation.maxEdgeLen - evaluation.minEdgeLen) / colums * scale;
    start = evaluation.minEdgeLen * scale;
    for(size_t i = 0; i < colums; ++i){
        std::stringstream label;
        label << std::setiosflags(std::ios::right);
        label << std::fixed << std::setprecision(2);
        label << start << '-' << (start + step) << ':';
        out << std::setw(24) << label.str();
        if(pureText) out << (evaluation.triEdgeLenHistogram.at(i) * 100) << "%";
        else printProgress(evaluation.triEdgeLenHistogram.at(i), 100);
        out << std::endl;
        start += step;
    }

    out.close();
    return true;
}
