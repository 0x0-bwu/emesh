#include "MeshIO.h"
#include "generic/geometry/TetrahedralizationIO.hpp"
#include "generic/tools/StringHelper.hpp"
#include "generic/tools/Parser.hpp"
#include <ctime>
using namespace generic;
namespace emesh {
namespace io {
bool ImportLayerStackFile(const std::string & stack, StackLayerInfos & infos)
{
    std::ifstream in(stack);
    if(!in.is_open()) return false;
    
    char sp(32);
    std::string tmp;
    size_t line = 0;

    size_t layers;
    double scale = 1.0;
    double elevation, thickness;
    while(!in.eof()){
        line++;
        std::getline(in, tmp);
        if(tmp.empty()) continue;

        auto items = parser::Split(tmp, sp);
        if(items.size() < 1) return false;
        layers = std::stol(items[0]);
        if(items.size() > 1) scale = std::stod(items[1]);
        break;
    }

    infos.clear();
    infos.reserve(layers);
    while(!in.eof()){
        line++;
        std::getline(in, tmp);
        if(tmp.empty()) continue;

        auto items = parser::Split(tmp, sp);
        if(items.size() < 2) return false;
        elevation = std::stod(items[0]) * scale;
        thickness = std::stod(items[1]) * scale;
        infos.push_back(StackLayerInfo{});
        infos.back().elevation = elevation;
        infos.back().thickness = thickness;
    }

    in.close();

    if(infos.size() > 1){
        for(size_t i = 0; i < infos.size() - 1; ++i){
            infos[i].thickness = infos[i].elevation - infos[i + 1].elevation;
            if(infos[i].thickness <= 0) return false;
        }
    }
    return infos.size() == layers;
}

bool ImportDomDmcFiles(const std::string & dom, const std::string & dmc, std::map<int, SPtr<PolygonContainer> > & results)
{
    results.clear();
    
    std::ifstream f_dom(dom);
    if(!f_dom.is_open()) return false;
    
    std::ifstream f_dmc(dmc);
    if(!f_dmc.is_open()) return false;

    std::string tmp, netName, layerName;
    int64_t x, y, llx, urx, lly, ury;
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
            polygon << Point2D<coor_t>(x, y);
        }
        auto iter = results.find(layerId);
        if(iter == results.end()){
            iter = results.insert(std::make_pair(layerId, std::make_shared<PolygonContainer>())).first;
        }
        iter->second->emplace_back(std::move(polygon));
    }
    f_dom.close();
    f_dmc.close();
    return true;
}

bool ImportWktFile(const std::string & wkt, PolygonWithHolesContainer & pwhs)
{
    pwhs.clear();
    return geometry::GeometryIO::Read<PolygonWithHoles2D<coor_t> >(wkt, std::back_inserter(pwhs));
}

bool ImportMshFile(const std::string & msh, Triangulation<Point2D<coor_t> > & triangulation)
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
    
    auto loadNodes = [&triangulation, &in, nodes]() mutable
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
            triangulation.points.emplace_back(Point2D<coor_t>(x, y));
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

bool ImportNodeAndEdges(const std::string & ne, Point3DContainer & points, std::list<IndexEdge> & edges)
{
    std::ifstream in(ne);
    if(!in.is_open()) return false;

    points.clear();
    edges.clear();

    size_t size, index;
    int64_t x, y, z, v1, v2;
    std::string line, tmp;
    while(!in.eof()){
        line.clear();
        std::getline(in, line);
        if(line.empty()) continue;
        if(str::StartsWith(line, "NODES")){
            std::stringstream ss(line);
            ss >> tmp >> size;
            points.reserve(size);
            while(!in.eof() && size != points.size()){
                std::getline(in, line);
                std::stringstream ss(line);
                ss >> index >> x >> y >> z;
                points.emplace_back(x, y, z);
            }
            std::getline(in, line);
            if(!str::StartsWith(line, "END_OF_NODES")) return false;
        }
        if(str::StartsWith(line, "EDGES")){
            std::stringstream ss(line);
            ss >> tmp >> size;
            while(!in.eof() && size != edges.size()){
                std::getline(in, line);
                std::stringstream ss(line);
                ss >> index >> v1 >> v2;
                edges.emplace_back(v1, v2);
            }
            std::getline(in, line);
            if(!str::StartsWith(line, "END_OF_EDGES")) return false;
        }
    }
    
    in.close();
    return true;
}

bool ExportMshFile(const std::string & msh, const Triangulation<Point2D<coor_t> > & triangulation)
{
    std::ofstream out(msh);
    if(!out.is_open()) return false;

    const auto & points = triangulation.points;
    const auto & vertices = triangulation.vertices;   
    const auto & triangles = triangulation.triangles;

    char sp(32);
    out << sp << vertices.size() << sp << triangles.size() << GENERIC_DEFAULT_EOL;

    out << std::setiosflags(std::ios::right);
    out << std::setiosflags(std::ios::fixed) << std::setprecision(12);

    size_t index = 0;
    out << "NODES" << GENERIC_DEFAULT_EOL;
    for(const auto & vertex : vertices){
        index++;
        const auto & point = points[vertex.index];
        out << "Node:" << std::setw(10) << index << sp;
        for(size_t i = 0; i < Point2D<coor_t>::dim; ++i) {
            out << std::setw(20) << point[i] << sp;
        }
        out << "P" << index << GENERIC_DEFAULT_EOL;
    }
    out << "END_OF_NODES" << GENERIC_DEFAULT_EOL;

    index = 0;
    out << "ELEMENTS" << GENERIC_DEFAULT_EOL;
    for(const auto & triangle : triangles){
        index++;
        const auto & vs = triangle.vertices;
        out << "Element:" << std::setw(10) << index << sp;
        for(size_t i = 0; i < 3; ++i){
            out << std::setw(10) << vs[i] + 1;
        }
        out << std::setw(10) << vs[2] + 1;
        out << sp << "R1" << GENERIC_DEFAULT_EOL;
    }
    out << "END_OF_ELEMENTS" << GENERIC_DEFAULT_EOL;
    out << "END";

    out.close();
    return true;    
}

bool ExportReportFile(const std::string & rpt, const MeshEvaluation2D & evaluation, bool pureText)
{
    std::ofstream out(rpt);
    if(!out.is_open()) return false;

    auto currentTime = std::time(nullptr);
    out << std::asctime(std::localtime(&currentTime)) << GENERIC_DEFAULT_EOL;

    auto printProgress = [&out](float_t progress, size_t len) mutable { for(size_t i = 0; i < progress * len; ++i) out << '#'; };

    out << std::setiosflags(std::ios::left);
    out << std::fixed << std::setprecision(2);
    out << std::setw(16) << "Nodes:" << evaluation.nodes << GENERIC_DEFAULT_EOL;
    out << std::setw(16) << "Elements:" << evaluation.elements << GENERIC_DEFAULT_EOL;
    out << GENERIC_DEFAULT_EOL;

    out << std::setw(16) << "Minimum Angle:" << math::Deg(evaluation.minAngle) << GENERIC_DEFAULT_EOL;
    out << std::setw(16) << "Maximum Angle:" << math::Deg(evaluation.maxAngle) << GENERIC_DEFAULT_EOL;
    out << std::setw(16) << "Distribution:" << GENERIC_DEFAULT_EOL;

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
        out << GENERIC_DEFAULT_EOL;
        start += step;
    }
    out << GENERIC_DEFAULT_EOL;

    out << std::setw(16) << "Minimum Edge:" << evaluation.minEdgeLen << GENERIC_DEFAULT_EOL;
    out << std::setw(16) << "Maximum Edge:" << evaluation.maxEdgeLen << GENERIC_DEFAULT_EOL;
    out << std::setw(16) << "Distribution:" << GENERIC_DEFAULT_EOL;

    colums = evaluation.triEdgeLenHistogram.size();
    step = (evaluation.maxEdgeLen - evaluation.minEdgeLen) / colums;
    start = evaluation.minEdgeLen;
    for(size_t i = 0; i < colums; ++i){
        std::stringstream label;
        label << std::setiosflags(std::ios::right);
        label << std::fixed << std::setprecision(2);
        label << start << '-' << (start + step) << ':';
        out << std::setw(24) << label.str();
        if(pureText) out << (evaluation.triEdgeLenHistogram.at(i) * 100) << "%";
        else printProgress(evaluation.triEdgeLenHistogram.at(i), 100);
        out << GENERIC_DEFAULT_EOL;
        start += step;
    }

    out.close();
    return true;
}

bool ExportNodeAndEdges(const std::string & ne, const Point3DContainer & points, const std::list<IndexEdge> & edges)
{
    //index start from 0
    std::ofstream out(ne);
    if(!out.is_open()) return false;

    char sp(32);
    out << "NODES" << sp << points.size() << GENERIC_DEFAULT_EOL;
    size_t index = 0;
    for(const auto & point : points){
        out << index++ << sp << point[0] << sp << point[1] << sp << point[2] << std::endl;
    }
    out << "END_OF_NODES" << GENERIC_DEFAULT_EOL;
 
    out << "EDGES" << sp << edges.size() << GENERIC_DEFAULT_EOL;    
    index = 0;
    for(const auto & edge : edges){
        out << index++ << sp << edge.v1() << sp << edge.v2() << GENERIC_DEFAULT_EOL;
    }
    out << "END_OF_EDGES" << GENERIC_DEFAULT_EOL;

    out.close();
    return true;
}

bool ExportVtkFile(const std::string & vtk, const TetrahedronData & tet)
{
    return geometry::tet::WriteVtkFile(vtk, tet);
}

bool ExportMshFile(const std::string & msh, const TetrahedronData & tet)
{
    std::ofstream out(msh);
    if(!out.is_open()) return false;

    char sp(32);
    out << "$MeshFormat" << GENERIC_DEFAULT_EOL;
    out << "2.2 0" << sp << sizeof(coor_t) << GENERIC_DEFAULT_EOL;
    out << "$EndMeshFormat" << GENERIC_DEFAULT_EOL;

    out << "$Nodes" << GENERIC_DEFAULT_EOL;
    out << tet.points.size() << GENERIC_DEFAULT_EOL;
    
    size_t index = 0;
    for(const auto & point : tet.points){
        index++;
        out << index << sp <<point[0] << sp << point[1] << sp << point[2] << GENERIC_DEFAULT_EOL;
    }
    out << "$EndNodes" << GENERIC_DEFAULT_EOL;
    out << "$Elements" << GENERIC_DEFAULT_EOL;
    out << tet.tetrahedrons.size() << GENERIC_DEFAULT_EOL;
    index = 0;
    size_t elmType = 4;
    size_t phyType = 0;
    size_t mshPart = 0;
    for(const auto & tetrahedron : tet.tetrahedrons){
        index++;
        out << index << sp << elmType << sp << phyType << sp << mshPart;
        for(size_t i = 0; i < 4; ++i){
            out << sp << tetrahedron.vertices[i];
        }
        out << GENERIC_DEFAULT_EOL; 
    }
    out << "$EndElements" << GENERIC_DEFAULT_EOL;

    out.close();
    return true;
}

}//namespace io
}//namespace emesh