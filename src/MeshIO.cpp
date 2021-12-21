#include "MeshIO.h"
using namespace generic;
namespace emesh {
namespace io {

bool LoadDomDmcFiles(const std::string & dom, const std::string & dmc, std::map<int, SPtr<PolygonContainer> > & results)
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
        iter.second->emplace_back(std::move(polygon));
    }
    f_dom.close();
    f_dmc.close();
    return true;
}

bool LoadWktFile(const std::string & wkt, std::list<PolygonWithHoles2D<coor_t> > & pwhs)
{
    pwhs.clear();
    return geometry::GeometryIO::Read<PolygonWithHoles2D<coor_t> >(wkt, std::back_inserter(pwhs));
}

}//namespace io
}//namespace emesh