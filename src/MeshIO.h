#pragma once
#include "MeshCommon.h"
#include <string>
#include <map>
namespace emesh {
namespace io {

bool LoadDomDmcFiles(const std::string & dom, const std::string & dmc, std::map<int, SPtr<PolygonContainer> > & results);
bool LoadWktFile(const std::string & wkt, PolygonWithHolesContainer & pwhs);

}//namespace io
}//namespace emesh