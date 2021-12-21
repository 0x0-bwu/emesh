#ifndef EMESH_IO_MESHIO_H
#define EMESH_IO_MESHIO_H
#include "MeshCommon.h"
#include <string>
#include <map>
namespace emesh {
namespace io {

bool LoadDomDmcFiles(const std::string & dom, const std::string & dmc, std::map<int, SPtr<PolygonContainer> > & results);
bool LoadWktFile(const std::string & wkt, PolygonWithHolesContainer & pwhs);

}//namespace io
}//namespace emesh

#endif//EMESH_IO_MESHIO_H