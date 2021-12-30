#ifndef EMESH_IO_MESHIO_H
#define EMESH_IO_MESHIO_H
#include "Mesh2D.h"
#include "Mesh3D.h"
#include <string>
#include <map>
namespace emesh {
namespace io {

bool ImportLayerStackFile(const std::string & stack, StackLayerInfos & infos);
bool ImportDomDmcFiles(const std::string & dom, const std::string & dmc, std::map<int, SPtr<PolygonContainer> > & results);
bool ImportWktFile(const std::string & wkt, PolygonWithHolesContainer & pwhs);
bool ImportMshFile(const std::string & msh, Triangulation<Point2D<coor_t> > & triangulation);
bool ImportNodeAndEdges(const std::string & ne, Point3DContainer & points, std::list<IndexEdge> & edges);
bool ExportMshFile(const std::string & msh, const Triangulation<Point2D<coor_t> > & triangulation);
bool ExportVtkFile(const std::string & vtk, const Triangulation<Point2D<coor_t> > & triangulation);
bool ExportReportFile(const std::string & rpt, const MeshEvaluation2D & evaluation, bool pureText = false);
bool ExportNodeAndEdges(const std::string & ne, const Point3DContainer & points, const std::list<IndexEdge> & edges);
bool ExportVtkFile(const std::string & vtk, const TetrahedronData & tet);
bool ExportMshFile(const std::string & msh, const TetrahedronData & tet);

}//namespace io
}//namespace emesh

#endif//EMESH_IO_MESHIO_H