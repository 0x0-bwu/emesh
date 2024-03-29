#pragma once
#include "MeshCommon.h"
#include "Mesher2D.h"
#include <string>
#include <map>
namespace emesh {
class MeshFileUtility
{
    friend class Mesher2D;
    using float_t = typename Mesher2D::float_t;
public:
    //2d
    static bool LoadMeshCtrlFile(const std::string & txt, Mesh2Ctrl & ctrl);
    static bool LoadWktFile(const std::string & wkt, float_t scale2Int, PolygonContainer & polygons);
    static bool LoadDomDmcFiles(const std::string & dom, const std::string & dmc, float_t scale2Int, PolygonContainer & polygons);
    static bool ImportMshFile(const std::string & msh, Triangulation<Point2D<coor_t> > & triangulation, float_t scale2Int);
    static bool ExportMshFile(const std::string & msh, const Triangulation<Point2D<coor_t> > & triangulation, float_t scale);
    static bool ExportReportFile(const std::string & rpt, const Mesher2D::MeshEvaluation & evaluation, float_t scale, bool pureText = false);
};
}//namespace emesh