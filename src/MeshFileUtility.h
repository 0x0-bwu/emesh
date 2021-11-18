#ifndef EMESH_MESHFILEUTILITY_H
#define EMESH_MESHFILEUTILITY_H
#include "MeshCommon.h"
#include "Mesher2D.h"
#include <string>
namespace emesh {

class MeshFileUtility
{
    friend class Mesher2D;
    using coor_t = typename Mesher2D::coor_t;
    using float_t = typename Mesher2D::float_t;
public:
    //2d
    static bool LoadMeshCtrlFile(const std::string & txt, MeshCtrl & ctrl);
    static bool LoadWktFile(const std::string & wkt, float_t scale2Int, std::list<Polygon2D<coor_t> > & polygons);
    static bool LoadWktFile(const std::string & wkt, std::list<PolygonWithHoles2D<coor_t> > & pwhs);//wbtest
    static bool LoadDomDmcFiles(const std::string & dom, const std::string & dmc, float_t scale2Int, std::list<Polygon2D<coor_t> > & polygons);
    static bool ImportMshFile(const std::string & msh, Triangulation<Point2D<coor_t> > & triangulation, float_t scale2Int);
    static bool ExportMshFile(const std::string & msh, const Triangulation<Point2D<coor_t> > & triangulation, float_t scale);
    static bool ExportReportFile(const std::string & rpt, const Mesher2D::MeshEvaluation & evaluation, float_t scale, bool pureText = false);

    //3d
    static bool LoadLayerStackInfos(const std::string & filename, StackLayerInfos & infos);

};

}//namespace emesh
#endif//EMESH_MESHFILEUTILITY_H