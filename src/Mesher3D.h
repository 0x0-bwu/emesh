#ifndef EMESH_MESHER3D_H
#define EMESH_MESHER3D_H
#include "generic/geometry/Tetrahedralization.hpp"
#include "generic/math/MathUtility.hpp"
#include "generic/tools/Log.hpp"
#include "Mesh3D.h"
namespace emesh {

struct Mesh3DFlowDB
{
    Mesh3DFlowDB()
     : meshCtrl(new MeshCtrl3D)
    {}
    Mesh3DFlowDB(Mesh3DFlowDB && ) = delete;
    Mesh3DFlowDB(const Mesh3DFlowDB & ) = delete;
    Mesh3DFlowDB & operator= (Mesh3DFlowDB && ) = delete;
    Mesh3DFlowDB & operator= (const Mesh3DFlowDB & ) = delete;

    template<typename T>
    using Data = std::unique_ptr<T>;

    Data<std::string> workPath;
    Data<std::string> projName;
    Data<MeshCtrl3D>       meshCtrl;
    Data<StackLayerInfos>    sInfos;
    Data<TetrahedronData>    tetras;
    Data<StackLayerPolygons> inGoems;
    Data<InterfaceIntersections> intersections;
};

class Mesher3D
{
public:
    using float_t = float_type<coor_t>;

    Mesher3D();
    ~Mesher3D();
    bool Run();
    bool RunTest();
    Mesh3DFlowDB db;
private:
    void InitLogger();
    void CloseLogger();
    bool RunGenerateMesh();
};
}//namespace emesh
 #endif//EMESH_MESHER3D_H