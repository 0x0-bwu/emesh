#ifndef EMESH_MESHER3D_H
#define EMESH_MESHER3D_H
#include <boost/core/noncopyable.hpp>
#include "generic/geometry/Tetrahedralization.hpp"
#include "generic/math/MathUtility.hpp"
#include "generic/tools/Log.hpp"
#include "Mesh3D.h"
namespace emesh {

struct Mesh3Options
{
    size_t threads;
    Mesh3Ctrl meshCtrl;
    std::string workPath;
    std::string projName;
    FileFormat iFileFormat = FileFormat::WKT;
    FileFormat oFileFormat = FileFormat::VTK;
};

struct Mesh3DB : private boost::noncopyable
{
    Mesh3DB() = default;

    template<typename T>
    using Data = std::unique_ptr<T>;
    
    Data<StackLayerModel>  model;
    Data<TetrahedronData>  tetras;
};

class Mesher3D
{
public:
    using float_t = float_type<coor_t>;

    Mesher3D();
    ~Mesher3D();
    bool Run();
    bool RunTest();

    Mesh3DB db;
    Mesh3Options options;
private:
    void InitLogger();
    void CloseLogger();
    bool RunGenerateMesh();
};
}//namespace emesh
 #endif//EMESH_MESHER3D_H