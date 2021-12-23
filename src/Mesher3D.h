#ifndef EMESH_MESHER3D_H
#define EMESH_MESHER3D_H
#include <boost/core/noncopyable.hpp>
#include "generic/math/MathUtility.hpp"
#include "Mesh3D.h"
#include "Mesher.h"
namespace emesh {
class Mesher3D : public Mesher
{
public:
    Mesher3D();
    ~Mesher3D() = default;
    bool Run();
    bool RunTest();
    std::string GetProjFileName() const;

    Mesh3DB db;
    Mesh3Options options;

private:
    bool RunGenerateMesh();
};
}//namespace emesh
 #endif//EMESH_MESHER3D_H