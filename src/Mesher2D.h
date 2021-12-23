#ifndef EMESH_MESHER2D_H
#define EMESH_MESHER2D_H
#include "Mesh2D.h"
#include "Mesher.h"
namespace emesh {

class Mesher2D : public Mesher
{
public:
    Mesher2D() = default;
    ~Mesher2D() = default;
    bool Run();
    bool RunTest();
    std::string GetProjFileName() const;

    Mesh2DB db;
    Mesh2Options options;
private:
    bool RunGenerateMesh();
    bool RunMeshEvaluation();
};
}//namespace emesh
 #endif//EMESH_MESHER2D_H