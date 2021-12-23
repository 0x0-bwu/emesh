#ifndef EMESH_MESHER_H
#define EMESH_MESHER_H
#include "generic/tools/Log.hpp"
#include <string>
namespace emesh {

class Mesher
{
public:
    Mesher() = default;
    virtual ~Mesher() = default;
    virtual bool Run() = 0;
    virtual bool RunTest() = 0;
    virtual std::string GetProjFileName() const = 0;

protected:
    void InitLogger();
    void CloseLogger();
};

}//namespace emesh

#endif//EMESH_MESHER_H