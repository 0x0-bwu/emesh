#include "Mesher2D.h"
#include "Mesher3D.h"
#include "generic/tools/ProgramOptions.hpp"
#include "generic/tools/FileSystem.hpp"
#include "generic/tools/Format.hpp"
#include "generic/tools/Tools.hpp"
#include <iostream>
#include <stdlib.h>
using namespace generic;
using namespace emesh;
struct MeshOptions
{
    bool testFlow = false;
    bool surfaceMesh = false;
    bool printHelpMsg = false;
    int threads = 1;
    int partLvl = 0;
    int maxGradeLvl = 0;
    int tolerance = 0;
    int maxEdgeLen = std::numeric_limits<int>::max();
    int refineIte = 0;
    double smartZRatio = 1.0;
    double minAlpha = 15;//deg
    std::string workPath;
    std::string projName;
    FileFormat iFileFormat = FileFormat::DomDmc;
    FileFormat oFileFormat = FileFormat::MSH;

    void SetMesh2Options(Mesh2Options & op) const
    {
        op.threads = threads;
        op.maxGradeLvl = maxGradeLvl;
        op.workPath = workPath;
        op.projName = projName;
        op.iFileFormat = iFileFormat;
        op.oFileFormat = oFileFormat;
        op.meshCtrl.minAlpha = math::Rad(minAlpha);
        op.meshCtrl.refineIte = refineIte;
        op.meshCtrl.maxEdgeLen = maxEdgeLen;
    }

    void SetMesh3Options(Mesh3Options & op) const
    {
        op.threads = threads;
        op.partLvl = partLvl;
        op.maxGradeLvl = maxGradeLvl;
        op.workPath = workPath;
        op.projName = projName;
        op.iFileFormat = iFileFormat;
        op.oFileFormat = oFileFormat;
        op.meshCtrl.smartZRatio = smartZRatio;
        op.meshCtrl.tolerance = tolerance;
        op.meshCtrl.maxEdgeLenH = maxEdgeLen;
    }
};

bool ParseOptions(int argc, char *argv[], MeshOptions & mOp, std::ostream & os = std::cout)
{
    using namespace program_options;
    OptionParser op("mesh options");
    auto testOption = op.Add<Switch>("b", "beta", "test flow");
	auto helpOption = op.Add<Switch>("h", "help", "produce help message");
    auto surfOption = op.Add<Switch>("s", "surface", "planer surface mesh");
    auto ifmtOption = op.Add<Value<std::string> >("i", "input", "input file format", "dmcdom");
    auto ofmtOption = op.Add<Value<std::string> >("o", "output", "output file format", "vtk");
    auto jobsOption = op.Add<Value<int> >("j", "jobs", "cpu core numbers in use", 1);
    auto partOption = op.Add<Value<int> >("p", "partition", "partition level(3d)", 0);
    auto grdeOption = op.Add<Value<int> >("g", "gradelevel", "grade level", 0);
    auto smtZOption = op.Add<Value<double> >("z", "ratioz", "vertical slice ratio(3d)", 1.0);
    auto toleOption = op.Add<Value<int> >("t", "tolerance", "tolerance to simplify input geometries", 0);
    auto iterOption = op.Add<Value<int> >("r", "refine", "mesh refine iteration(2d)", 0);
    auto aphaOption = op.Add<Value<double> >("a", "alpha", "minimum angle(unit: deg) of refinement(2d)", 15);
    auto maxEOption = op.Add<Value<int> >("l", "maxlen", "max edge length", 0);
    try {
        op.Parse(argc, argv);

        //test
        if(testOption->isSet()){
            mOp.testFlow = true;
        }

        //help
        if(helpOption->isSet()){
            mOp.printHelpMsg = true;
            os << op.Help();
            return true;
        }
        
        //work path
        auto noOptArgs = op.NonOptionArgs();
        if(noOptArgs.empty()){
            os << "Error: missing argument" << GENERIC_DEFAULT_EOL;
            return false;
        }

        const std::string & path = noOptArgs.back();
        mOp.workPath = filesystem::DirName(path);
        mOp.projName = filesystem::FileName(path);

        //dimension
        if(surfOption->isSet())
            mOp.surfaceMesh = true;
        
        //input format
        if(ifmtOption->isSet()){
            auto fmt = ifmtOption->GetValue();
            if("domdmc" == fmt || "dmcdom" == fmt) mOp.iFileFormat = FileFormat::DomDmc;
            else if("wkt" == fmt) mOp.iFileFormat = FileFormat::WKT;
            else{
                os << format::Format2String("Error: unsupported input file format: %1%", fmt) << GENERIC_DEFAULT_EOL;
                return false;
            }
        }

        //output format
        if(ofmtOption->isSet()){
            auto fmt = ofmtOption->GetValue();
            if("msh" == fmt) mOp.oFileFormat = FileFormat::MSH;
            else if("vtk" == fmt) mOp.oFileFormat = FileFormat::VTK;
            else{
                os << format::Format2String("Error: unsupported output file format: %1%", fmt) << GENERIC_DEFAULT_EOL;
                return false;
            }
        }

        //threads
        if(jobsOption->isSet()){
            int num = jobsOption->GetValue();
            if(0 < num && num < 256) mOp.threads = num;
        }

        //partition level
        if(partOption->isSet()){
            int num = partOption->GetValue();
            if(0 < num && num < 256) mOp.partLvl = num;
        }
        
        //grade level
        if(grdeOption->isSet()){
            int num = grdeOption->GetValue();
            if(0 < num && num < 256) mOp.maxGradeLvl = num;
        }

        //vertical slice ratio
        if(smtZOption->isSet()){
            double ratio = smtZOption->GetValue();
            if(ratio > 1.0) mOp.smartZRatio = ratio;
        }

        //tolerance
        if(toleOption->isSet()){
            int tolerance = toleOption->GetValue();
            if(tolerance > 0) mOp.tolerance = tolerance;
        }

        //refine
        if(iterOption->isSet()){
            int refineIte = iterOption->GetValue();
            if(refineIte > 0) mOp.refineIte = refineIte;
        }

        //minimum angle
        if(aphaOption->isSet()){
            double alpha = aphaOption->GetValue();
            if(0 < alpha && alpha < 30) mOp.minAlpha = alpha;
        }

        //max edge length
        if(maxEOption->isSet()){
            double len = maxEOption->GetValue();
            if(len > 0) mOp.maxEdgeLen = len;
        }
        return true;
    }
    catch (const InvalidOption & e)
    {
        os << e.what() << GENERIC_DEFAULT_EOL;
        os << "Error:";
        if(e.error == InvalidOption::Error::MissingArgument)
            os << "missing argument";
        else if(e.error == InvalidOption::Error::InvalidArgument)
            os << "invalid argument";
        else if(e.error == InvalidOption::Error::TooManyArgument)
            os << "too many arguments";
        else if(e.error == InvalidOption::Error::MissingOption)
            os << "missing option";
        os << std::endl;
        return false;
    }
    catch (const std::exception & e)
    {
        os << e.what() << std::endl;
        return false;
    }
    return false;
}

int main(int argc, char *argv[])
{
    tools::ProgressTimer t;

    MeshOptions options;
    auto & os = std::cout;
    if(!ParseOptions(argc, argv, options, os))
        return EXIT_FAILURE;

    if(options.printHelpMsg){
        return EXIT_SUCCESS;
    }

    bool res = true;
    if(options.surfaceMesh){
        auto mesher = std::unique_ptr<Mesher2D>(new Mesher2D);
        options.SetMesh2Options(mesher->options);
        if(options.testFlow)
            res = mesher->RunTest();
        else res = mesher->Run();
    }
    else{
        auto mesher = std::unique_ptr<Mesher3D>(new Mesher3D);
        options.SetMesh3Options(mesher->options);
        if(options.testFlow)
            res = mesher->RunTest();
        else res = mesher->Run();
    }

    if(res) return EXIT_SUCCESS;
    return EXIT_FAILURE;
};
