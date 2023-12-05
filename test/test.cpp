#include "Mesher2D.h"
#include "Mesher3D.h"
#include "generic/tools/ProgramOptions.hpp"
#include "generic/tools/Format.hpp"
#include "generic/tools/Tools.hpp"
#include <iostream>
#include <stdlib.h>
using namespace generic;
using namespace emesh;
struct MeshOptions
{
    size_t threads = 1;
    bool testFlow = false;
    bool surfaceMesh = false;
    bool printHelpMsg = false;
    std::string workPath;
    std::string projName;
    FileFormat iFileFormat = FileFormat::WKT;
    FileFormat oFileFormat = FileFormat::VTK;

    void SetMesh2Options(Mesh2Options & op) const
    {
        //todo
    }

    void SetMesh3Options(Mesh3Options & op) const
    {
        op.threads = threads;
        // op.workPath = workPath;//wbtest
        // op.projName = projName;//wbtest
        op.iFileFormat = iFileFormat;
        op.oFileFormat = oFileFormat;
    }
};

void PrintHelpMessage(std::ostream & os = std::cout)
{
    os << "will add help message soon..." << std::endl;
}

bool ParseOptions(int argc, char *argv[], MeshOptions & mOp, std::ostream & os = std::cout)
{
    using namespace program_options;
    OptionParser op("mesh options");
    auto testOption = op.Add<Switch>("t", "test", "test flow");
	auto helpOption = op.Add<Switch>("h", "help", "produce help message");
    auto surfOption = op.Add<Switch>("s", "surface", "planer surface mesh");
    auto ifmtOption = op.Add<Value<std::string> >("i", "input", "input file format", "wkt");
    auto ofmtOption = op.Add<Value<std::string> >("o", "output", "output file format", "vtk");
    auto jobsOption = op.Add<Value<int> >("j", "jobs", "cpu core numbers in use", 1);
    try {
        op.Parse(argc, argv);

        //test
        if(testOption->isSet()){
            mOp.testFlow = true;
        }

        //help
        if(helpOption->isSet()){
            mOp.printHelpMsg = true;
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
            if("domdmc" == fmt || "dmcdom == fmt") mOp.iFileFormat = FileFormat::DomDmc;
            else if("wkt" == fmt) mOp.iFileFormat = FileFormat::WKT;
            else{
                os << fmt::Fmt2Str("Error: unsupported input file format: %1%", fmt) << GENERIC_DEFAULT_EOL;
                return false;
            }
        }

        //output format
        if(ofmtOption->isSet()){
            auto fmt = ifmtOption->GetValue();
            if("msh" == fmt) mOp.oFileFormat = FileFormat::MSH;
            else if("vtk" == fmt) mOp.oFileFormat = FileFormat::VTK;
            else{
                os << fmt::Fmt2Str("Error: unsupported output file format: %1%", fmt) << GENERIC_DEFAULT_EOL;
                return false;
            }
        }

        //threads
        if(jobsOption->isSet()){
            int num = jobsOption->GetValue();
            if(num > 0) mOp.threads = static_cast<size_t>(num);
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
        PrintHelpMessage(os);
        return EXIT_SUCCESS;
    }

    bool res = true;
    if(options.surfaceMesh){
        auto mesher = std::unique_ptr<Mesher2D>(new Mesher2D);
        options.SetMesh2Options(mesher->options);
        res = mesher->Run();
        // os << "Error: plainer surface mesh currently disabled!" << std::endl;
        // return EXIT_FAILURE;
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
