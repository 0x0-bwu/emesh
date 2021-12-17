#include "Mesher2D.h"
#include "Mesher3D.h"
#include "generic/tools/ProgramOptions.hpp"
#include "generic/tools/Tools.hpp"
#include <iostream>
#include <stdlib.h>
using namespace generic;
using namespace emesh;
struct MeshOptions
{
    bool surfaceMesh = false;
    bool printHelpMsg = false;
    std::string workPath;
    std::string projName;
};

void PrintHelpMessage(std::ostream & os = std::cout)
{
    os << "will add help message soon..." << std::endl;
}

bool ParseOptions(int argc, char *argv[], MeshOptions & mOp, std::ostream & os = std::cout)
{
    using namespace program_options;
    OptionParser op("mesh options");
	auto helpOption = op.Add<Switch>("h", "help", "produce help message");
    auto surfOption = op.Add<Switch>("s", "surface", "planer surface mesh");
    try {
        op.Parse(argc, argv);

        if(helpOption->Count() == 1){
            mOp.printHelpMsg = true;
            return true;
        }
        
        //work path
        auto noOptArgs = op.NonOptionArgs();
        if(noOptArgs.empty()){
            os << "Error: missing argument" << std::endl;
            return false;
        }

        const std::string & path = noOptArgs.back();
        mOp.workPath = filesystem::DirName(path);
        mOp.projName = filesystem::FileName(path);

        if(surfOption->Count() == 1)
            mOp.surfaceMesh = true;
                
        return true;
    }
    catch (const InvalidOption & e)
    {
        os << e.what() << std::endl;
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
        os << "Error: plainer surface mesh currently disabled!" << std::endl;
        return EXIT_FAILURE;
    }
    else{
        auto mesher = std::unique_ptr<Mesher3D>(new Mesher3D);
        mesher->db.workPath.reset(new std::string(options.workPath));
        mesher->db.projName.reset(new std::string(options.projName));
        res = mesher->Run();
    }
    if(res) return EXIT_SUCCESS;
    return EXIT_FAILURE;
};
