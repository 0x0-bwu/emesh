#include "Mesher2D.h"
#include "Mesher3D.h"
#include "generic/tools/ProgramOptions.hpp"
#include "generic/tools/Tools.hpp"
#include <iostream>
#include <stdlib.h>
using namespace generic;
using namespace emesh;
bool ParseOptions(int argc, char *argv[], Mesh2DFlowDB & db, std::ostream & os = std::cout)
{
    using namespace program_options;
    OptionParser op("Mesh options");
	auto helpOption = op.Add<Switch>("h", "help", "produce help message");

    try {
        op.Parse(argc, argv);
        
        auto noOptArgs = op.NonOptionArgs();
        if(noOptArgs.empty()){
            os << "Error: missing argument" << std::endl;
            return false;
        }

        const std::string & path = noOptArgs.back();
        db.workPath.reset(new std::string(filesystem::DirName(path)));
        db.projName.reset(new std::string(filesystem::FileName(path)));
        db.tasks.reset(new MeshTasks);
        db.tasks->push(MeshTask::MeshGeneration);
        db.tasks->push(MeshTask::MeshEvaluation);
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
    return true;
}

int main(int argc, char *argv[])
{
    tools::ProgressTimer t;
    // auto mesher = std::unique_ptr<Mesher2D>(new Mesher2D);
    // if(!ParseOptions(argc, argv, mesher->db, std::cout)) return EXIT_FAILURE;

    auto mesher = std::unique_ptr<Mesher3D>(new Mesher3D);

    auto res = mesher->Run();
    if(res) return EXIT_SUCCESS;
    return EXIT_FAILURE;
};
