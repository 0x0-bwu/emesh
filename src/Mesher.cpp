#include "Mesher.h"
using namespace generic;
using namespace emesh;
void Mesher::InitLogger()
{
    std::string dbgFile = GetProjFileName() + ".dbg";
    std::string logFile = GetProjFileName() + ".log";

    auto traceSink = std::make_shared<log::StreamSinkMT>(std::cout);
    auto debugSink = std::make_shared<log::FileSinkMT>(dbgFile);
    auto infoSink  = std::make_shared<log::FileSinkMT>(logFile);
    traceSink->SetLevel(log::Level::Trace);
    debugSink->SetLevel(log::Level::Debug);
    infoSink->SetLevel(log::Level::Info);

    auto logger = log::MultiSinksLogger(std::string{}, {traceSink, debugSink, infoSink});
    logger->SetLevel(log::Level::Trace);
    log::SetDefaultLogger(logger);
}

void Mesher::CloseLogger()
{
    log::ShutDown();
}