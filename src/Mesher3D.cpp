#include "Mesher3D.h"
#include "MeshFlow3D.h"
#include "MeshFlowMT.h"
using namespace generic;
using namespace emesh;

Mesher3D::Mesher3D()
{
    std::string dataPath = filesystem::CurrentPath() + GENERIC_FOLDER_SEPS + "thirdpart" + GENERIC_FOLDER_SEPS + "internal" + GENERIC_FOLDER_SEPS + "testdata";
    
    // std::string workPath = dataPath + GENERIC_FOLDER_SEPS + "Iluvatar";
    // std::string projName = "Iluvatar";

    // std::string workPath = dataPath + GENERIC_FOLDER_SEPS + "odb";
    // std::string projName = "odb";

    // std::string workPath = dataPath + GENERIC_FOLDER_SEPS + "subgds";
    // std::string projName = "subgds";
  
    std::string workPath = dataPath + GENERIC_FOLDER_SEPS + "fccsp";
    std::string projName = "fccsp";

    // std::string dataPath = filesystem::CurrentPath() + GENERIC_FOLDER_SEPS + "test";
    // std::string workPath = dataPath + GENERIC_FOLDER_SEPS + "cube";
    // std::string projName = "cube";
    
    db.workPath.reset(new std::string(std::move(workPath)));
    db.projName.reset(new std::string(std::move(projName)));
}

Mesher3D::~Mesher3D()
{
}

bool Mesher3D::Run()
{
    if(!db.workPath || !db.projName) return false;

    InitLogger();
    
    auto res = RunGenerateMesh();

    CloseLogger();
    return res;
}

bool Mesher3D::RunTest()
{
    //test
    TetrahedronData tet;
    std::vector<Point3D<coor_t> > points {{0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0}, {0, 0, 10}, {10, 0, 10}, {10, 10, 10}, {0, 10, 10}, {5, 5, 5}};
    std::list<IndexEdge> edges {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7} };
    std::vector<Point3D<coor_t> > addin {{5, 5, 5}};
    auto res = MeshFlow3D::Tetrahedralize(points, edges, nullptr, tet);
    std::cout << "total nodes: " << tet.vertices.size() << std::endl;
    std::cout << "total elements: " << tet.tetrahedrons.size() << std::endl;
    return res;
}

bool Mesher3D::RunGenerateMesh()
{
    bool res(true);
    size_t threads = db.ctrl->threads;

    //
    log::Info("start to load geometries from file...");
    db.model.reset(new StackLayerModel);
    db.model->sInfos.reset(new StackLayerInfos);
    db.model->inGoems.reset(new StackLayerPolygons);
    res = MeshFlow3D::LoadGeometryFiles(*db.workPath, *db.projName, *(db.model->inGoems), *(db.model->sInfos));
    if(!res){
        log::Error("fail to load geometries");
        return false;
    }

    size_t geomCount = 0;
    geometry::Box2D<coor_t> bbox;
    for(const auto & geoms : *(db.model->inGoems)){
        geomCount += geoms.size();
        for(const auto & geom : geoms)
            bbox |= geometry::Extent(geom);
    }
    log::Info("total geometries: %1%", geomCount);
    db.model->bbox = bbox;

    //
    if(db.ctrl->tolerance != 0){
        log::Info("start simplify geometries... , tolerance: %1%", db.ctrl->tolerance);
        res = MeshFlow3DMT::CleanGeometries(*(db.model->inGoems), db.ctrl->tolerance, threads);
        if(!res){
            log::Error("fail to simplify geometries, tolerance might be to large");
            return false;
        }
    }
    
    //
    log::Info("start create sub models...");
    db.model->CreateSubModels();
    // for(size_t i = 0; i < 4; ++i){
    //     db.model->subModels[i]->CreateSubModels();
    // }

    //
    log::Info("start extract interface intersections...");
    res = MeshFlow3DMT::ExtractInterfaceIntersections(*db.model, threads);
    if(!res){
        log::Error("fail to extract interface intersections");
        return false;
    }

    //
    coor_t maxLength = std::max(bbox.Length(), bbox.Width()) / 10;
    log::Info("start split overlength edges... , max length: %1%", maxLength);
    res = MeshFlow3DMT::SplitOverlengthEdges(*db.model, maxLength, threads);
    if(!res){
        log::Error("fail to splitting overlength edges");
        return false;
    }
    
    //
    log::Info("start build mesh sketch models...");
    auto models = std::make_unique<std::vector<MeshSketchModel> >();
    res = MeshFlow3D::BuildMeshSketchModels(*db.model, *models);
    if(!res){
        log::Error("fail to build mesh sketch models");
        return false;
    }

    //
    // log::Info("start insert grade points to mesh sketch layers...");
    // res = MeshFlow3D::AddGradePointsForMeshModels(*models, 100);

    //
    if(math::GT<float_t>(db.ctrl->smartZRatio, 1.0)){
        log::Info("start slice mesh sketch models... , ratio: %1%", db.ctrl->smartZRatio);
        res = MeshFlow3D::SliceOverheightModels(*models, db.ctrl->smartZRatio);
        if(!res){
            log::Error("fail to slice mesh sketch models");
            return false;
        }
    }

    //
    log::Info("start generate mesh per sketch layer...");
    auto tetVec = std::make_unique<TetrahedronDataVec>();
    res = MeshFlow3D::GenerateTetrahedronsFromSketchModels(*models, *tetVec);
    if(!res){
        log::Error("fail to generate mesh per sketch layer");
        return false;
    }

    log::Info("start write layer mesh result files...");
    for(size_t i = 0; i < tetVec->size(); ++i){
        std::string filename = *db.workPath + GENERIC_FOLDER_SEPS + *db.projName + "_" + std::to_string(i + 1) + ".vtk";
        MeshFlow3D::ExportVtkFile(filename, tetVec->at(i));
    }

    //
    log::Info("start merge mesh results...");
    db.tetras.reset(new TetrahedronData);
    res = MeshFlow3D::MergeTetrahedrons(*db.tetras, *tetVec);
    if(!res){
        log::Error("fail to merge mesh results");
        return false;
    }

    log::Info("start write final mesh result file...");
    std::string filename = *db.workPath + GENERIC_FOLDER_SEPS + *db.projName + ".vtk";
    MeshFlow3D::ExportVtkFile(filename, *db.tetras);

    log::Info("total nodes: %1%", db.tetras->vertices.size());
    log::Info("total elements: %1%", db.tetras->tetrahedrons.size());

    return true;
}

void Mesher3D::InitLogger()
{
    std::string dbgFile = *db.workPath + GENERIC_FOLDER_SEPS + *db.projName + ".dbg";
    std::string logFile = *db.workPath + GENERIC_FOLDER_SEPS + *db.projName + ".log";

    auto traceSink = std::make_shared<log::StreamSinkMT>(std::cout);
    auto debugSink = std::make_shared<log::FileSinkMT>(dbgFile);
    auto infoSink  = std::make_shared<log::FileSinkMT>(logFile);
    traceSink->SetLevel(log::Level::Trace);
    debugSink->SetLevel(log::Level::Debug);
    infoSink->SetLevel(log::Level::Info);

    auto logger = log::MultiSinksLogger("eMesh", {traceSink, debugSink, infoSink});
    logger->SetLevel(log::Level::Trace);
    log::SetDefaultLogger(logger);
}

void Mesher3D::CloseLogger()
{
    log::ShutDown();
}
