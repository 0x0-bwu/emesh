#include "Mesher3D.h"
#include "MeshFlow3D.h"
#include "MeshFlowMT.h"
using namespace generic;
using namespace emesh;

Mesher3D::Mesher3D()
{
    std::string dataPath = fs::CurrentPath() + 
                            GENERIC_FOLDER_SEPS + "thirdpart" + 
                            GENERIC_FOLDER_SEPS + "internal" + 
                            GENERIC_FOLDER_SEPS + "testdata" +
                            GENERIC_FOLDER_SEPS + "wkt";

    // std::string workPath = dataPath + GENERIC_FOLDER_SEPS + "Iluvatar";
    // std::string projName = "Iluvatar";

    // std::string workPath = dataPath + GENERIC_FOLDER_SEPS + "odb";
    // std::string projName = "odb";

    // std::string workPath = dataPath + GENERIC_FOLDER_SEPS + "subgds";
    // std::string projName = "subgds";
  
    std::string workPath = dataPath + GENERIC_FOLDER_SEPS + "fccsp";
    std::string projName = "fccsp";

    // dataPath = filesystem::CurrentPath() + GENERIC_FOLDER_SEPS + "test";
    // std::string workPath = dataPath + GENERIC_FOLDER_SEPS + "cube";
    // std::string projName = "cube";
    
    options.workPath = workPath;
    options.projName = projName;//wbtest
}

Mesher3D::~Mesher3D()
{
}

bool Mesher3D::Run()
{
    if(options.workPath.empty() || options.projName.empty()) return false;

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
    std::list<std::vector<size_t> > faces {{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}};
    std::list<IndexEdge> edges {{0, 8}, {1, 8}, {2, 8}, {3, 8}};
    auto res = MeshFlow3D::Tetrahedralize(points, faces, edges, tet);
    generic::log::Trace("total nodes: %1%", tet.vertices.size());
    generic::log::Trace("total elements: %1%", tet.tetrahedrons.size());
    return res;
}

bool Mesher3D::RunGenerateMesh()
{
    bool res = true;
    std::string filename = options.workPath + GENERIC_FOLDER_SEPS + options.projName;
    //
    log::Info("start to load geometries from file...");
    db.model.reset(new StackLayerModel);
    db.model->sInfos.reset(new StackLayerInfos);
    db.model->inGeoms.reset(new StackLayerPolygons);
    res = MeshFlow3D::LoadGeometryFiles(filename, options.iFileFormat, *(db.model->inGeoms), *(db.model->sInfos));
    if(!res){
        log::Error("failed to load geometries");
        return false;
    }

    size_t geomCount = 0;
    geometry::Box2D<coor_t> bbox;
    for(auto geoms : *(db.model->inGeoms)){
        geomCount += geoms->size();
        for(const auto & geom : *geoms)
            bbox |= geometry::Extent(geom);
    }
    log::Info("total geometries: %1%", geomCount);
    db.model->bbox = bbox;

    //
    if(options.meshCtrl.tolerance != 0){
        log::Info("start simplify geometries... , tolerance: %1%", options.meshCtrl.tolerance);
        res = MeshFlow3DMT::CleanGeometries(*(db.model->inGeoms), options.meshCtrl.tolerance, options.threads);
        if(!res){
            log::Error("failed to simplify geometries, tolerance might be to large");
            return false;
        }
    }
    
    //
    size_t level = 0;
    log::Info("start create sub models..., level=%1%", level);
    StackLayerModel::CreateSubModels(*db.model, level);

    std::vector<StackLayerModel *> subModels;
    StackLayerModel::GetAllLeafModels(*db.model, subModels);
    //
    log::Info("start extract models intersections...");
    res = MeshFlow3DMT::ExtractModelsIntersections(subModels, options.threads);
    if(!res){
        log::Error("failed to extract models intersections");
        return false;
    }

    coor_t maxLength = std::max(bbox.Length(), bbox.Width()) / 10;
    log::Info("start split overlength edges... , max length: %1%", maxLength);
    res = MeshFlow3DMT::SplitOverlengthEdges(*db.model, maxLength, options.threads);
    if(!res){
        log::Error("failed to splitting overlength edges");
        return false;
    }
    
    //
    log::Info("start build mesh sketch models...");
    auto models = std::make_unique<std::vector<MeshSketchModel> >();
    res = MeshFlow3D::BuildMeshSketchModels(*db.model, *models);
    if(!res){
        log::Error("failed to build mesh sketch models");
        return false;
    }
    log::Info("total mesh sketch models: %1%", models->size());

    //
    log::Info("start insert grade points to mesh sketch layers...");
    res = MeshFlow3D::AddGradePointsForMeshModels(*models, 100);

    //
    if(math::GT<float_t>(options.meshCtrl.smartZRatio, 1.0)){
        log::Info("start slice mesh sketch models... , ratio: %1%", options.meshCtrl.smartZRatio);
        res = MeshFlow3D::SliceOverheightModels(*models, options.meshCtrl.smartZRatio);
        if(!res){
            log::Error("failed to slice mesh sketch models");
            return false;
        }
    }

    //
    log::Info("start generate mesh per sketch layer...");
    auto tetVec = std::make_unique<TetrahedronDataVec>();
    if(0 == level) res = MeshFlow3D::GenerateTetrahedronVecFromSketchModel(models->front(), *tetVec);
    else res = MeshFlow3D::GenerateTetrahedronVecFromSketchModels(*models, *tetVec);
    if(!res){
        log::Error("failed to generate mesh per sketch layer");
        return false;
    }
    db.model.reset();

    log::Info("start write layer mesh result files...");
    for(size_t i = 0; i < tetVec->size(); ++i){
        std::string filename = options.workPath + GENERIC_FOLDER_SEPS + options.projName + "_" + std::to_string(i + 1) + ".vtk";
        MeshFlow3D::ExportVtkFile(filename, tetVec->at(i));
    }

    //
    log::Info("start merge mesh results...");
    db.tetras.reset(new TetrahedronData);
    res = MeshFlow3D::MergeTetrahedrons(*db.tetras, *tetVec);
    if(!res){
        log::Error("failed to merge mesh results");
        return false;
    }

    log::Info("start write final mesh result file...");
    std::string vtkFile = filename + ".vtk";
    MeshFlow3D::ExportVtkFile(vtkFile, *db.tetras);

    log::Info("total nodes: %1%", db.tetras->vertices.size());
    log::Info("total elements: %1%", db.tetras->tetrahedrons.size());

    return true;
}

void Mesher3D::InitLogger()
{
    std::string dbgFile = options.workPath + GENERIC_FOLDER_SEPS + options.projName + ".dbg";
    std::string logFile = options.workPath + GENERIC_FOLDER_SEPS + options.projName + ".log";

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
