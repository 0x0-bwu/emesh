#include "Mesher3D.h"
#include "generic/tools/FileSystem.hpp"
#include "MeshFlow3D.h"
using namespace generic;
using namespace emesh;

Mesher3D::Mesher3D()
{
}

bool Mesher3D::Run()
{
    if(GetProjFileName().empty()) return false;

    InitLogger();
    
    auto res = RunGenerateMesh();

    CloseLogger();
    return res;
}

std::string Mesher3D::GetProjFileName() const
{
    if(options.workPath.empty() || options.projName.empty()) return std::string{};
    return options.workPath + GENERIC_FOLDER_SEPS + options.projName;
}

bool Mesher3D::RunTest()
{
    //test
    TetrahedronData tet;
    std::vector<Point3D<coor_t> > points {{0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0}, {0, 0, 10}, {10, 0, 10}, {10, 10, 10}, {0, 10, 10}, {5, 5, 5}};
    std::list<std::vector<size_t> > faces {{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}};
    std::list<IndexEdge> edges {{0, 8}, {1, 8}, {2, 8}, {3, 8}};
    auto res = MeshFlow3D::Tetrahedralize(points, faces, edges, tet);
    std::cout << "total nodes: " << tet.vertices.size() << GENERIC_DEFAULT_EOL;
    std::cout << "total elements: " << tet.tetrahedrons.size() << std::endl;
    return res;
}

bool Mesher3D::RunGenerateMesh()
{
    bool res = true;
    std::string filename = GetProjFileName();
    size_t threads = std::max<size_t>(1, options.threads);
    log::Info("3d mesh config:");
    log::Info("path: %1%", filename);
    log::Info("threads: %1%", threads);

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

    //wbtest
    db.model->bbox = bbox;
    for(auto layer : *(db.model->inGeoms)){
        if(layer) layer->emplace_back(toPolygon(bbox));
    }
    //wbtest

    //
    if(options.meshCtrl.tolerance != 0){
        log::Info("start simplify geometries... , tolerance: %1%", options.meshCtrl.tolerance);
        res = MeshFlow3DMT::CleanGeometries(*(db.model->inGeoms), options.meshCtrl.tolerance, threads);
        if(!res){
            log::Error("failed to simplify geometries, tolerance might be to large");
            return false;
        }
    }
    
    //
    if(options.partLvl > 0){
        log::Info("start create sub models..., level=%1%", options.partLvl);
        StackLayerModel::CreateSubModels(*db.model, options.partLvl);
    }

    std::vector<Ptr<StackLayerModel> > subModels;
    StackLayerModel::GetAllLeafModels(*db.model, subModels);

    //
    log::Info("start extract models intersections...");
    res = MeshFlow3DMT::ExtractModelsIntersections(subModels, threads);
    if(!res){
        log::Error("failed to extract models intersections");
        return false;
    }

    //
    coor_t maxLength = std::max(bbox.Length(), bbox.Width()) / 10;
    coor_t minLength = std::min(bbox.Length(), bbox.Width()) / 5000;
    options.meshCtrl.maxEdgeLenH = std::min(maxLength, options.meshCtrl.maxEdgeLenH);
    options.meshCtrl.maxEdgeLenH = std::max(minLength, options.meshCtrl.maxEdgeLenH);
    log::Info("start split over length edges..., length: %1%", options.meshCtrl.maxEdgeLenH);
    res = MeshFlow3D::SplitOverlengthModelsIntersections(subModels, options.meshCtrl.maxEdgeLenH);
    if(!res){
        log::Error("failed to split over length edges");
        return false;
    }

    if(options.maxGradeLvl > 0){
        log::Info("start insert grade points to mesh sketch layers..., level: %1%", options.maxGradeLvl);
        res = MeshFlow3D::AddLayerModelsGradePoints(subModels, options.maxGradeLvl);
        if(!res){
            log::Error("failed to insert grade points to mesh sketch layers...");
            return false; 
        }
    }

    //
    log::Info("start build mesh sketch models...");
    auto models = std::make_unique<std::vector<MeshSketchModel> >();
    res = MeshFlow3D::BuildMeshSketchModels(subModels, *models);
    if(!res){
        log::Error("failed to build mesh sketch models");
        return false;
    }
    log::Info("total mesh sketch models: %1%", models->size());

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
    log::Info("start generate mesh per model per sketch layer...");
    auto tetVec = std::make_unique<TetrahedronDataVec>();
    res = MeshFlow3D::GenerateTetrahedronVecFromSketchModels(*models, *tetVec, options.meshCtrl);
    if(!res){
        log::Error("failed to generate mesh per model per sketch layer");
        return false;
    }
    db.model.reset();

    // log::Info("start write layer mesh result files...");
    // for(size_t i = 0; i < tetVec->size(); ++i){
    //     std::string layerFile = filename + "_" + std::to_string(i + 1);
    //     MeshFlow3D::ExportResultFile(layerFile, options.oFileFormat, tetVec->at(i));
    // }

    //
    log::Info("start merge mesh results...");
    db.tetras.reset(new TetrahedronData);
    res = MeshFlow3D::MergeTetrahedrons(*db.tetras, *tetVec);
    if(!res){
        log::Error("failed to merge mesh results");
        return false;
    }

    log::Info("start write final mesh result file..., output file format: %1%", toString(options.oFileFormat));
    MeshFlow3D::ExportResultFile(filename, options.oFileFormat, *db.tetras);

    log::Info("total nodes: %1%", db.tetras->vertices.size());
    log::Info("total elements: %1%", db.tetras->tetrahedrons.size());

    return true;
}

