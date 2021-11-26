#include "Mesh3D.h"
using namespace generic;
using namespace emesh;

void MeshSketchLayer::SetConstrains(std::shared_ptr<Segment2DContainer> top, std::shared_ptr<Segment2DContainer> bot)
{
    constrains[0] = top;
    constrains[1] = bot;
}

void MeshSketchLayer::SetTopBotHeight(coor_t tH, coor_t bH)
{
    height[0] = tH;
    height[1] = bH;
}

coor_t MeshSketchLayer::GetHeight() const
{
    return std::abs(height[0] - height[1]);
}

std::unique_ptr<Point3DContainer> MeshSketchLayer::GetAdditionalPoints() const
{
    Point3DContainer points;
    if(addPoints[0]){
        points.reserve(addPoints[0]->size());
        for(const auto & p : *addPoints[0])
            points.push_back(Point3D<coor_t>(p[0], p[1], height[0]));
    }
    if(addPoints[1]){
        points.reserve(points.size() + addPoints[1]->size());
        for(const auto & p : *addPoints[1])
            points.push_back(Point3D<coor_t>(p[0], p[1], height[1]));
    }
    if(0 == points.size()) return nullptr;
    return std::make_unique<Point3DContainer>(points);   
}


std::pair<MeshSketchLayer, MeshSketchLayer> MeshSketchLayer::Slice() const
{
    MeshSketchLayer top = *this;
    MeshSketchLayer bot = *this;
    auto mid = (height[0] + height[1]) / 2;
    top.height[1] = mid;
    bot.height[0] = mid;
    bot.addPoints[0] = addPoints[1];
    bot.constrains[0] = constrains[1];//C0-C1 -> C0-C1-C1
    return std::make_pair(top, bot);
}

TetrahedronDataMerger::TetrahedronDataMerger(TetrahedronData & data)
 : m_data(data)
{
    BuildIndexMap();
}

void TetrahedronDataMerger::BuildIndexMap()
{
    for(size_t v = 0; v < m_data.vertices.size(); ++v){
        const auto & p = m_data.points[m_data.vertices[v].index];
        if(!m_indexMap.count(p[2]))
            m_indexMap.insert(std::make_pair(p[2], PointIndexMap{}));
        auto & vertexIndexMap = m_indexMap[p[2]];
        vertexIndexMap.insert(std::make_pair(Point2D<coor_t>(p[0], p[1]), v));
    }
}

void TetrahedronDataMerger::Merge(const TetrahedronData & data)
{
    using Vertex = typename TetrahedronData::Vertex;
    std::unordered_set<size_t> exists;
    std::unordered_map<size_t, size_t> map2this;
    for(size_t v = 0; v < data.vertices.size(); ++v){
        const auto & p = data.points[data.vertices[v].index];
        if(!m_indexMap.count(p[2]))
            m_indexMap.insert(std::make_pair(p[2], PointIndexMap{}));
        auto & vertexIndexMap = m_indexMap[p[2]];
        auto p2d = Point2D<coor_t>(p[0], p[1]);
        auto it = vertexIndexMap.find(p2d);
        if(it != vertexIndexMap.end()){
            exists.insert(v);
            map2this.insert(std::make_pair(v, it->second));
        }
        else{
            size_t pIndex = m_data.points.size();
            m_data.points.push_back(p);
            size_t vIndex = m_data.vertices.size();
            m_data.vertices.push_back(Vertex{});
            m_data.vertices.back().index = pIndex;
            map2this.insert(std::make_pair(v, vIndex));
        }
    }
    size_t tetOffset = m_data.tetrahedrons.size();
    m_data.tetrahedrons.insert(m_data.tetrahedrons.end(), data.tetrahedrons.begin(), data.tetrahedrons.end());
    for(size_t v = 0; v < data.vertices.size(); ++v){
        const auto & origin = data.vertices[v];
        auto & vertex = m_data.vertices[map2this[v]];
        for(auto tet : origin.tetrahedrons){
            vertex.tetrahedrons.insert(tet + tetOffset);
        }
    }

    for(size_t t = tetOffset; t < m_data.tetrahedrons.size(); ++t){
        auto & tetrahedron = m_data.tetrahedrons[t];
        for(size_t i = 0; i < 4; ++i)
            tetrahedron.vertices[i] = map2this[tetrahedron.vertices[i]];
        for(size_t i = 0; i < 4; ++i){
            if(tetrahedron.neighbors[i] != tet::noNeighbor)
                tetrahedron.neighbors[i] += tetOffset;
            else {
                //todo connect with this
            }
        }        
    }

    for(const auto & edge : data.fixedEdges){
        m_data.fixedEdges.insert(IndexEdge(map2this[edge.v1()], map2this[edge.v2()]));
    }
}