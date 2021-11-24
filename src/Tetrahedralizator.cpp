#include "Tetrahedralizator.h"
#include "tetgen/tetgen.h"
using namespace emesh;
bool Tetrahedralizator::Tetrahedralize(const std::vector<Point> & points, const std::list<Edge> & edges, const std::vector<Point> * addin)
{
    tet.Clear();

    size_t index;
    tetgenbehavior b;
    tetgenio in, add, out, bgmin;

    b.plc = 1;//-p
    b.convex = 1;//-c
    b.vtkview = 1;//-k

    in.mesh_dim = 3;
    in.numberofpointattributes = 0;
    in.numberofpoints = points.size();
    in.pointlist = new double[in.numberofpoints * 3];
    
    index = 0;
    for(size_t i = 0; i < in.numberofpoints; ++i)
        for(size_t j = 0; j < in.mesh_dim; ++j)
            in.pointlist[index++] = points[i][j];
    
    in.firstnumber = 0;
    in.numberofedges = edges.size();
    in.edgelist = new int[in.numberofedges * 2];
    index = 0;
    for(const auto & edge : edges){
        in.edgelist[index++] = edge.v1();
        in.edgelist[index++] = edge.v2();
    }

    tetgenmesh m;
    tetrahedralize(&b, &in, &out, &add, &bgmin, &m);

    assert(out.firstnumber == 0);

    index = 0;
    tet.points.resize(out.numberofpoints);
    tet.vertices.resize(out.numberofpoints);
    for(size_t i = 0; i < out.numberofpoints; ++i){
        tet.vertices[i].index = i;
        for(size_t j = 0; j < out.mesh_dim; ++j)
            tet.points[i][j] = out.pointlist[index++];
    }

    index = 0; 
    tet.tetrahedrons.resize(out.numberoftetrahedra);
    for(size_t i = 0; i < out.numberoftetrahedra; ++i){
        auto & tetrahedron = tet.tetrahedrons[i];
        for(size_t j = 0; j < 4; ++j){
            tetrahedron.vertices[j] = out.tetrahedronlist[index++];
            tet.vertices[tetrahedron.vertices[j]].tetrahedrons.insert(i);
        }
    }

    return true;
}


