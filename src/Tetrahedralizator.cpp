#include "Tetrahedralizator.h"
#include "tetgen/tetgen.h"
using namespace emesh;
bool Tetrahedralizator::Tetrahedralize(const std::vector<Point> & points, const std::list<Edge> & edges)
{
    tet.Clear();

    size_t index;
    tetgenbehavior b;
    tetgenio in, add, out, bgmin;

    b.quiet = 1;
    b.plc = 1;//-p
    b.convex = 1;//-c
    // b.vtkview = 1;//-k

    in.mesh_dim = 3;
    in.numberofpointattributes = 0;
    in.numberofpoints = points.size();
    in.pointlist = new double[in.numberofpoints * in.mesh_dim];
    
    index = 0;
    for(int i = 0; i < in.numberofpoints; ++i)
        for(int j = 0; j < in.mesh_dim; ++j)
            in.pointlist[index++] = points[i][j];
    
    in.firstnumber = 0;
    in.numberofedges = edges.size();
    in.edgelist = new int[in.numberofedges * 2];
    index = 0;
    for(const auto & edge : edges){
        in.edgelist[index++] = edge.v1();
        in.edgelist[index++] = edge.v2();
    }

    // b.quality = 1;
    // b.minratio = 20;
    // b.mindihedral = 10;
    // b.verbose = std::numeric_limits<int>::max();
    tetgenmesh m;

    try {
        tetrahedralize(&b, &in, &out, &add, &bgmin, &m);
    }
    catch (...){
        tet.Clear();
        return false;
    }

    assert(out.firstnumber == 0);

    index = 0;
    tet.points.resize(out.numberofpoints);
    tet.vertices.resize(out.numberofpoints);
    for (int i = 0; i < out.numberofpoints; ++i){
        tet.vertices[i].index = i;
        for (int j = 0; j < out.mesh_dim; ++j)
            tet.points[i][j] = out.pointlist[index++];
    }

    index = 0; 
    tet.tetrahedrons.resize(out.numberoftetrahedra);
    for (int i = 0; i < out.numberoftetrahedra; ++i){
        auto & tetrahedron = tet.tetrahedrons[i];
        for (int j = 0; j < 4; ++j){
            tetrahedron.vertices[j] = out.tetrahedronlist[index++];
            tet.vertices[tetrahedron.vertices[j]].tetrahedrons.insert(i);
        }
    }
    return true;
}

bool Tetrahedralizator::Tetrahedralize(const std::vector<Point> & points, const std::list<Face> & faces, const std::list<Edge> & edges)
{
    tet.Clear();

    size_t index;
    tetgenbehavior b;
    tetgenio in, add, out, bgmin;

    b.quiet = 1;
    b.plc = 1;//-p
    b.convex = 1;//-c
    // b.vtkview = 1;//-k

    in.mesh_dim = 3;
    in.numberofpointattributes = 0;
    in.numberofpoints = points.size();
    in.pointlist = new double[in.numberofpoints * in.mesh_dim];
    
    index = 0;
    for (int i = 0; i < in.numberofpoints; ++i)
        for (int j = 0; j < in.mesh_dim; ++j)
            in.pointlist[index++] = points[i][j];
    
    in.firstnumber = 0;
    in.numberofedges = edges.size();
    in.edgelist = new int[in.numberofedges * 2];
    index = 0;
    for(const auto & edge : edges){
        in.edgelist[index++] = edge.v1();
        in.edgelist[index++] = edge.v2();
    }


    in.numberoffacets = faces.size();
    in.facetlist = new typename tetgenio::facet[in.numberoffacets];
    index = 0;
    for(const auto & face : faces){
        auto f = &(in.facetlist[index++]);
        f->numberofpolygons = 1;
        f->polygonlist = new typename tetgenio::polygon[f->numberofpolygons];
        f->numberofholes = 0;
        f->holelist = (double*)(nullptr);
        auto p = &(f->polygonlist[0]);
        p->numberofvertices = face.size();
        p->vertexlist = new int[p->numberofvertices];
        for(int j = 0; j < p->numberofvertices; ++j){
            p->vertexlist[j] = face[j];
        }
    }
    
    // b.quality = 1;
    // b.minratio = 20;
    // b.mindihedral = 10;
    // b.verbose = std::numeric_limits<int>::max();
    tetgenmesh m;

    try {
        tetrahedralize(&b, &in, &out, &add, &bgmin, &m);
    }
    catch (...){
        tet.Clear();
        return false;
    }

    assert(out.firstnumber == 0);

    index = 0;
    tet.points.resize(out.numberofpoints);
    tet.vertices.resize(out.numberofpoints);
    for (int i = 0; i < out.numberofpoints; ++i){
        tet.vertices[i].index = i;
        for (int j = 0; j < out.mesh_dim; ++j)
            tet.points[i][j] = out.pointlist[index++];
    }

    index = 0; 
    tet.tetrahedrons.resize(out.numberoftetrahedra);
    for (int i = 0; i < out.numberoftetrahedra; ++i){
        auto & tetrahedron = tet.tetrahedrons[i];
        for (int j = 0; j < 4; ++j){
            tetrahedron.vertices[j] = out.tetrahedronlist[index++];
            tet.vertices[tetrahedron.vertices[j]].tetrahedrons.insert(i);
        }
    }
    return true; 
}
