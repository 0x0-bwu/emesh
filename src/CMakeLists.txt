
add_library(Emesh SHARED
Mesh3D.cpp
Mesher2D.cpp
Mesher3D.cpp
MeshFileUtility.cpp
MeshFlow3D.cpp
MeshFlowMT.cpp
MeshIO.cpp
Tetrahedralizator.cpp
./tetgen/predicates.cxx
./tetgen/tetgen.cxx
)