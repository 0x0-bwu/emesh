# emesh
ecad 2d/3d mesher (unfinished)

## mesh options:
  -b, --beta                 test flow
  -h, --help                 produce help message
  -s, --surface              planer surface mesh
  -i, --input arg (=dmcdom)  input file format
  -o, --output arg (=vtk)    output file format
  -j, --jobs arg (=1)        cpu core numbers in use
  -p, --partition arg (=0)   partition level(3d)
  -g, --gradelevel arg (=0)  grade level
  -z, --ratioz arg (=1)      vertical slice ratio(3d)
  -t, --tolerance arg (=0)   tolerance to simplify input geometries
  -r, --refine arg (=0)      mesh refine iteration(2d)
  -a, --alpha arg (=15)      minimum angle(unit: deg) of refinement(2d)
  -l, --maxlen arg (=0)      max edge length
