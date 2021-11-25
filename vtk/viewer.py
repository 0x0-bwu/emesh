import collections
import os

# noinspection PyUnresolvedReferences
import vtkmodules.vtkInteractionStyle
# noinspection PyUnresolvedReferences
import vtkmodules.vtkRenderingOpenGL2
from vtkmodules.vtkCommonColor import vtkNamedColors
from vtkmodules.vtkCommonDataModel import (
    vtkCellTypes,
    vtkPlane
)
from vtkmodules.vtkCommonTransforms import vtkTransform
from vtkmodules.vtkFiltersGeneral import vtkTableBasedClipDataSet
from vtkmodules.vtkIOLegacy import vtkUnstructuredGridReader
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkDataSetMapper,
    vtkRenderWindow,
    vtkRenderWindowInteractor,
    vtkRenderer
)


def get_filename():
    cwd = os.getcwd()
    return cwd + '/test/subgds/SubGDS_DIE1.vtk'
    # return cwd + '/test/fccsp/layer.vtk'


def main():
    filename = get_filename()

    # Create the reader for the data.
    reader = vtkUnstructuredGridReader()
    reader.SetFileName(filename)
    reader.Update()

    bounds = reader.GetOutput().GetBounds()
    center = reader.GetOutput().GetCenter()

    colors = vtkNamedColors()
    renderer = vtkRenderer()
    renderer.SetBackground(colors.GetColor3d('Wheat'))
    renderer.UseHiddenLineRemovalOn()

    renderWindow = vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindow.SetSize(640, 480)

    interactor = vtkRenderWindowInteractor()
    interactor.SetRenderWindow(renderWindow)

    xnorm = [1.0, 1.0, 0.0]

    clipPlane = vtkPlane()
    clipPlane.SetOrigin(reader.GetOutput().GetCenter())
    clipPlane.SetNormal(xnorm)

    clipper = vtkTableBasedClipDataSet()
    clipper.SetClipFunction(clipPlane)
    clipper.SetInputData(reader.GetOutput())
    clipper.SetValue(0.0)
    clipper.GenerateClippedOutputOn()
    clipper.Update()
    
    clippedMapper = vtkDataSetMapper()
    # clippedMapper.SetInputData(reader.GetOutput())
    clippedMapper.SetInputData(clipper.GetClippedOutput())
    clippedMapper.ScalarVisibilityOff()

    clippedActor = vtkActor()
    clippedActor.SetMapper(clippedMapper)
    clippedActor.GetProperty().SetDiffuseColor(colors.GetColor3d('tomato'))
    clippedActor.GetProperty().EdgeVisibilityOn()

    clippedTransform = vtkTransform()
    clippedTransform.Translate(-center[0], -center[1], -center[2])
    clippedActor.SetUserTransform(clippedTransform)

    renderer.AddViewProp(clippedActor)

    renderer.ResetCamera()
    renderer.GetActiveCamera().Dolly(1.4)
    renderer.ResetCameraClippingRange()
    renderWindow.Render()
    renderWindow.SetWindowName('mesh viewer')
    renderWindow.Render()

    interactor.Start()

if __name__ == '__main__':
    main()