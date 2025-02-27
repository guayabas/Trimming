from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Fuse
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeFace
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeSphere
from OCC.Core.gp import gp_Pnt, gp_Dir, gp_Vec, gp_Pln
import OCC.Display.SimpleGui

def convert_list_to_vec3_OCCT(list):
    return gp_Vec(list[0], list[1], list[2])

def make_plane(normal, origin, size):
    plane_normal = gp_Dir(convert_list_to_vec3_OCCT(normal))
    plane_origin = gp_Pnt(convert_list_to_vec3_OCCT(origin).XYZ())
    plane_size = size
    plane = gp_Pln(plane_origin, plane_normal)
    return BRepBuilderAPI_MakeFace(plane, -plane_size, plane_size, -plane_size, plane_size).Face()

origin_shape = BRepPrimAPI_MakeSphere(gp_Pnt(0, 0, 0), 0.1).Shape()

plane1_shape = make_plane([0, 0, 1], [+1,  0.0, 0], 1.0)
plane2_shape = make_plane([0, 0, 1], [-1,  0.0, 0], 1.0)
plane3_shape = make_plane([0, 0, 1], [ 0.0, +1, 0], 1.0)
plane4_shape = make_plane([0, 0, 1], [ 0.0, -1, 0], 1.0)

boolean1 = BRepAlgoAPI_Common(plane1_shape, plane3_shape).Shape()
boolean2 = BRepAlgoAPI_Common(plane1_shape, plane4_shape).Shape()
boolean3 = BRepAlgoAPI_Common(plane2_shape, plane3_shape).Shape()
boolean4 = BRepAlgoAPI_Common(plane2_shape, plane4_shape).Shape()
# combined = BRepAlgoAPI_Fuse(boolean1, boolean2).Shape()

display, start_display, _, _ = OCC.Display.SimpleGui.init_display()
display.DisplayShape(origin_shape)
# display.DisplayShape(plane1_shape)
# display.DisplayShape(plane2_shape)
# display.DisplayShape(plane3_shape)
# display.DisplayShape(plane4_shape)
display.DisplayShape(boolean1)
display.DisplayShape(boolean2)
display.DisplayShape(boolean3)
display.DisplayShape(boolean4)
# display.DisplayShape(combined)
display.FitAll()
start_display()