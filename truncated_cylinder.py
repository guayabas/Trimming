from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeFace
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCC.Core.gp import gp_Pnt, gp_Dir, gp_Vec, gp_Ax2, gp_Pln
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeWire
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_EDGE
import OCC.Display.SimpleGui

def convert_list_to_vec3_OCCT(list):
    return gp_Vec(list[0], list[1], list[2])

def make_plane(normal, origin, size):
    plane_normal = gp_Dir(convert_list_to_vec3_OCCT(normal))
    plane_origin = gp_Pnt(convert_list_to_vec3_OCCT(origin).XYZ())
    plane_size = size
    plane = gp_Pln(plane_origin, plane_normal)
    return BRepBuilderAPI_MakeFace(plane, -plane_size, plane_size, -plane_size, plane_size).Face()

def extract_wire_from_common_shape(common_shape):
    wire_maker = BRepBuilderAPI_MakeWire()
    explorer = TopExp_Explorer(common_shape, TopAbs_EDGE)
    while explorer.More():
        wire_maker.Add(explorer.Current())
        explorer.Next()
    return wire_maker.Wire()

cylinder_base = gp_Pnt(convert_list_to_vec3_OCCT([-2, 0, 0]).XYZ())
cylinder_height = 4.0
cylinder_direction = gp_Dir(convert_list_to_vec3_OCCT([1, 0, 0]))
cylinder_radius = 1.0
cylinder_shape = BRepPrimAPI_MakeCylinder(gp_Ax2(cylinder_base, cylinder_direction), cylinder_radius, cylinder_height).Shape()

plane1_shape = make_plane([1, 0, -1], [ 0, 0, 0], 2.0)
plane2_shape = make_plane([1, 0, 0], [-2, 0, 0], 2.0)

boolean1_shape = BRepAlgoAPI_Common(cylinder_shape, plane1_shape).Shape()
boolean2_shape = BRepAlgoAPI_Common(cylinder_shape, plane2_shape).Shape()

wire1 = extract_wire_from_common_shape(boolean1_shape)
wire2 = extract_wire_from_common_shape(boolean2_shape)
loft = BRepOffsetAPI_ThruSections(True, True)  # Solid, Ruled surface enabled
loft.AddWire(wire1)
loft.AddWire(wire2)
loft.Build()
lofted_solid = loft.Shape()

display, start_display, _, _ = OCC.Display.SimpleGui.init_display()
# display.DisplayShape(cylinder_shape)
# display.DisplayShape(plane1_shape)
# display.DisplayShape(plane2_shape)
# display.DisplayShape(boolean1_shape)
# display.DisplayShape(boolean2_shape)
display.DisplayShape(lofted_solid)
display.FitAll()
start_display()