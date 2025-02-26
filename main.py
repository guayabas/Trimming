from OCC.Core.gp import gp_Ax2, gp_Pnt, gp_Dir, gp_Pln
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace

def make_cylinder_OCCT(radius = 1.0, height = 1.0, base = [0.0, 0.0, 0.0], direction = [0.0, 0.0, 1.0]):
    base_of_cylinder = gp_Pnt(base[0], base[1], base[2])
    axis_of_cylinder = gp_Ax2(base_of_cylinder, gp_Dir(direction[0], direction[1], direction[2]))
    return BRepPrimAPI_MakeCylinder(axis_of_cylinder, radius, height).Shape()

def make_domain_OCCT():
    p1 = gp_Pnt(-1.0, -1.0, -1.0)
    p2 = gp_Pnt(+1.0, -1.0, -1.0)
    p3 = gp_Pnt(+1.0, +1.0, -1.0)
    p4 = gp_Pnt(-1.0, +1.0, -1.0)
    p5 = gp_Pnt(-1.0, -1.0, +1.0)
    p6 = gp_Pnt(+1.0, -1.0, +1.0)
    p7 = gp_Pnt(+1.0, +1.0, +1.0)
    p8 = gp_Pnt(-1.0, +1.0, +1.0)
    edges = [gp_Pnt()] * 12
    edges[ 0] =  BRepBuilderAPI_MakeEdge(p1, p2).Edge()
    edges[ 1] =  BRepBuilderAPI_MakeEdge(p2, p3).Edge()
    edges[ 2] =  BRepBuilderAPI_MakeEdge(p3, p4).Edge()    
    edges[ 3] =  BRepBuilderAPI_MakeEdge(p4, p1).Edge()
    edges[ 4] =  BRepBuilderAPI_MakeEdge(p5, p6).Edge()
    edges[ 5] =  BRepBuilderAPI_MakeEdge(p6, p7).Edge()
    edges[ 6] =  BRepBuilderAPI_MakeEdge(p7, p8).Edge()
    edges[ 7] =  BRepBuilderAPI_MakeEdge(p8, p5).Edge()
    edges[ 8] =  BRepBuilderAPI_MakeEdge(p1, p5).Edge()
    edges[ 9] =  BRepBuilderAPI_MakeEdge(p4, p8).Edge()
    edges[10] =  BRepBuilderAPI_MakeEdge(p2, p6).Edge()
    edges[11] =  BRepBuilderAPI_MakeEdge(p3, p7).Edge()
    return edges

from OCC.Display.SimpleGui import init_display
display, start_display, add_menu, add_function = init_display()

domain = make_domain_OCCT()
for edge in domain:
    display.DisplayShape(edge, update=True)

cylinder1 = make_cylinder_OCCT(1.0, 1.0, [0.0, 0.0,  0.0], [0, 0, 1])
cylinder2 = make_cylinder_OCCT(1.0, 1.0, [0.0, 0.0, -1.0])
display.DisplayShape(cylinder1, update=True)
display.DisplayShape(cylinder2, update=True)

plane_origin = gp_Pnt(0.0, 0.0, 0.0)
plane_normal = gp_Dir(0.0, 0.0, 1.0)
plane = gp_Pln(plane_origin, plane_normal)
face = BRepBuilderAPI_MakeFace(plane, -2.0, 2.0, -2.0, 2.0).Face()
display.DisplayShape(face, update=True)

start_display()