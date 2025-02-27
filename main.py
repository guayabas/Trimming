# TODO : Use the tolerance given by the primitive "err"
# TODO : Make boolean operation between planes
# TODO : Make a face when doing plane with circular curve
# TODO : Check if remove the color encoding
# TODO : Verify the trim of cylinder does not give non-axis-aligned ellipse
# TODO : Do intersection of quads
# TODO : Add the corners (and think if they are relevant for the algorithms?)

from OCC.Core.gp import gp_Ax2, gp_Pnt, gp_Dir, gp_Vec, gp_Pln
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse, BRepAlgoAPI_Common, BRepAlgoAPI_Cut
from OCC.Core.TopoDS import TopoDS_Shape
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeWire
from OCC.Display.SimpleGui import init_display
import OCC.Core.Quantity
import random
import json
import sys

class DisplayShape:
    def __init__(self, shape : TopoDS_Shape, color = None):
        self.shape = shape
        self.color = color

COLOR_PALETTE = [
    (1.0, 0.0, 0.0),       # Red
    (0.0, 1.0, 0.0),       # Green
    (0.0, 0.0, 1.0),       # Blue
    (1.0, 1.0, 0.0),       # Yellow
    (1.0, 0.65, 0.0),      # Orange
    (0.5, 0.0, 0.5),       # Purple
    (0.0, 1.0, 1.0),       # Cyan
    (1.0, 0.0, 1.0),       # Magenta
    (1.0, 0.75, 0.8),      # Pink
    (0.2, 0.8, 0.2),       # Lime
    (0.65, 0.16, 0.16),    # Brown
    (1.0, 0.84, 0.0),      # Gold
    (0.75, 0.75, 0.75),    # Silver
    (0.0, 0.0, 0.5),       # Navy
    (0.0, 0.5, 0.5),       # Teal
    (0.5, 0.5, 0.0),       # Olive
    (0.66, 0.66, 0.66),    # Dark Gray
    (0.83, 0.83, 0.83),    # Light Gray
    (0.5, 0.0, 0.0),       # Maroon
    (0.53, 0.81, 0.92),    # Sky Blue
    (1.0, 0.08, 0.58),     # Deep Pink
    (0.25, 0.88, 0.82)     # Turquoise
]

def get_color_OCCT(index):
    color = COLOR_PALETTE[index]
    return OCC.Core.Quantity.Quantity_Color(color[0], color[1], color[2], OCC.Core.Quantity.Quantity_TOC_RGB)

def make_dir_OCCT_to_string(d : gp_Dir):
    return (str(d.X()) + ", " + str(d.Y()) + ", " + str(d.Z()))

def make_pnt_OCCT_to_string(p : gp_Pnt):
    return (str(p.X()) + ", " + str(p.Y()) + ", " + str(p.Z()))

def make_vec_OCCT_to_string(v : gp_Vec):
    return (str(v.X()) + ", " + str(v.Y()) + ", " + str(v.Z()))

def convert_list_to_vec3_OCCT(list):
    if len(list) != 3:
        print("input list needs to have dimension 3")
        exit(-1)
    return gp_Vec(list[0], list[1], list[2])

def convert_vec3_OCCT_to_list(v : gp_Vec):
    return [v.X(), v.Y(), v.Z()]

def convert_vec3_OCCT_to_pnt_OCCT(v : gp_Vec):
    return gp_Pnt(v.X(), v.Y(), v.Z())

def normalize_normal_with_OCCT(normal):
    normal_vector = gp_Vec(normal[0], normal[1], normal[2])
    if normal_vector.Magnitude() > 0:
        normal_vector.Normalize()
    return normal_vector

def multiply_vec3_OCCT_with_scalar(v : gp_Vec, scalar : float):
    return gp_Vec(v.X() * scalar, v.Y() * scalar, v.Z() * scalar)

def make_cylinder_OCCT(radius = 1.0, height = 1.0, base = [0.0, 0.0, 0.0], direction = [0.0, 0.0, 1.0]):
    base_of_cylinder = gp_Pnt(base[0], base[1], base[2])
    direction_of_cylinder = gp_Dir(direction[0], direction[1], direction[2])
    axis_of_cylinder = gp_Ax2(base_of_cylinder, direction_of_cylinder)
    print("\tCreating cylinder BRep with")
    print("\t\tradius      :", radius)
    print("\t\theight      :", height)
    print("\t\tbase        :", make_pnt_OCCT_to_string(base_of_cylinder))
    print("\t\tdirection   :", make_pnt_OCCT_to_string(direction_of_cylinder))
    return BRepPrimAPI_MakeCylinder(axis_of_cylinder, radius, height).Shape()

def make_plane_with_distance_OCCT(normal = [0.0, 0.0, 1.0], distance_to_origin = 0.0, length = 1.0):
    plane_normal = gp_Dir(normalize_normal_with_OCCT(normal))
    origin = [distance_to_origin * plane_normal.X(), distance_to_origin * plane_normal.Y(), distance_to_origin * plane_normal.Z()]
    plane_origin = gp_Pnt(origin[0], origin[1], origin[2])
    plane = gp_Pln(plane_origin, plane_normal)
    print("\tCreating plane BRep with (distance)")
    print("\t\tnormal             : ", make_dir_OCCT_to_string(plane_normal))
    print("\t\tdistance to origin : ", distance_to_origin)
    print("\t\tside length        : ", length)
    return BRepBuilderAPI_MakeFace(plane, -length, length, -length, length).Face()

def make_plane_with_origin_OCCT(normal = [0.0, 0.0, 1.0], origin = [0.0, 0.0, 0.0], length = 1.0):
    plane_normal = gp_Dir(normalize_normal_with_OCCT(normal))
    plane_origin = gp_Pnt(origin[0], origin[1], origin[2])
    plane = gp_Pln(plane_origin, plane_normal)
    print("\tCreating plane BRep with (origin)")
    print("\t\tnormal      : ", make_dir_OCCT_to_string(plane_normal))
    print("\t\torigin      : ", make_pnt_OCCT_to_string(plane_origin))
    print("\t\tside length : ", length)
    return BRepBuilderAPI_MakeFace(plane, -length, length, -length, length).Face()

def make_plane_with_normal_and_tangent_OCCT(normal = [0.0, 0.0, 1.0], origin = [0.0, 0.0, 0.0], tangent = [1.0, 0.0, 0.0], length = 1.0):
    e1 = convert_list_to_vec3_OCCT(tangent).Normalized()
    e2 = e1.Crossed(convert_list_to_vec3_OCCT(normal)).Normalized()
    d1 = multiply_vec3_OCCT_with_scalar(e1, length)
    d2 = multiply_vec3_OCCT_with_scalar(e2, length)
    o = convert_list_to_vec3_OCCT(origin)
    p1 = convert_vec3_OCCT_to_pnt_OCCT(o - d1 - d2)
    p2 = convert_vec3_OCCT_to_pnt_OCCT(o - d1 + d2)
    p3 = convert_vec3_OCCT_to_pnt_OCCT(o + d1 + d2)
    p4 = convert_vec3_OCCT_to_pnt_OCCT(o + d1 - d2)
    edge1 = BRepBuilderAPI_MakeEdge(p1, p2).Edge()
    edge2 = BRepBuilderAPI_MakeEdge(p2, p3).Edge()
    edge3 = BRepBuilderAPI_MakeEdge(p3, p4).Edge()
    edge4 = BRepBuilderAPI_MakeEdge(p4, p1).Edge()
    wire = BRepBuilderAPI_MakeWire(edge1, edge2, edge3, edge4).Wire()
    print("\tCreating plane BRep with (normal and tangent)")
    print("\t\tp1 : ", make_pnt_OCCT_to_string(p1))
    print("\t\tp2 : ", make_pnt_OCCT_to_string(p2))
    print("\t\tp3 : ", make_pnt_OCCT_to_string(p3))
    print("\t\tp4 : ", make_pnt_OCCT_to_string(p4))
    return BRepBuilderAPI_MakeFace(wire).Face()

def make_curve_OCCT(points, lines):
    curve = [BRepBuilderAPI_MakeEdge(gp_Pnt(*points[line[0]]), gp_Pnt(*points[line[1]])).Edge() for line in lines]
    # for line in lines:
    #     p1 = points[line[0]]
    #     p2 = points[line[1]]
    #     edge = BRepBuilderAPI_MakeEdge(gp_Pnt(p1[0], p1[1], p1[2]), gp_Pnt(p2[0], p2[1], p2[2])).Edge()
    #     curve.append(DisplayShape(edge, color))
    return curve

def make_domain_OCCT():
    p1 = gp_Pnt(-1.0, -1.0, -1.0)
    p2 = gp_Pnt(+1.0, -1.0, -1.0)
    p3 = gp_Pnt(+1.0, +1.0, -1.0)
    p4 = gp_Pnt(-1.0, +1.0, -1.0)
    p5 = gp_Pnt(-1.0, -1.0, +1.0)
    p6 = gp_Pnt(+1.0, -1.0, +1.0)
    p7 = gp_Pnt(+1.0, +1.0, +1.0)
    p8 = gp_Pnt(-1.0, +1.0, +1.0)
    edges = []
    edges.append(BRepBuilderAPI_MakeEdge(p1, p2).Edge())
    edges.append(BRepBuilderAPI_MakeEdge(p2, p3).Edge())
    edges.append(BRepBuilderAPI_MakeEdge(p3, p4).Edge())  
    edges.append(BRepBuilderAPI_MakeEdge(p4, p1).Edge())
    edges.append(BRepBuilderAPI_MakeEdge(p5, p6).Edge())
    edges.append(BRepBuilderAPI_MakeEdge(p6, p7).Edge())
    edges.append(BRepBuilderAPI_MakeEdge(p7, p8).Edge())
    edges.append(BRepBuilderAPI_MakeEdge(p8, p5).Edge())
    edges.append(BRepBuilderAPI_MakeEdge(p1, p5).Edge())
    edges.append(BRepBuilderAPI_MakeEdge(p4, p8).Edge())
    edges.append(BRepBuilderAPI_MakeEdge(p2, p6).Edge())
    edges.append(BRepBuilderAPI_MakeEdge(p3, p7).Edge())
    return edges

class Primitive:
    def __init__(self):
        pass

class Plane(Primitive):
    def __init__(self):
        super().__init__()
        self.normal = [0.0, 0.0, 1.0]
        self.distance_to_origin = 0.0

class Cylinder(Primitive):
    def __init__(self):
        super().__init__()
        self.radius = 1.0
        self.height = 1.0
        self.base = [0.0, 0.0, 0.0]
        self.direction = [0.0, 0.0, 1.0]

class Curve:
    def __init__(self):
        self.id = -1
        self.points = None
        self.lines = None

def signed_distance_point_to_plane_OCCT(point_to_find_distance : gp_Vec, normal : gp_Vec, point_in_plane : gp_Vec):
    return (point_to_find_distance - point_in_plane).Dot(normal)

def find_start_end_of_curve(curve : Curve):
    count = {}
    # print(curve.lines)
    for line in curve.lines:
        count[line[0]] = count.get(line[0], 0) + 1
        count[line[1]] = count.get(line[1], 0) + 1
    start_end_points = []
    for index_count in count:
        if count[index_count] == 1:
            start_end_points.append(index_count)
    return start_end_points

def trim_cylinder(curve : Curve, cylinder : Cylinder):
    P0 = convert_list_to_vec3_OCCT(cylinder.base)
    d = normalize_normal_with_OCCT(cylinder.direction)
    P = convert_list_to_vec3_OCCT(curve.points[0])
    candidate_radius = (P - P0).Crossed(d).Magnitude()
    print("\t", candidate_radius - cylinder.radius)
    # if (abs(candidate_radius - cylinder.radius) < sys.float_info.epsilon):
    if (abs(candidate_radius - cylinder.radius) < 1e-4):
        signed_distance = (P - P0).Dot(d)
        if signed_distance < 0:
            d = -d
        cylinder.height = abs(signed_distance)
        trimmed_cylinder = make_cylinder_OCCT(cylinder.radius, cylinder.height, cylinder.base, convert_vec3_OCCT_to_list(d))
        # trimmed_cylinder = make_cylinder_OCCT(cylinder.radius, cylinder.height, convert_vec3_OCCT_to_list(P) , convert_vec3_OCCT_to_list(d))
        return trimmed_cylinder
    return None

def trim_plane(curve : Curve, plane : Plane):
    Q1 = convert_list_to_vec3_OCCT(curve.points[ 0])
    Q2 = convert_list_to_vec3_OCCT(curve.points[-1])
    N = normalize_normal_with_OCCT(plane.normal)
    P = N * plane.distance_to_origin
    signed_distance_to_plane_q1 = signed_distance_point_to_plane_OCCT(Q1, N, P)
    signed_distance_to_plane_q2 = signed_distance_point_to_plane_OCCT(Q2, N, P)
    print("\t", signed_distance_to_plane_q1, " | ", signed_distance_to_plane_q2)
    if (abs(signed_distance_to_plane_q1) < 1e-4 and abs(signed_distance_to_plane_q2) < 1e-4):

        start_end_indices = find_start_end_of_curve(curve)
        print("\t", start_end_indices)

        # curve_length = 0.0
        # for line in curve.lines:
        #     p1 = convert_list_to_vec3_OCCT(curve.points[line[0]])
        #     p2 = convert_list_to_vec3_OCCT(curve.points[line[1]])
        #     curve_length += (p2 - p1).Magnitude()
        # curve_center_of_mass = gp_Vec(0.0, 0.0, 0.0)
        # for point in curve.points:
        #     # print("\t", point)
        #     curve_center_of_mass += convert_list_to_vec3_OCCT(point)
        # curve_center_of_mass /= len(curve.points)
        # # print(curve.points[0])
        # # print(curve.points[-1])
        # print("\t", curve_length)
        # print("\t", len(curve.points))
        # print("\t", make_vec_OCCT_to_string(curve_center_of_mass))
        # A = Q1
        # B = curve_center_of_mass
        # C = Q2
        # print("\t", ((B - A).Crossed(B - C)).Magnitude())
        # are_vectors_collinear = ((B - A).Crossed(B - C)).Magnitude() < 1e-5
        # if are_vectors_collinear:
        #     return make_plane_with_normal_and_tangent_OCCT(plane.normal, convert_vec3_OCCT_to_list(curve_center_of_mass), convert_vec3_OCCT_to_list(Q2 - Q1), curve_length * 0.5)

        if len(start_end_indices) == 2:
            P1 = convert_list_to_vec3_OCCT(curve.points[start_end_indices[0]])
            P2 = convert_list_to_vec3_OCCT(curve.points[start_end_indices[1]])
            curve_length = (P1 - P2).Magnitude()
            curve_center_of_mass = multiply_vec3_OCCT_with_scalar((P1 + P2), 0.5)
            print("\t", curve_length)
            print("\t", make_vec_OCCT_to_string(curve_center_of_mass))
            return make_plane_with_normal_and_tangent_OCCT(plane.normal, convert_vec3_OCCT_to_list(curve_center_of_mass), convert_vec3_OCCT_to_list(Q2 - Q1), curve_length * 0.5)

    return None

def make_shape(primitive):
    params = primitive['params']
    if primitive['type'] == "plane":
        return make_plane_with_distance_OCCT(params[0][0], params[1], 1.0)
    if primitive['type'] == "cylinder":
        positive_cylinder = make_cylinder_OCCT(params[2], 1.0, params[1], params[0])
        negative_cylinder = make_cylinder_OCCT(params[2], 1.0, params[1], [-params[0][0], -params[0][1], -params[0][2]])
        return BRepAlgoAPI_Fuse(positive_cylinder, negative_cylinder).Shape()
        # return make_cylinder_OCCT(params[2], 1.0, params[1], [-params[0][0], -params[0][1], -params[0][2]])
        # return make_cylinder_OCCT(params[2], 1.0, params[1], params[0])
    return None

def trim(curve : Curve, primitive):
    print("Checking curve : ", curve.id)
    params = primitive['params']
    if primitive['type'] == "plane":
        plane_normal = params[0][0]
        plane_distance_to_origin = params[1]
        plane = Plane()
        plane.normal = plane_normal
        plane.distance_to_origin = plane_distance_to_origin
        return trim_plane(curve, plane)
    if primitive['type'] == "cylinder":
        cylinder_direction = params[0]
        cylinder_base = params[1]
        cylinder_radius = params[2]
        cylinder = Cylinder()
        cylinder.direction = cylinder_direction
        cylinder.base = cylinder_base
        cylinder.radius = cylinder_radius
        return trim_cylinder(curve, cylinder)
    return None
        
def display_scene_OCCT(display_shapes):
    display, start_display, _, _ = init_display()
    for display_shape in display_shapes:
        if display_shape.shape is not None:
            if display_shape.color is not None:
                display.DisplayShape(display_shape.shape, update=False, color=display_shape.color)
            else:
                display.DisplayShape(display_shape.shape, update=False)
    display.FitAll()
    start_display()

if __name__  == '__main__':
    with open('topo.json') as file:
        edges = json.load(file)
    print("number of curves : ", len(edges['curves']))

    with open("surface_info.json") as file:
        primitives = json.load(file)
    print("number of primitives : ", len(primitives))

    shapes = []

    domain = make_domain_OCCT()
    for edge in domain:
        shapes.append(DisplayShape(edge, OCC.Core.Quantity.Quantity_NOC_RED))

    # primitive_index = 2
    # for primitive_index in range(len(primitives)):
    # for primitive_index in [0, 4, 6, 9, 10]:
    # for primitive_index in [1, 2, 3, 5, 7, 8, 11, 12, 13, 14, 15]:
    for primitive_index in [9]:
    # print(primitive_index)
    # print(primitives[primitive_index])
        # curve_index = 2
        trimmed_shapes = []
        for curve_index in range(len(edges['curves'])):
        # for curve_index in [6, 18]:
            curve_points = edges['curves'][curve_index]['pv_points']
            curve_lines = edges['curves'][curve_index]['pv_lines']
            curve = make_curve_OCCT(curve_points, curve_lines)
            curve_color = get_color_OCCT(curve_index)
            for line in curve:
                shapes.append(DisplayShape(line, curve_color))
            curve = Curve()
            curve.points = curve_points
            curve.lines = curve_lines
            curve.id = curve_index
            trimmed_primitive = trim(curve, primitives[primitive_index])
            if trimmed_primitive is not None:
                trimmed_shapes.append(trimmed_primitive)

        # print(len(trimmed_shapes))
        # n = 0
        for trimmed_shape in trimmed_shapes:
            # shapes.append(DisplayShape(trimmed_shape, get_color_OCCT(n)))
            shapes.append(DisplayShape(trimmed_shape))
            # n = n + 1

        # if len(trimmed_shapes) == 4:
        #     # boolean_shape_1 = BRepAlgoAPI_Common(trimmed_shapes[0], trimmed_shapes[2]).Shape()
        #     # boolean_shape_2 = BRepAlgoAPI_Common(trimmed_shapes[0], trimmed_shapes[3]).Shape()
        #     # boolean_shape_3 = BRepAlgoAPI_Common(trimmed_shapes[1], trimmed_shapes[2]).Shape()
        #     # boolean_shape_4 = BRepAlgoAPI_Common(trimmed_shapes[1], trimmed_shapes[3]).Shape()

        #     boolean_shape_1 = BRepAlgoAPI_Common(trimmed_shapes[0], trimmed_shapes[1]).Shape()
        #     boolean_shape_2 = BRepAlgoAPI_Common(trimmed_shapes[0], trimmed_shapes[3]).Shape()
        #     boolean_shape_3 = BRepAlgoAPI_Common(trimmed_shapes[2], trimmed_shapes[1]).Shape()
        #     boolean_shape_4 = BRepAlgoAPI_Common(trimmed_shapes[2], trimmed_shapes[3]).Shape()

        #     # boolean_shape_1 = BRepAlgoAPI_Common(trimmed_shapes[0], trimmed_shapes[2]).Shape()
        #     # boolean_shape_2 = BRepAlgoAPI_Common(trimmed_shapes[0], trimmed_shapes[1]).Shape()
        #     # boolean_shape_3 = BRepAlgoAPI_Common(trimmed_shapes[3], trimmed_shapes[2]).Shape()
        #     # boolean_shape_4 = BRepAlgoAPI_Common(trimmed_shapes[3], trimmed_shapes[1]).Shape()

        #     shapes.append(DisplayShape(boolean_shape_1))
        #     shapes.append(DisplayShape(boolean_shape_2))
        #     shapes.append(DisplayShape(boolean_shape_3))
        #     shapes.append(DisplayShape(boolean_shape_4))

        # shapes.append(DisplayShape(make_shape(primitives[primitive_index])))

    shapes.append(DisplayShape(make_shape(primitives[0])))
    shapes.append(DisplayShape(make_shape(primitives[4])))
    shapes.append(DisplayShape(make_shape(primitives[6])))
    shapes.append(DisplayShape(make_shape(primitives[10])))

    shapes.append(DisplayShape(make_shape(primitives[2])))
    shapes.append(DisplayShape(make_shape(primitives[9])))
    shapes.append(DisplayShape(make_shape(primitives[8])))

    display_scene_OCCT(shapes)
