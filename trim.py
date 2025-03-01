# Alejandro Guayaquil 02.2025

from OCC.Core.gp import gp_Ax2, gp_Pnt, gp_Dir, gp_Vec
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Fuse
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeWire
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.StlAPI import StlAPI_Writer
from OCC.Core.TopoDS import TopoDS_Compound
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_EDGE
from OCC.Core.BRep import BRep_Builder
from OCC.Core.GC import GC_MakeCircle
import json
import sys
import os

ARITHMETIC_TOLERANCE = 0.001
LOG_TO_CONSOLE = False

class Primitive:
    def __init__(self):
        self.id = -1
        self.tolerance = 0.0

class Plane(Primitive):
    def __init__(self):
        super().__init__()
        self.normal = [0.0, 0.0, 1.0]
        self.distance_to_origin = 0.0

    def __str__(self):
        return f"\tPlane\n\t\tnormal = {self.normal}\n\t\tdistance to origin = {self.distance_to_origin}"

class Cylinder(Primitive):
    def __init__(self):
        super().__init__()
        self.radius = 1.0
        self.base = [0.0, 0.0, 0.0]
        self.direction = [0.0, 0.0, 1.0]

    def __str__(self):
        return f"\tCylinder\n\t\tradius = {self.radius}\n\t\tbase = {self.base}\n\t\tdirection={self.direction}"

class Curve:
    def __init__(self):
        self.primitives = []
        self.start_end = []
        self.points = None
        self.lines = None
        self.index = -1

# ===================================================
# Utilities between OCCT and Python objects
# ===================================================

def make_pnt_OCCT_to_string(p : gp_Pnt):
    return (str(p.X()) + ", " + str(p.Y()) + ", " + str(p.Z()))

def make_dir_OCCT_to_string(d : gp_Dir):
    return (str(d.X()) + ", " + str(d.Y()) + ", " + str(d.Z()))

def make_vec_OCCT_to_string(v : gp_Vec):
    return (str(v.X()) + ", " + str(v.Y()) + ", " + str(v.Z()))

def convert_vec3_OCCT_to_list(v : gp_Vec):
    return [v.X(), v.Y(), v.Z()]

def convert_vec3_OCCT_to_pnt_OCCT(v : gp_Vec):
    return gp_Pnt(v.X(), v.Y(), v.Z())

def check_dimension_of_list(list):
    if len(list) != 3:
        print("input list needs to have dimension 3")
        exit(-1)

def convert_list_to_dir_OCCT(list):
    check_dimension_of_list(list)
    return gp_Dir(list[0], list[1], list[2])

def convert_list_to_pnt_OCCT(list):
    check_dimension_of_list(list)
    return gp_Pnt(list[0], list[1], list[2])

def convert_list_to_vec3_OCCT(list):
    check_dimension_of_list(list)
    return gp_Vec(list[0], list[1], list[2])

# ===================================================
# Geometric operations based on OCCT
# ===================================================

def normalize_normal_with_OCCT(normal):
    normal_vector = gp_Vec(normal[0], normal[1], normal[2])
    if normal_vector.Magnitude() > 0:
        normal_vector.Normalize()
    return normal_vector

def are_vectors_parallel_with_OCCT(u, v, tolerance = ARITHMETIC_TOLERANCE):
    return (u.Crossed(v).Magnitude() <= tolerance)

def multiply_vec3_OCCT_with_scalar(v : gp_Vec, scalar : float):
    return gp_Vec(v.X() * scalar, v.Y() * scalar, v.Z() * scalar)

def signed_distance_point_to_plane_OCCT(point_to_find_distance : gp_Vec, normal : gp_Vec, point_in_plane : gp_Vec):
    return (point_to_find_distance - point_in_plane).Dot(normal)

# ===================================================
# Geometric operations from scratch
# ===================================================

def check_curve_is_in_plane(curve : Curve, plane : Plane, tolerance = ARITHMETIC_TOLERANCE):
    point_on_curve = None
    if len(curve.start_end) != 0:
        start_point_index = curve.start_end[0]
        end_point_index = curve.start_end[1]
        point_on_curve = multiply_vec3_OCCT_with_scalar(convert_list_to_vec3_OCCT(curve.points[start_point_index]) + convert_list_to_vec3_OCCT(curve.points[end_point_index]), 0.5)
    else:
        point_on_curve = convert_list_to_vec3_OCCT(curve.points[0])
    Q = point_on_curve
    N = normalize_normal_with_OCCT(plane.normal)
    P = N * plane.distance_to_origin
    signed_distance_to_plane = signed_distance_point_to_plane_OCCT(Q, N, P)
    return (abs(signed_distance_to_plane) <= tolerance)

def check_curve_is_in_cylinder(curve : Curve, cylinder : Cylinder, tolerance = ARITHMETIC_TOLERANCE):
    P0 = convert_list_to_vec3_OCCT(cylinder.base)
    d = normalize_normal_with_OCCT(cylinder.direction)
    P = convert_list_to_vec3_OCCT(curve.points[0])
    candidate_radius = (P - P0).Crossed(d).Magnitude()
    return (abs(candidate_radius - cylinder.radius) <= tolerance)

def find_start_end_of_curve(curve : Curve):
    count = {}
    for line in curve.lines:
        count[line[0]] = count.get(line[0], 0) + 1
        count[line[1]] = count.get(line[1], 0) + 1
    start_end_points = []
    for index_count in count:
        if count[index_count] == 1:
            start_end_points.append(index_count)
    curve.start_end = start_end_points

def make_circle_from_curve(curve : Curve):
    point_in_circle = convert_list_to_vec3_OCCT(curve.points[0])
    distance_point_on_opposite_side = sys.float_info.min
    point_on_opposite_side = gp_Pnt(0.0, 0.0, 0.0)
    for point in curve.points:
        P = convert_list_to_vec3_OCCT(point)
        distance = (P - point_in_circle).Magnitude()
        if (distance > distance_point_on_opposite_side):
            point_on_opposite_side = P
            distance_point_on_opposite_side = distance
    center_of_mass = multiply_vec3_OCCT_with_scalar((point_in_circle + point_on_opposite_side), 0.5)
    radius = distance_point_on_opposite_side * 0.5
    return radius, center_of_mass

def collect_primitives_as_objects(primitives : list):
    if len(primitives) == 0:
        return None
    result = []
    for primitive in primitives:
        params = primitive['params']
        primitive_as_object = Primitive()
        if (primitive['type'] == "plane"):
            plane = Plane()
            plane.normal = params[0][0]
            plane.distance_to_origin = params[1]
            primitive_as_object = plane            
        elif (primitive['type'] == "cylinder"):
            cylinder = Cylinder()
            cylinder.direction = params[0]
            cylinder.base = params[1]
            cylinder.radius = params[2]
            primitive_as_object = cylinder
        else:
            print("Unknown primitive")
        primitive_as_object.id = primitive['id']
        primitive_as_object.tolerance = primitive['err']
        result.append(primitive_as_object)
    return result

def find_primitives_for_curve(curve : Curve, primitives : list):
    for primitive in primitives:
        primitive_found = False
        if isinstance(primitive, Plane):
            if check_curve_is_in_plane(curve, primitive):
                primitive_found = True
        elif isinstance(primitive, Cylinder):
            if check_curve_is_in_cylinder(curve, primitive):
                primitive_found = True
        if primitive_found:
            curve.primitives.append(primitive.id)

# ===================================================
# BRep generators based on OCCT
# ===================================================

def make_cylinder_OCCT(radius = 1.0, height = 1.0, base = [0.0, 0.0, 0.0], direction = [0.0, 0.0, 1.0]):
    base_of_cylinder = gp_Pnt(base[0], base[1], base[2])
    direction_of_cylinder = gp_Dir(direction[0], direction[1], direction[2])
    axis_of_cylinder = gp_Ax2(base_of_cylinder, direction_of_cylinder)
    if LOG_TO_CONSOLE:
        print("\t\tCreating cylinder BRep with")
        print("\t\t\tradius      :", radius)
        print("\t\t\theight      :", height)
        print("\t\t\tbase        :", make_pnt_OCCT_to_string(base_of_cylinder))
        print("\t\t\tdirection   :", make_pnt_OCCT_to_string(direction_of_cylinder))
    return BRepPrimAPI_MakeCylinder(axis_of_cylinder, radius, height).Shape()

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
    if LOG_TO_CONSOLE:
        print("\t\tCreating plane BRep with (normal and tangent)")
        print("\t\t\tp1 : ", make_pnt_OCCT_to_string(p1))
        print("\t\t\tp2 : ", make_pnt_OCCT_to_string(p2))
        print("\t\t\tp3 : ", make_pnt_OCCT_to_string(p3))
        print("\t\t\tp4 : ", make_pnt_OCCT_to_string(p4))
    return BRepBuilderAPI_MakeFace(wire).Face()

def make_circular_face_OCCT(radius = 1.0, origin = [0.0, 0.0, 0.0], normal = [0.0, 0.0, 1.0]):
    o = convert_list_to_pnt_OCCT(origin)
    d = convert_list_to_dir_OCCT(normal)
    circle = GC_MakeCircle(gp_Ax2(o, d), radius).Value()
    circle_edge = BRepBuilderAPI_MakeEdge(circle).Edge()
    circle_wire = BRepBuilderAPI_MakeWire(circle_edge).Wire()
    if LOG_TO_CONSOLE:
        print("\t\tCreating plane BRep with (radius)")
        print("\t\t\torigin    : ", make_pnt_OCCT_to_string(o))
        print("\t\t\tdirection : ", make_dir_OCCT_to_string(d))
        print("\t\t\tradius    : ", radius)
    return BRepBuilderAPI_MakeFace(circle_wire).Face()

def extract_wire_from_shape_OCCT(shape):
    wire_maker = BRepBuilderAPI_MakeWire()
    explorer = TopExp_Explorer(shape, TopAbs_EDGE)
    while explorer.More():
        wire_maker.Add(explorer.Current())
        explorer.Next()
    return wire_maker.Wire()

def combine_faces_with_loft_OCCT(face1, face2):
    wire1 = extract_wire_from_shape_OCCT(face1)
    wire2 = extract_wire_from_shape_OCCT(face2)
    loft = BRepOffsetAPI_ThruSections(True, True)
    loft.AddWire(wire1)
    loft.AddWire(wire2)
    loft.Build()
    return loft.Shape()

def make_domain_OCCT(size = 1.0):
    p1 = gp_Pnt(-size, -size, -size)
    p2 = gp_Pnt(+size, -size, -size)
    p3 = gp_Pnt(+size, +size, -size)
    p4 = gp_Pnt(-size, +size, -size)
    p5 = gp_Pnt(-size, -size, +size)
    p6 = gp_Pnt(+size, -size, +size)
    p7 = gp_Pnt(+size, +size, +size)
    p8 = gp_Pnt(-size, +size, +size)
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

def make_curve_OCCT(points, lines):
    return [BRepBuilderAPI_MakeEdge(gp_Pnt(*points[line[0]]), gp_Pnt(*points[line[1]])).Edge() for line in lines]

# ===================================================
# Trimming operations
# ===================================================

def trim_cylinder(curve : Curve, cylinder : Cylinder, plane : Plane):
    D = convert_list_to_vec3_OCCT(cylinder.direction)
    N = convert_list_to_vec3_OCCT(plane.normal)
    if are_vectors_parallel_with_OCCT(D, N):
        P0 = convert_list_to_vec3_OCCT(cylinder.base)
        P = convert_list_to_vec3_OCCT(curve.points[0])
        d = normalize_normal_with_OCCT(cylinder.direction)
        signed_distance = (P - P0).Dot(d)
        if signed_distance < 0:
            d = -d
        cylinder_height = abs(signed_distance)
        return make_cylinder_OCCT(cylinder.radius, cylinder_height, cylinder.base, convert_vec3_OCCT_to_list(d))
    else:
        radius, center_of_mass = make_circle_from_curve(curve)
        face1 = make_circular_face_OCCT(radius, convert_vec3_OCCT_to_list(center_of_mass), plane.normal)
        face2 = make_circular_face_OCCT(radius, cylinder.base, cylinder.direction)
        return combine_faces_with_loft_OCCT(face1, face2)

def trim_plane(curve : Curve, plane : Plane):
    is_curve_closed = (len(curve.start_end) == 0)
    if is_curve_closed:
        radius, curve_center_of_mass = make_circle_from_curve(curve)
        return make_circular_face_OCCT(radius, convert_vec3_OCCT_to_list(curve_center_of_mass), plane.normal), curve_center_of_mass
    else:
        P1 = convert_list_to_vec3_OCCT(curve.points[curve.start_end[0]])
        P2 = convert_list_to_vec3_OCCT(curve.points[curve.start_end[1]])
        tangent = (P1 - P2)
        curve_length = tangent.Magnitude()
        curve_center_of_mass = multiply_vec3_OCCT_with_scalar((P1 + P2), 0.5)
        return make_plane_with_normal_and_tangent_OCCT(
                plane.normal, 
                convert_vec3_OCCT_to_list(curve_center_of_mass), 
                convert_vec3_OCCT_to_list(tangent), 
                curve_length * 0.5
            ), curve_center_of_mass

def trim(curve : Curve, primitives : list):
    number_of_primitives = len(curve.primitives)
    if (number_of_primitives == 1):
        primitive = primitives[curve.primitives[0]]
        if isinstance(primitive, Plane):
            trimmed_shape, _ = trim_plane(curve, primitive)
            return trimmed_shape
    elif (number_of_primitives == 2):
        primitive1 = primitives[curve.primitives[0]]
        primitive2 = primitives[curve.primitives[1]]
        if (isinstance(primitive1, Plane) and isinstance(primitive2, Cylinder)):
            return trim_cylinder(curve, primitive2, primitive1)
        elif (isinstance(primitive1, Cylinder) and isinstance(primitive2, Plane)):
            return trim_cylinder(curve, primitive1, primitive2)
        elif (isinstance(primitive1, Plane) and isinstance(primitive2, Plane)):
            face1, face1_center  = trim_plane(curve, primitive1)
            face2, face2_center = trim_plane(curve, primitive2)
            return [[face1, primitive1.normal, face1_center], [face2, primitive2.normal, face2_center]]
    elif (number_of_primitives == 3):
        cylinders_indices = []
        plane_index = -1
        for primitive_index in curve.primitives:
            primitive = primitives[primitive_index]
            if isinstance(primitive, Cylinder):
                cylinders_indices.append(primitive_index)
            elif isinstance(primitive, Plane):
                plane_index = primitive_index
        D1 = convert_list_to_vec3_OCCT(primitives[cylinders_indices[0]].direction)
        D2 = convert_list_to_vec3_OCCT(primitives[cylinders_indices[1]].direction)
        if (are_vectors_parallel_with_OCCT(D1, D2)):
            return trim_cylinder(curve, primitives[cylinders_indices[1]], primitives[plane_index])
        else:
            print("A more complex intersection of 3 primitives is required")
            return None
    else:
        print("Trim operation for", number_of_primitives, "primitives not supported")
    return None

def trim_collect_shapes_that_are_in_the_same_plane(trimmed_shapes):
    shapes_in_same_plane = {}
    for trimmed_shape in trimmed_shapes:
        if isinstance(trimmed_shape, list):
            for face_with_normal in trimmed_shape:
                normal = face_with_normal[1]
                hash_normal_key = hash(tuple(normal))
                if hash_normal_key not in shapes_in_same_plane:
                    shapes_in_same_plane[hash_normal_key] = []
                shapes_in_same_plane[hash_normal_key].append([face_with_normal[0], face_with_normal[2]])
    collected_trimmed_shapes = []
    for value in shapes_in_same_plane.values():
        if (len(value) == 4):
            max_distance_between_faces_index = -1
            max_distance_between_faces_value = sys.float_info.min
            for face_index, face in enumerate(value):
                distance_between_faces = (value[0][1] - face[1]).Magnitude()
                if (distance_between_faces > max_distance_between_faces_value):
                    max_distance_between_faces_value = distance_between_faces
                    max_distance_between_faces_index = face_index
            other_faces_indices = []
            if max_distance_between_faces_index == 1:
                other_faces_indices.extend([2, 3])
            if max_distance_between_faces_index == 2:
                other_faces_indices.extend([1, 3])
            if max_distance_between_faces_index == 3:
                other_faces_indices.extend([1, 2])
            for other_face_index in other_faces_indices:
                trimmed_shape1 = BRepAlgoAPI_Common(value[0][0], value[other_face_index][0]).Shape()
                trimmed_shape2 = BRepAlgoAPI_Common(value[max_distance_between_faces_index][0], value[other_face_index][0]).Shape()
                combined = BRepAlgoAPI_Fuse(trimmed_shape1, trimmed_shape2).Shape()
                collected_trimmed_shapes.append(combined)
    return collected_trimmed_shapes

def trim_curve_with_primitive(curve : Curve, primitives : list):
    find_start_end_of_curve(curve)
    find_primitives_for_curve(curve, primitives)
    if (len(curve.primitives) != 0):
        if LOG_TO_CONSOLE:
            is_curve_closed = (len(curve.start_end) == 0)
            print("Curve", curve.index, "(", is_curve_closed, ") has primitves :", *curve.primitives)
            for primitive_index in curve.primitives:
                print(primitives[primitive_index])
        trimmed_shape = trim(curve, primitives)
        if trimmed_shape is not None:
            return trimmed_shape
    else:
        print("Error finding primitives for curve ", curve.index)
    return None

def trim_cube(edges, primitives):
    # Face 1 [2, 3, 4, 5] - Front
    # Face 2 [9, 12, 20, 21] - Back
    # Face 3 [2, 8, 9, 10] - Top
    # Face 4 [3, 11, 12, 13] - Bottom
    # Face 5 [4, 8, 11, 20] - Right
    # Face 6 [10, 13, 21, 5] - Left
    cube_planes = []
    cube_faces = \
    [
        [2, 3, 4, 5],
        [9, 12, 20, 21],
        [2, 8, 9, 10],
        [3, 11, 12, 13],
        [4, 8, 11, 20],
        [10, 13, 21, 5],
    ]
    for cube_face in cube_faces:
        trimmed_shapes = []
        for cube_edge in cube_face:
            curve = edges['curves'][cube_edge]
            curve_as_object = Curve()
            curve_as_object.points = curve['pv_points']
            curve_as_object.lines = curve['pv_lines']
            curve_as_object.index = cube_edge
            trimmed_shapes.append(trim_curve_with_primitive(curve_as_object, primitives))
        trim_in_same_plane = trim_collect_shapes_that_are_in_the_same_plane(trimmed_shapes)
        combined = BRepAlgoAPI_Fuse(trim_in_same_plane[0], trim_in_same_plane[1]).Shape()
        cube_planes.append(combined)
    front_and_back_planes = BRepAlgoAPI_Fuse(cube_planes[0], cube_planes[1]).Shape()
    top_and_bottom_planes = BRepAlgoAPI_Fuse(cube_planes[2], cube_planes[3]).Shape()
    right_and_left_planes = BRepAlgoAPI_Fuse(cube_planes[4], cube_planes[5]).Shape()
    cube = BRepAlgoAPI_Fuse(BRepAlgoAPI_Fuse(front_and_back_planes, top_and_bottom_planes).Shape(), right_and_left_planes).Shape()
    return cube, list(set(num for sublist in cube_faces for num in sublist))

def trim_object(edges, primitives):
    primitives_as_object = collect_primitives_as_objects(primitives)
    combined_cube, cube_indices = trim_cube(edges, primitives_as_object)
    trimmed_cylinders = []
    for curve_index, curve in enumerate(edges['curves']):
        if (curve_index not in cube_indices):
            curve_as_object = Curve()
            curve_as_object.points = curve['pv_points']
            curve_as_object.lines = curve['pv_lines']
            curve_as_object.index = curve_index
            trimmed_cylinders.append(trim_curve_with_primitive(curve_as_object, primitives_as_object))
    combined_cylinders = None
    if len(trimmed_cylinders) != 0:
        combined_cylinders = trimmed_cylinders[0]
        for trimmed_shape in trimmed_cylinders[1:]: 
            combined_cylinders = BRepAlgoAPI_Fuse(combined_cylinders, trimmed_shape).Shape()
    builder = BRep_Builder()
    result_compound = TopoDS_Compound()
    builder.MakeCompound(result_compound)
    builder.Add(result_compound, combined_cube)
    builder.Add(result_compound, combined_cylinders)
    return result_compound

# ===================================================
# Utilities
# ===================================================

def display_scene_OCCT(display_shapes, show_domain = False):
    from OCC.Display.SimpleGui import init_display
    if show_domain:
        domain = make_domain_OCCT()
        for edge in domain:
            display_shapes.append(edge)
    display, start_display, _, _ = init_display()
    for display_shape in display_shapes:
        if display_shape is not None:
            display.DisplayShape(display_shape, update=False, transparency=0.0)
    display.FitAll()
    start_display()

def read_data(surfaces_file, curves_file):
    primitives = None
    edges = None
    if (os.path.exists(surfaces_file) and os.path.exists(curves_file)):
        with open(curves_file) as file:
            edges = json.load(file)
        with open(surfaces_file) as file:
            primitives = json.load(file)
    else:
        print("Error reading files", surfaces_file, curves_file)
        exit(-1)
    return edges, primitives

def save_to_step(shape : TopoDS_Compound):
    writer = STEPControl_Writer()
    writer.Transfer(shape, STEPControl_AsIs)
    writer.Write("trimmed.step")

def save_to_stl(shape : TopoDS_Compound):
    mesh = BRepMesh_IncrementalMesh(shape, 0.001)
    mesh.Perform()
    print("Saving file trimmed.stl")
    writer = StlAPI_Writer()
    writer.Write(shape, "trimmed.stl")

# ===================================================
# Entry point
# ===================================================

if __name__ == '__main__':
    display_on_screen = True
    edges, primitives = read_data("input_data/surface_info.json", "input_data/topo.json")
    trimmed_object_from_primitives = trim_object(edges, primitives)
    save_to_step(trimmed_object_from_primitives)
    save_to_stl(trimmed_object_from_primitives)
    if display_on_screen:
        display_shapes = []
        display_shapes.append(trimmed_object_from_primitives)
        for curve in edges['curves']:
            display_shapes.extend(make_curve_OCCT(curve['pv_points'], curve['pv_lines']))
        display_scene_OCCT(display_shapes)