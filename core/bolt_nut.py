import math

from pxr import Gf, UsdGeom, UsdPhysics


def _polygon_radius_at_angle(radius, sides, angle):
    if sides < 3:
        return radius
    half_angle = math.pi / sides
    sector = (angle + half_angle) % (2.0 * math.pi / sides) - half_angle
    return radius * math.cos(half_angle) / math.cos(sector)


def _apply_rigid_body(prim):
    UsdPhysics.RigidBodyAPI.Apply(prim)


def _apply_collision(prim):
    UsdPhysics.CollisionAPI.Apply(prim)


def _create_prism_mesh(
    stage,
    prim_path,
    radius,
    height,
    outer_sides,
    inner_radius=None,
    inner_sides=None,
):
    xform = UsdGeom.Xform.Define(stage, prim_path)
    mesh = UsdGeom.Mesh.Define(stage, f"{prim_path}/hex_mesh")
    points = []
    face_counts = []
    face_indices = []
    use_hole = inner_radius is not None and 0.0 < inner_radius < radius
    if not use_hole:
        for i in range(outer_sides):
            angle = 2.0 * math.pi * i / outer_sides
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            points.append(Gf.Vec3f(x, y, 0.0))
            points.append(Gf.Vec3f(x, y, height))
        face_counts = [outer_sides, outer_sides] + [4] * outer_sides
        bottom = [i * 2 for i in range(outer_sides)]
        top = [i * 2 + 1 for i in range(outer_sides)]
        face_indices.extend(bottom)
        face_indices.extend(reversed(top))
        for i in range(outer_sides):
            next_i = (i + 1) % outer_sides
            face_indices.extend([
                bottom[i],
                bottom[next_i],
                top[next_i],
                top[i],
            ])
    else:
        inner_shape_sides = inner_sides or outer_sides
        ring_sides = max(outer_sides, inner_shape_sides)
        for i in range(ring_sides):
            angle = 2.0 * math.pi * i / ring_sides
            outer_r = _polygon_radius_at_angle(radius, outer_sides, angle)
            points.append(Gf.Vec3f(outer_r * math.cos(angle), outer_r * math.sin(angle), 0.0))
        for i in range(ring_sides):
            angle = 2.0 * math.pi * i / ring_sides
            outer_r = _polygon_radius_at_angle(radius, outer_sides, angle)
            points.append(Gf.Vec3f(outer_r * math.cos(angle), outer_r * math.sin(angle), height))
        for i in range(ring_sides):
            angle = 2.0 * math.pi * i / ring_sides
            inner_r = _polygon_radius_at_angle(inner_radius, inner_shape_sides, angle)
            points.append(Gf.Vec3f(inner_r * math.cos(angle), inner_r * math.sin(angle), 0.0))
        for i in range(ring_sides):
            angle = 2.0 * math.pi * i / ring_sides
            inner_r = _polygon_radius_at_angle(inner_radius, inner_shape_sides, angle)
            points.append(Gf.Vec3f(inner_r * math.cos(angle), inner_r * math.sin(angle), height))

        outer_bottom_start = 0
        outer_top_start = ring_sides
        inner_bottom_start = ring_sides * 2
        inner_top_start = ring_sides * 3

        for i in range(ring_sides):
            next_i = (i + 1) % ring_sides
            ob_i = outer_bottom_start + i
            ob_next = outer_bottom_start + next_i
            ot_i = outer_top_start + i
            ot_next = outer_top_start + next_i
            face_counts.append(4)
            face_indices.extend([ob_i, ob_next, ot_next, ot_i])

        for i in range(ring_sides):
            next_i = (i + 1) % ring_sides
            ib_i = inner_bottom_start + i
            ib_next = inner_bottom_start + next_i
            it_i = inner_top_start + i
            it_next = inner_top_start + next_i
            face_counts.append(4)
            face_indices.extend([ib_i, it_i, it_next, ib_next])

        for i in range(ring_sides):
            next_i = (i + 1) % ring_sides
            ot_i = outer_top_start + i
            ot_next = outer_top_start + next_i
            it_i = inner_top_start + i
            it_next = inner_top_start + next_i
            face_counts.append(4)
            face_indices.extend([ot_i, ot_next, it_next, it_i])

        for i in range(ring_sides):
            next_i = (i + 1) % ring_sides
            ob_i = outer_bottom_start + i
            ob_next = outer_bottom_start + next_i
            ib_i = inner_bottom_start + i
            ib_next = inner_bottom_start + next_i
            face_counts.append(4)
            face_indices.extend([ob_i, ib_i, ib_next, ob_next])

    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr(face_counts)
    mesh.CreateFaceVertexIndicesAttr(face_indices)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateSubdivisionSchemeAttr("none")
    return xform


class Bolt:
    def __init__(
        self,
        assembly_path="/World/bolt_assembly_01",
        head_radius=0.05,
        head_height=0.07,
        shaft_radius=0.03,
        shaft_height=0.15,
        sides=32,
        offset_from_nut=None,
    ):
        self.assembly_path = assembly_path
        self.head_radius = head_radius
        self.head_height = head_height
        self.shaft_radius = shaft_radius
        self.shaft_height = shaft_height
        self.sides = sides
        self.offset_from_nut = offset_from_nut or Gf.Vec3d(0.3, 0.0, 0.0)

    def create(self, stage, base_pos):
        root = UsdGeom.Xform.Define(stage, self.assembly_path)
        UsdGeom.XformCommonAPI(root).SetTranslate(base_pos)
        _apply_rigid_body(root.GetPrim())

        head_xform = _create_prism_mesh(
            stage,
            f"{self.assembly_path}/head",
            self.head_radius,
            self.head_height,
            self.sides,
        )
        UsdGeom.XformCommonAPI(head_xform).SetTranslate(Gf.Vec3d(0.0, 0.0, -self.head_height / 2.0))

        shaft_xform = _create_prism_mesh(
            stage,
            f"{self.assembly_path}/shaft",
            self.shaft_radius,
            self.shaft_height,
            self.sides,
        )
        UsdGeom.XformCommonAPI(shaft_xform).SetTranslate(Gf.Vec3d(0.0, 0.0, self.head_height / 2.0))

        _apply_collision(stage.GetPrimAtPath(f"{self.assembly_path}/head/hex_mesh"))
        _apply_collision(stage.GetPrimAtPath(f"{self.assembly_path}/shaft/hex_mesh"))
        return root


class Nut:
    def __init__(
        self,
        prim_path="/World/hexagon_01",
        radius=0.05,
        inner_radius=None,
        height=0.05,
        outer_sides=6,
        inner_sides=32,
        z_offset=1.0,
    ):
        self.prim_path = prim_path
        self.radius = radius
        self.inner_radius = inner_radius if inner_radius is not None else radius * 0.6
        self.height = height
        self.outer_sides = outer_sides
        self.inner_sides = inner_sides
        self.z_offset = z_offset

    def create(self, stage, center):
        xform = _create_prism_mesh(
            stage,
            self.prim_path,
            self.radius,
            self.height,
            self.outer_sides,
            inner_radius=self.inner_radius,
            inner_sides=self.inner_sides,
        )
        UsdGeom.XformCommonAPI(xform).SetTranslate(center)
        _apply_rigid_body(xform.GetPrim())
        _apply_collision(stage.GetPrimAtPath(f"{self.prim_path}/hex_mesh"))
        return xform
