import math

from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics



def _polygon_radius_at_angle(radius, sides, angle):
    if sides < 3:
        return radius
    half_angle = math.pi / sides
    sector = (angle + half_angle) % (2.0 * math.pi / sides) - half_angle
    return radius * math.cos(half_angle) / math.cos(sector)


def _apply_rigid_body(prim):
    UsdPhysics.RigidBodyAPI.Apply(prim)


def _apply_collision(prim):
    if prim is None or not prim.IsValid():
        return
    UsdPhysics.CollisionAPI.Apply(prim)
    mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
    mesh_collision.CreateApproximationAttr().Set("convexHull")
    physx_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    if hasattr(physx_api, "CreateApproximationAttr"):
        physx_api.CreateApproximationAttr().Set("convexHull")
    elif hasattr(physx_api, "GetApproximationAttr"):
        attr = physx_api.GetApproximationAttr()
        if not attr:
            attr = prim.CreateAttribute("physxCollision:approximation", Sdf.ValueTypeNames.Token)
        attr.Set("convexHull")
    else:
        prim.CreateAttribute("physxCollision:approximation", Sdf.ValueTypeNames.Token).Set("convexHull")


def _create_prism_mesh(
    stage,
    prim_path,
    radius,
    height,
    outer_sides,
    inner_radius=None,
    inner_sides=None,
    create_xform=True,
):
    if create_xform:
        xform = UsdGeom.Xform.Define(stage, prim_path)
        mesh_path = f"{prim_path}/hex_mesh"
    else:
        xform = None
        mesh_path = prim_path
    mesh = UsdGeom.Mesh.Define(stage, mesh_path)
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
    return xform or mesh


class Bolt:
    _counter = 0

    def __init__(
        self,
        assembly_path="/World/bolt",
        # 볼트 사이즈 조정
        head_radius=0.05,
        head_height=0.05,
        shaft_radius=0.03,
        shaft_height=0.13,
        sides=32,
        offset_from_nut=None,
    ):
        self.assembly_path = self._next_path(assembly_path)
        self.head_radius = head_radius
        self.head_height = head_height
        self.shaft_radius = shaft_radius
        self.shaft_height = shaft_height
        self.sides = sides
        self.offset_from_nut = offset_from_nut or Gf.Vec3d(0.3, 0.0, 0.0)

    @classmethod
    def _next_path(cls, base_path):
        cls._counter += 1
        return f"{base_path}_{cls._counter:02d}"

    def create(self, stage, position):
        bolt_xform = UsdGeom.Xform.Define(stage, self.assembly_path)
        xform_api = UsdGeom.XformCommonAPI(bolt_xform)
        xform_api.SetTranslate(position)
        # 볼트 180도 회전하고 싶을 경우 위 한줄 주석, 아래 세 줄 주석 해제
        # xform_api.SetTranslate(position)
        # xform_api.SetRotate((180.0, 0.0, 0.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        # _apply_rigid_body(bolt_xform.GetPrim())
        _apply_rigid_body(bolt_xform.GetPrim())

        head_mesh = _create_prism_mesh(
            stage,
            f"{self.assembly_path}/head_hex_mesh",
            self.head_radius,
            self.head_height,
            self.sides,
            create_xform=False,
        )
        tip_offset = self.head_height / 2.0 + self.shaft_height
        head_translate = Gf.Vec3d(0.0, 0.0, -self.head_height / 2.0 - tip_offset)
        UsdGeom.XformCommonAPI(head_mesh).SetTranslate(head_translate)

        shaft_mesh = _create_prism_mesh(
            stage,
            f"{self.assembly_path}/shaft_hex_mesh",
            self.shaft_radius,
            self.shaft_height,
            self.sides,
            create_xform=False,
        )
        # 볼트 기둥 끝으로 xform 이동
        shaft_translate = Gf.Vec3d(0.0, 0.0, self.head_height / 2.0 - tip_offset)
        UsdGeom.XformCommonAPI(shaft_mesh).SetTranslate(shaft_translate)

        _apply_collision(stage.GetPrimAtPath(f"{self.assembly_path}/head_hex_mesh"))
        _apply_collision(stage.GetPrimAtPath(f"{self.assembly_path}/shaft_hex_mesh"))
        return bolt_xform


class Nut:
    _counter = 0

    def __init__(
        self,
        prim_path="/World/nut",
        radius=0.05,
        inner_radius=None,
        height=0.05,
        outer_sides=6,
        inner_sides=32,
        z_offset=1.0,
    ):
        self.prim_path = self._next_path(prim_path)
        self.radius = radius
        self.inner_radius = inner_radius if inner_radius is not None else radius * 0.6
        self.height = height
        self.outer_sides = outer_sides
        self.inner_sides = inner_sides
        self.z_offset = z_offset

    def create(self, stage, position):
        xform = _create_prism_mesh(
            stage,
            self.prim_path,
            self.radius,
            self.height,
            self.outer_sides,
            inner_radius=self.inner_radius,
            inner_sides=self.inner_sides,
        )

        if isinstance(position, Gf.Vec3d):
            base_position = position
        else:
            base_position = Gf.Vec3d(*position)
        UsdGeom.XformCommonAPI(xform).SetTranslate(base_position)
        mesh_prim = stage.GetPrimAtPath(f"{self.prim_path}/hex_mesh")
        if mesh_prim.IsValid():
            # Offset mesh so the xform origin sits on the nut top.
            UsdGeom.XformCommonAPI(mesh_prim).SetTranslate(
                Gf.Vec3d(0.0, 0.0, -self.height)
            )
        _apply_rigid_body(xform.GetPrim())
        _apply_collision(stage.GetPrimAtPath(f"{self.prim_path}/hex_mesh"))
        return xform

    @classmethod
    def _next_path(cls, base_path):
        cls._counter += 1
        return f"{base_path}_{cls._counter:02d}"
