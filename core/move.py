import math
from pxr import Gf

class HumanController:
    def __init__(self, stage, human_path):
        self.human_prim = stage.GetPrimAtPath(human_path)
        self.initial_pos = self._get_initial_pos()
        self.amplitude = 1.5
        self.frequency = 0.01

    def _get_initial_pos(self):
        if self.human_prim.IsValid():
            return self.human_prim.GetAttribute("xformOp:translate").Get()
        return Gf.Vec3d(0, 0, 0)

    def move_human(self, current_time):
        if self.human_prim.IsValid():
            offset_x = self.amplitude * math.sin(current_time * self.frequency * 2 * math.pi)
            new_pos = Gf.Vec3d(self.initial_pos[0] + offset_x, self.initial_pos[1], self.initial_pos[2])
            self.human_prim.GetAttribute("xformOp:translate").Set(new_pos)