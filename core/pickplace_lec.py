from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})


import numpy as np
import sys
import carb
import asyncio
import omni

from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import SurfaceGripper
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.api.world import World


from isaacsim.robot.manipulators.examples.universal_robots.controllers.pick_place_controller import PickPlaceController


class Tutorial_UR10:
    def __init__(self) -> None:
        self.usd_path = "/home/rokey/Desktop/DTHRC/DTHRC/assets/env_gripper.usd"
        self._world = None
        self._placing_position = np.array([0.7, 0.7, 0.0515 / 2.0])
        self._end_effector_offset = np.array([0, 0, 0.02])
        self._task_done = False
        
        self.my_controller = None
        self.articulation_controller = None
        self.cube = None
        self.robots = None
        return

    
    def setup_scene(self):
        import os
        import omni.usd
        if os.path.exists(self.usd_path):
            omni.usd.get_context().open_stage(self.usd_path)
        else:
            raise FileNotFoundError(f"USD file not found at {self.usd_path}")
        self._world = World(stage_units_in_meters=1.0, physics_dt=1/200, rendering_dt=20/200)
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            sys.exit()

        asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        robot = add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR10")
        robot.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")
        gripper = SurfaceGripper(
            end_effector_prim_path="/World/UR10/ee_link", surface_gripper_path="/World/UR10/ee_link/SurfaceGripper"
        )
        ur10 = self._world.scene.add(
            SingleManipulator(
                prim_path="/World/UR10", name="my_ur10", end_effector_prim_path="/World/UR10/ee_link", gripper=gripper
            )
        )
        ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

        self._world.scene.add(
            DynamicCuboid(
                name="cube",
                position=np.array([0.3, 0.3, 0.3]), 
                prim_path="/World/Cube",
                scale=np.array([0.0515, 0.0515, 0.0515]),
                size=1.0,
                color=np.array([0, 0, 1]),
            )
        )
        return

    def setup_post_load(self):
        self._world.reset()
        self.robots = self._world.scene.get_object("my_ur10")
        self.cube = self._world.scene.get_object("cube")
        self.my_controller = PickPlaceController(
            name="pick_place_controller", 
            gripper=self.robots.gripper, 
            robot_articulation=self.robots
        )
        self.articulation_controller = self.robots.get_articulation_controller()

    def physics_step(self):
        picking_position = self.cube.get_world_pose()[0]

        actions = self.my_controller.forward(
            picking_position=picking_position,
            placing_position=self._placing_position,
            current_joint_positions=self.robots.get_joint_positions(),
            end_effector_offset=self._end_effector_offset,
        )

        self.articulation_controller.apply_action(actions)

        if self.my_controller.is_done():
            carb.log_info("Pick and Place �묒뾽 �꾨즺")
            self._task_done = True
        return

    async def setup_pre_reset(self):
        if self.my_controller is not None:
            self.my_controller.reset()
        self._task_done = False
        return
    
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    frame_count = 0

if __name__=="__main__":
    tutorial = Tutorial_UR10()
    tutorial.setup_scene()
    tutorial.setup_post_load()

    # asyncio.get_event_loop().run_until_complete(tutorial.setup_post_load())
    while simulation_app.is_running():
        tutorial._world.step(render=True)

        if tutorial._world.is_playing():
            # if tutorial._world.current_time_step_index == 0:
            #     tutorial._world.reset()
            #     tutorial.my_controller.reset()
            tutorial.physics_step()


    simulation_app.close()

    