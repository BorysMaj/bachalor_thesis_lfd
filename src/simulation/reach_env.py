"""
Custom robosuite environment: ReachTask
Robot must move its end-effector to touch a small target block on the table.
No camera, state-based observations only.

Usage:
    import robosuite as suite
    from src.simulation.reach_env import ReachTask

    env = suite.make("ReachTask", robots="Panda", has_renderer=True, ...)
"""

import numpy as np
from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
from robosuite.models.arenas import TableArena
from robosuite.models.objects import BoxObject
from robosuite.models.tasks import ManipulationTask
from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import UniformRandomSampler
from robosuite.environments.base import register_env


@register_env
class ReachTask(ManipulationEnv):
    """
    Move the end-effector to touch a small target block on the table.

    Observations (state-based, no camera):
        robot0_eef_pos       (3,)  end-effector position
        robot0_eef_quat      (4,)  end-effector orientation
        robot0_gripper_qpos  (2,)  gripper finger positions
        target_pos           (3,)  target block centre position
        eef_to_target        (3,)  vector from eef to target
    """

    def __init__(
        self,
        robots,
        env_configuration="default",
        controller_configs=None,
        gripper_types="default",
        base_types="default",
        initialization_noise="default",
        table_full_size=(0.8, 0.8, 0.05),
        table_friction=(1.0, 5e-3, 1e-4),
        use_camera_obs=False,
        use_object_obs=True,
        reward_shaping=True,
        reach_threshold=0.05, # success if eef within 5 cm of target
        placement_initializer=None,
        has_renderer=False,
        has_offscreen_renderer=False,
        render_camera="frontview",
        render_collision_mesh=False,
        render_visual_mesh=True,
        render_gpu_device_id=-1,
        control_freq=20,
        lite_physics=True,
        horizon=300,
        ignore_done=True,
        hard_reset=True,
        camera_names="agentview",
        camera_heights=256,
        camera_widths=256,
        camera_depths=False,
        camera_segmentations=None,
        renderer="mjviewer",
        renderer_config=None,
        seed=None,
    ):
        self.table_full_size = table_full_size
        self.table_friction = table_friction
        self.table_offset = np.array((0, 0, 0.8))

        self.reach_threshold = reach_threshold
        self.reward_shaping = reward_shaping

        self.use_object_obs = use_object_obs

        self.placement_initializer = placement_initializer

        super().__init__(
            robots=robots,
            env_configuration=env_configuration,
            controller_configs=controller_configs,
            base_types=base_types,
            gripper_types=gripper_types,
            initialization_noise=initialization_noise,
            use_camera_obs=use_camera_obs,
            has_renderer=has_renderer,
            has_offscreen_renderer=has_offscreen_renderer,
            render_camera=render_camera,
            render_collision_mesh=render_collision_mesh,
            render_visual_mesh=render_visual_mesh,
            render_gpu_device_id=render_gpu_device_id,
            control_freq=control_freq,
            lite_physics=lite_physics,
            horizon=horizon,
            ignore_done=ignore_done,
            hard_reset=hard_reset,
            camera_names=camera_names,
            camera_heights=camera_heights,
            camera_widths=camera_widths,
            camera_depths=camera_depths,
            camera_segmentations=camera_segmentations,
            renderer=renderer,
            renderer_config=renderer_config,
            seed=seed,
        )


    # Build the scene

    def _load_model(self):
        super()._load_model()

        # Position robot at table edge
        xpos = self.robots[0].robot_model.base_xpos_offset["table"](self.table_full_size[0])
        self.robots[0].robot_model.set_base_xpos(xpos)

        mujoco_arena = TableArena(
            table_full_size=self.table_full_size,
            table_friction=self.table_friction,
            table_offset=self.table_offset,
        )
        mujoco_arena.set_origin([0, 0, 0])

        # Small bright green target block — easy to see, easy to touch
        self.target = BoxObject(
            name="target",
            size_min=(0.02, 0.02, 0.02),
            size_max=(0.02, 0.02, 0.02),
            rgba=[0.2, 0.9, 0.2, 1],
            rng=self.rng,
        )

        if self.placement_initializer is not None:
            self.placement_initializer.reset()
            self.placement_initializer.add_objects(self.target)
        else:
            self.placement_initializer = UniformRandomSampler(
                name="ObjectSampler",
                mujoco_objects=self.target,
                x_range=[-0.15, 0.15],
                y_range=[-0.15, 0.15],
                rotation=None,
                ensure_object_boundary_in_range=False,
                ensure_valid_placement=True,
                reference_pos=self.table_offset,
                z_offset=0.01,
                rng=self.rng,
            )

        self.model = ManipulationTask(
            mujoco_arena=mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=self.target,
        )


    # References

    def _setup_references(self):
        super()._setup_references()
        self.target_body_id = self.sim.model.body_name2id(self.target.root_body)


    # Reset

    def _reset_internal(self):
        super()._reset_internal()
        if not self.deterministic_reset:
            object_placements = self.placement_initializer.sample()
            for obj_pos, obj_quat, obj in object_placements.values():
                self.sim.data.set_joint_qpos(
                    obj.joints[0],
                    np.concatenate([np.array(obj_pos), np.array(obj_quat)])
                )


    # Reward

    def reward(self, action=None):
        if self._check_success():
            return 1.0

        if self.reward_shaping:
            dist = self._gripper_to_target(
                gripper=self.robots[0].gripper,
                target=self.target.root_body,
                target_type="body",
                return_distance=True,
            )
            return -dist # closer = higher reward (less negative)

        return 0.0


    # Success check
    
    def _check_success(self):
        dist = self._gripper_to_target(
            gripper=self.robots[0].gripper,
            target=self.target.root_body,
            target_type="body",
            return_distance=True,
        )
        return bool(dist < self.reach_threshold)


    # Observations
    
    def _setup_observables(self):
        observables = super()._setup_observables()

        if self.use_object_obs:
            modality = "object"

            @sensor(modality=modality)
            def target_pos(obs_cache):
                return np.array(self.sim.data.body_xpos[self.target_body_id])

            @sensor(modality=modality)
            def eef_to_target(obs_cache):
                target = np.array(self.sim.data.body_xpos[self.target_body_id])
                eef = np.array(self.sim.data.site_xpos[
                    self.robots[0].eef_site_id["right"]
                    if isinstance(self.robots[0].eef_site_id, dict)
                    else self.robots[0].eef_site_id
                ])
                return target - eef

            for s in [target_pos, eef_to_target]:
                observables[s.__name__] = Observable(
                    name=s.__name__,
                    sensor=s,
                    sampling_rate=self.control_freq,
                )

        return observables