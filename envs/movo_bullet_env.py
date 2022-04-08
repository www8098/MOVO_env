import pybullet as p
import numpy as np
import gym
from gym import spaces
from pybullet_data import getDataPath
import pybullet as p
from time import sleep
from Camera import setCameraPicAndGetPic
try:
    import importlib.resources as pkg_resources
except ImportError:
    import importlib_resources as pkg_resources


BASE_RADIUS = 1
BASE_THICKNESS = 1

"""
MOVO environnement using PyBullet.
time_step: time per step
frameskip
"""
class MoVoBulletEnv(gym.Env):

    metadata = {
        "render.modes": ["human"],
        "video.frames_per_second": 100,
    }

    def __init__(self, time_step=0.05, frameskip=12, render=False):
        super().__init__()

        # Init PyBullet in GUI or DIRECT mode
        self._render = render
        if self._render:
            cid = p.connect(p.SHARED_MEMORY)
            if (cid < 0):
                cid = p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        # 18 actions (servomotors)
        self.n_actions = 18  # 7 + 7 + 3 + 1
        self.action_space = spaces.Box(low=-1, high=1,
                                       shape=(self.n_actions,),
                                       dtype="float32")
        self.n_observation = 3*18+6
        self.observation_space = spaces.Box(low=-1, high=1,
                                            shape=(self.n_observation,),
                                            dtype="float32")
        self.observation = np.zeros(self.n_observation, dtype="float32")

        # Environment timestep and constants
        self.dt = time_step
        self.frameskip = frameskip
        self.servo_max_speed = 6.308  # rad/s
        self.servo_max_torque = 1.57  # N.m

        # Seed random number generator
        self.seed()

        # Init world
        p.setTimeStep(self.dt / self.frameskip)  # between 0.001 and 0.01 s
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)  # Newton's apple
        p.setAdditionalSearchPath(getDataPath())  # Add pybullet_data
        p.loadURDF("plane.urdf")  # Load a ground

        # Load robot
        flags = p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE
        with pkg_resources.path("gym_kraby", "data") as path:
            self.robot_id = p.loadURDF(str(path / 'hexapod.urdf'), flags=flags)

        # Get all motorized joints id and name (which are revolute joints)
        self.joint_list = [j for j in range(p.getNumJoints(self.robot_id))
                           if p.getJointInfo(self.robot_id, j)[2] == p.JOINT_REVOLUTE]

    def reset(self):
        # Reset body position/orientation
        p.resetBasePositionAndOrientation(
            self.robot_id,
            [0, 0, 0.2],
            [0, 0, 0, 1],
        )

        # Reset all joint using normal distribution
        for j in self.joint_list:
            p.resetJointState(self.robot_id, j,
                              np.random.uniform(low=-np.pi/4, high=np.pi/4))

        # Set random target and put it in observations
        self.target_position = np.array([0, -1, 0.1])
        self.observation[-3:] = self.target_position

        # Show target as a crosshair
        p.removeAllUserDebugItems()
        p.addUserDebugLine(self.target_position - [0, 0, 0.1],
                           self.target_position + [0, 0, 0.1],
                           [0, 0, 0], 2)
        p.addUserDebugLine(self.target_position - [0, 0.1, 0],
                           self.target_position + [0, 0.1, 0],
                           [0, 0, 0], 2)

        # Return observation
        self._update_observation()
        return self.observation

    def step(self, action):
        # Update servomotors
        transformed_action = np.array(action) * self.servo_max_speed
        max_torques = [self.servo_max_torque] * self.n_actions
        p.setJointMotorControlArray(bodyIndex=self.robot_id,
                                    jointIndices=self.joint_list,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=action,
                                    forces=max_torques)

        # Wait for environment step
        for _ in range(self.frameskip):  # step self.dt
            p.stepSimulation()
            # setCameraPic(self.robot_id)
            if self._render:
                sleep(self.dt / self.frameskip)  # realtime

        # Return observation, reward and done
        self._update_observation()
        reward = self._get_reward()
        done = bool(self.observation[-4] < 0.08)  # Has fallen?
        return self.observation, reward, done, {}

    def render(self, mode='human'):
        if mode == "human":
            return np.array([])
        else:
            raise NameError

        # position = p.getBasePositionAndOrientation(self.robot_id)[0]
        # view_matrix = p.computeViewMatrixFromYawPitchRoll(
        #     cameraTargetPosition=position,
        #     distance=0.6,
        #     yaw=30,
        #     pitch=-30,
        #     roll=0,
        #     upAxisIndex=2,
        # )
        # proj_matrix = p.computeProjectionMatrixFOV(
        #     fov=60,
        #     aspect=960. / 720,
        #     nearVal=0.1,
        #     farVal=100.0,
        # )
        # _, _, px, _, _ = p.getCameraImage(
        #     width=960,
        #     height=720,
        #     viewMatrix=view_matrix,
        #     projectionMatrix=proj_matrix,
        #     renderer=p.ER_TINY_RENDERER,
        # )
        # rgb_array = np.array(px)
        # rgb_array = rgb_array[:, :, :3]
        # return rgb_array

    @staticmethod
    def seed(seed=None):
        np.random.seed(seed)

    def _get_reward(self):
        position = self.observation[-6:-3]
        target_distance = np.square(position - self.target_position).sum()

        # Comsuption is speed * torque
        speeds = self.observation[1:-6:3]
        torques = self.observation[2:-6:3]
        comsuption = self.dt * abs(sum(speeds * torques))
        w = 0.008  # comsuption weight

        # Compute reward
        reward = 1 - target_distance - w * comsuption
        return reward

    def _update_observation(self):
        """
        Update the observation from BulletPhysics
        """

        # Each servomotor position, speed and torque
        all_states = p.getJointStates(self.robot_id, self.joint_list)
        for i, (pos, vel, _, tor) in enumerate(all_states):
            self.observation[3*i:3*i+3] = [
                2 * pos / np.pi,
                vel / self.servo_max_speed,
                tor / self.servo_max_torque
            ]

        # Sometimes 1.0 is greater than 1
        self.observation = np.clip(self.observation, -1., 1.)

        # Robot position and orientation
        pos, ori = p.getBasePositionAndOrientation(self.robot_id)
        self.observation[-6:] = list(pos) + list(p.getEulerFromQuaternion(ori))
        self.observation[-3:] /= np.pi  # normalization
