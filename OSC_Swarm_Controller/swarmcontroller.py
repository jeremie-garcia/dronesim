import os
import sys

# Comment --> for debuging Amania Computer
current_dir = os.path.dirname(os.path.abspath(__file__))
grandparent_dir = os.path.dirname(os.path.dirname(current_dir))
dronesim_path = os.path.join(grandparent_dir, "dronesim")
sys.path.insert(0, dronesim_path)

import pybullet as p
import json

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QObject, QTimer

import argparse

from dronesim.control.INDIControl import INDIControl
from dronesim.envs.BaseAviary import DroneModel, Physics

from dronesim.envs.CtrlAviary import CtrlAviary
from dronesim.utils.trajGen import *
from dronesim.utils.utils import str2bool

from pgflow import Cases
from pgflow.arena import ArenaMap
from pgflow.utils.simulation_utils import step_simulation, set_new_attribute

CONTROL_FREQ = 30  # in HZ
CONTROL_RATE = int(1000 / CONTROL_FREQ)  # in ms

# at the moment, only convex buildings are supported for plotting
filename = 'VerticesData.json'  # case name from scenebuilder
ArenaMap.size = 0.1
ArenaMap.inflation_radius = 0.1
case = Cases.get_case(filename, 'scenebuilder')
# Load polygons from the text file
with open(filename, "r") as f:
    # Load the JSON data
    data = json.load(f)

num_drones = len(case.vehicle_list)
NB_OF_DRONES = num_drones
case.mode = 'fancy'
case.building_detection_threshold = 5
set_new_attribute(case, 'source_strength', 3)
target_speed = 2
set_new_attribute(case, 'max_speed', target_speed)


class SwarmController(QObject):

    def __init__(self):
        super().__init__()

        self.ARGS = None

        print("OSC server started")
        sys.stdout.flush()

        self.env = None
        self.NB_OF_DRONES = NB_OF_DRONES

        self.neighbourhood_radius = 4
        self.formation_2D = False

        self.target = case.vehicle_list[0].goal

        self.velocities = {i: {'vx': 0, 'vy': 0, 'vz': 0} for i in range(self.NB_OF_DRONES)}
        self.droneFPVIndex = -1
        self.forwardFrameCounter = 0
        self.stopFrameCounter = 0

        self.rotation = [0.0 for i in range(self.NB_OF_DRONES)]

        self.create_flyingsim()

        self.action = {
            str(i): np.array([0.3, 0.3, 0.3, 0.3]) for i in range(self.NB_OF_DRONES)
        }

        self.simulation_timer = QTimer()
        self.simulation_timer.timeout.connect(self.update_simulation)

        self.start_simulation()
        # print("simulation started")

    def create_flyingsim(self):

        ## Define and parse (optional) arguments for the script ##
        parser = argparse.ArgumentParser(
            description="Helix flight script using CtrlAviary or VisionAviary and DSLPIDControl"
        )
        parser.add_argument(
            "--drone",
            default=["robobee"] * num_drones,  # hexa_6DOF_simple
            type=list,
            help="Drone model (default: CF2X)",
            metavar="",
            choices=[DroneModel],
        )
        parser.add_argument(
            "--num_drones",
            default=self.NB_OF_DRONES,
            type=int,
            help="Number of drones (default: 3)",
            metavar="",
        )
        parser.add_argument(
            "--physics",
            default="pyb",
            type=Physics,
            help="Physics updates (default: PYB)",
            metavar="",
            choices=Physics,
        )
        parser.add_argument(
            "--vision",
            default=False,
            type=str2bool,
            help="Whether to use VisionAviary (default: False)",
            metavar="",
        )
        parser.add_argument(
            "--gui",
            default=True,
            type=str2bool,
            help="Whether to use PyBullet GUI (default: True)",
            metavar="",
        )
        parser.add_argument(
            "--record_video",
            default=False,
            type=str2bool,
            help="Whether to record a video (default: False)",
            metavar="",
        )
        parser.add_argument(
            "--plot",
            default=False,
            type=str2bool,
            help="Whether to plot the simulation results (default: True)",
            metavar="",
        )
        parser.add_argument(
            "--user_debug_gui",
            default=False,
            type=str2bool,
            help="Whether to add debug lines and parameters to the GUI (default: False)",
            metavar="",
        )
        parser.add_argument(
            "--aggregate",
            default=True,
            type=str2bool,
            help="Whether to aggregate physics steps (default: True)",
            metavar="",
        )
        parser.add_argument(
            "--obstacles",
            default=False,
            type=str2bool,
            help="Whether to add obstacles to the environment (default: True)",
            metavar="",
        )
        parser.add_argument(
            "--simulation_freq_hz",
            default=240,
            type=int,
            help="Simulation frequency in Hz (default: 240)",
            metavar="",
        )
        parser.add_argument(
            "--control_freq_hz",
            default=CONTROL_FREQ,
            type=int,
            help="Control frequency in Hz (default: 48)",
            metavar="",
        )
        parser.add_argument(
            "--duration_sec",
            default=20000,
            type=int,
            help="Duration of the simulation in seconds (default: 5)",
            metavar="",
        )
        parser.add_argument(
            "--neighbourhood_radius",
            default=2,
            type=float,
            help="Neighbourhood radius for the drones (default: 2)"
        )
        parser.add_argument(
            "--formation_2D",
            default=False,
            type=str2bool,
            help="Whether to use 2D formation (default: False)",
        )
        ARGS = parser.parse_args()

        if ARGS.num_drones:
            self.NB_OF_DRONES = ARGS.num_drones
            self.rotation = [0.0 for i in range(self.NB_OF_DRONES)]

        if ARGS.neighbourhood_radius:
            self.neighbourhood_radius = ARGS.neighbourhood_radius

        if ARGS.formation_2D:
            self.formation_2D = ARGS.formation_2D

        #### Initialize the simulation #############################
        H = 0.50
        H_STEP = 0.05
        R = 2

        AGGR_PHY_STEPS = (
            int(ARGS.simulation_freq_hz / ARGS.control_freq_hz) if ARGS.aggregate else 1
        )
        # INIT_XYZS = np.array(
        #     [
        #         [
        #             i % 5,
        #             int(i / 5),
        #             0,
        #         ]
        #         for i in range(ARGS.num_drones)
        #     ]
        # )
        INIT_XYZS = np.array([v.position for v in case.vehicle_list])
        INIT_RPYS = np.array([[0.0, 0.0, 0.0] for i in range(ARGS.num_drones)])
        INIT_VELS = np.array([[0.0, 0.0, 0.0] for _ in range(ARGS.num_drones)])

        #### Initialize a circular trajectory ######################
        PERIOD = 15
        NUM_WP = ARGS.control_freq_hz * PERIOD

        TARGET_POS = np.zeros((NUM_WP, 3))
        for i in range(NUM_WP):
            TARGET_POS[i, :] = (
                R * np.cos((i / NUM_WP) * (4 * np.pi) + np.pi / 2) + INIT_XYZS[0, 0],
                R * np.sin((i / NUM_WP) * (4 * np.pi) + np.pi / 2) - R + INIT_XYZS[0, 1],
                0,
            )

        TARGET_RPYS = np.zeros((NUM_WP, 3))
        for i in range(NUM_WP):
            TARGET_RPYS[i, :] = [0.0, 0.0, 0.0]  # 0.4+(i*1./200)]

        #### Create the environment
        self.env = CtrlAviary(
            drone_model=ARGS.drone,
            num_drones=NB_OF_DRONES,
            initial_xyzs=INIT_XYZS,
            initial_vels=INIT_VELS,
            initial_rpys=INIT_RPYS,
            physics=ARGS.physics,
            neighbourhood_radius=10,
            freq=ARGS.simulation_freq_hz,
            aggregate_phy_steps=AGGR_PHY_STEPS,
            gui=ARGS.gui,
            record=ARGS.record_video,
            obstacles=ARGS.obstacles,
            user_debug_gui=ARGS.user_debug_gui,
        )

        #### Obtain the PyBullet Client ID from the environment ####
        PYB_CLIENT = self.env.getPyBulletClient()
        self.ARGS = ARGS

        polygons = []
        obstacles = data["scenebuilder"]["buildings"]
        # max_X = max(abs(v.goal[0]) for v in case.vehicle_list)
        # max_Y = max(abs(v.goal[1]) for v in case.vehicle_list)
        # load_building_ray = max(max_Y, max_X)
        # print(load_building_ray)
        sys.stdout.flush()
        for id, obs in enumerate(obstacles):
            # vertex = obs["vertices"][0]
            # for x in INIT_XYZS:
            # # if building is far from the drone or its goal, ignore it
            # if not (abs(x[0]-vertex[0])>load_building_ray or abs(x[1]-vertex[1])>load_building_ray):
            floor = np.array(obs["vertices"]).copy()
            floor[:, 2] = 0.0
            ceil = np.array(obs["vertices"]).copy()
            # ceil[:,2] = 3.0
            tmp = np.vstack((floor, ceil))
            polygons.append(tmp)
        # else:
        #     break

        # Create polygons in the simulation
        for polygon_vertices in polygons:
            polygon_id = p.createCollisionShape(p.GEOM_MESH, vertices=polygon_vertices)
            p.createMultiBody(baseCollisionShapeIndex=polygon_id)

        #### Initialize the controllers ############################
        self.ctrl = [INDIControl(drone_model=drone) for drone in ARGS.drone]
        #### Run the simulation ####################################
        self.CTRL_EVERY_N_STEPS = int(np.floor(self.env.SIM_FREQ / ARGS.control_freq_hz))

    def update_simulation(self):
        #### Step the simulation ###################################

        obs, reward, done, info = self.env.step(self.action)

        for vehicle in case.vehicle_list:  ## Update the target of the drones in case user sent a new target
            vehicle.goal = self.target

        for j in range(self.NB_OF_DRONES):
            position = [self.env.pos[j, 0], self.env.pos[j, 1], self.env.pos[j, 2]]
            rotation = [self.env.rpy[j, 0], self.env.rpy[j, 1], self.env.rpy[j, 2]]
            # print(f"{obs[str(j)]["state"]}")
            pos = obs[str(j)]["state"][:3]
            case.vehicle_list[j].position = pos
        step_simulation(case)

        #### Compute control for the current way point #############
        for j in range(self.env.NUM_DRONES):

            # if the drone doesn't move for a few frames, forwardFrameCounter (handles acceleration of the drone if going forward continuously) is reset
            if (self.velocities[j]['vx'] == 0 and self.velocities[j]['vy'] == 0 and self.velocities[j]['vz'] == 0):
                self.stopFrameCounter += 1
            if (self.stopFrameCounter > 10):
                self.forwardFrameCounter = 0
                self.stopFrameCounter = 0

            # if the drone is controlled in FPV, target velocities and rotations are those received by OSC instead of PGFlow
            if (j == self.droneFPVIndex): 
                desired_vector = np.array([self.velocities[j]['vx'], self.velocities[j]['vy'], self.velocities[j]['vz']])
                acceleration = (1 + self.forwardFrameCounter / 20)
                self.action[str(j)], _, _ = self.ctrl[j].computeControlFromState(
                    control_timestep=self.CTRL_EVERY_N_STEPS * self.env.TIMESTEP,
                    state=obs[str(j)]["state"],
                    target_pos = np.hstack([obs[str(j)]["state"][:2], obs[str(j)]["state"][2]]),
                    target_vel=desired_vector*target_speed*acceleration,
                    # target_acc=np.zeros(3),
                    target_rpy=self.rotation[j]*np.array([0,0,1])
                    # target_rpy_rates=np.zeros(3)
                )
                self.velocities[j] = {'vx': 0, 'vy': 0, 'vz': 0}

            # for the rest of the drones, the target velocities and rotations are those computed by PGFlow
            else:
                vehicle = case.vehicle_list[j]
                # if vehicle.state==1:
                #     desired_vector = np.array([0,0,-1])
                # else:
                desired_vector = vehicle.desired_vectors[-1]
                desired_vector = np.hstack([desired_vector, 0])

            if j == self.droneFPVIndex:  ##### if the drone is controlled in FPV, target velocities and rotations are those received by OSC instead of PGFlow
                desired_vector = np.array(
                    [self.velocities[j]['vx'], self.velocities[j]['vy'], self.velocities[j]['vz']])
                acceleration = (1 + self.forwardFrameCounter / 20)
                self.action[str(j)], _, _ = self.ctrl[j].computeControlFromState(
                    control_timestep=self.CTRL_EVERY_N_STEPS * self.env.TIMESTEP,
                    state=obs[str(j)]["state"],
                    target_pos=np.hstack([obs[str(j)]["state"][:2], obs[str(j)]["state"][2]]),
                    target_vel=desired_vector * target_speed * acceleration,
                    # target_acc=np.zeros(3),
                    target_rpy=self.rotation[j] * np.array([0, 0, 1])
                    # target_rpy_rates=np.zeros(3)
                )
                self.velocities[j] = {'vx': 0, 'vy': 0, 'vz': 0}
            else:
                vehicle = case.vehicle_list[j]
                # if vehicle.state==1:
                #     desired_vector = np.array([0,0,-1])
                # else:
                desired_vector = vehicle.desired_vectors[-1]
                desired_vector = np.hstack([desired_vector, 0])

                ##### NOTE HERE IS WHERE YOU WOULD PUT YOUR TARGET VELOCITY FOR THE DRONES TO FOLLOW
                self.action[str(j)], _, _ = self.ctrl[j].computeControlFromState(
                    control_timestep=self.CTRL_EVERY_N_STEPS * self.env.TIMESTEP,
                    state=obs[str(j)]["state"],
                    target_pos=np.hstack([obs[str(j)]["state"][:2], 2]),
                    target_vel=desired_vector * target_speed,
                    # target_acc=np.zeros(3),
                    target_rpy=[0, 0, np.arctan2(desired_vector[1], desired_vector[0])]
                    # target_rpy_rates=np.zeros(3),
                )

        #### Printout ##############################################
        #self.env.render()

    def start_simulation(self):
        self.simulation_timer.start(CONTROL_RATE)

    def stop_simulation(self):
        self.simulation_timer.stop()

    def close_env(self):
        ### Close the environment ###
        self.env.close()

    # EOF


if __name__ == "__main__":
    app = QApplication(sys.argv)
    controller = SwarmController()
    app.aboutToQuit.connect(controller.stop_simulation)
    app.aboutToQuit.connect(controller.close_env)
    app.exec()
