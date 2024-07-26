import os
import sys
import time

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

from pgflow import Cases, vehicle
from pgflow.arena import ArenaMap
from pgflow.utils.simulation_utils import step_simulation, set_new_attribute

CONTROL_FREQ = 30   # in HZ
CONTROL_RATE = int(1000 / CONTROL_FREQ)  # in ms

# at the moment, only convex buildings are supported for plotting
filename = 'VerticesData.json'  # case name from scenebuilder
ArenaMap.size = 0.4 # panel size; basically more panels means better precision but more calculation, this number can be lowered to 5 panels by building so for a 3x3 building, size=0.6 max.
ArenaMap.inflation_radius = 1.0 # buildings are bigger in the simulation so that vehicles turn earlier in anticipation
case = Cases.get_case(filename, 'scenebuilder')
# Load polygons from the text file
with open(filename, "r") as f:
    # Load the JSON data
    data = json.load(f)

num_drones = len(case.vehicle_list)
NB_OF_DRONES = num_drones
case.mode = '' # keeping this to none is the best option speed-wise, others increase precision but take longer

case.building_detection_threshold = 3 # distance for drones to see buildings
# case.max_avoidance_distance = 1.5 # distance for drones to see each other
# set_new_attribute(case, 'imag_source_strength', 100) # building source strength, not working for now
# set_new_attribute(case, 'ARRIVAL_DISTANCE', 2) # meters from the target for the drones to consider they arrived at their destination
# set_new_attribute(case, 'turn_radius', 0.3) # max radius of turn
set_new_attribute(case, 'source_strength', 1) # vehicle source strength
# set_new_attribute(case, 'goal', np.array([14.06, -8.23, 0]))
target_speed = 2 # m/s 
FPV_speed = 4 # max speed higher just so FPV is not too boring
set_new_attribute(case, 'max_speed', target_speed) # max speed for the drones

# here for performance debug, prints every 500 frames time taken by one step of pgflow
frameIndex = 0 
time_taken_pgflow = [0]*500

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

        self.targets = [case.vehicle_list[i].goal for i in range(self.NB_OF_DRONES)]

        self.velocities = {i: {'vx': 0, 'vy': 0, 'vz': 0} for i in range(self.NB_OF_DRONES)}
        self.droneFPVIndex = -1
        self.forwardFrameCounter = 0
        self.stopFrameCounter = 0
        self.actionStrength = 1

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
            self.drone_model = ["robobee"] * self.NB_OF_DRONES

        if ARGS.neighbourhood_radius:
            self.neighbourhood_radius = ARGS.neighbourhood_radius

        if ARGS.formation_2D:
            self.formation_2D = ARGS.formation_2D

        #### Initialize the simulation #############################
        H = 0.50
        H_STEP = 0.05
        R = 2

        # myVehicle = Vehicle(
        #     source_strength=source_strength,
        #     imag_source_strength=imag_source_strength,
        # )
        # myVehicle.ID = ID
        # # FIXME the order the setting goal and initial position matters for the first entry
        # #  into desired vectors so be careful
        # myVehicle.Set_Goal(goal=goal, goal_strength=sink_strength)
        # myVehicle.set_initial_position(position)

        if ARGS.num_drones > len(case.vehicle_list):
            while len(case.vehicle_list) < ARGS.num_drones:
                n = len(case.vehicle_list)
                newvehicle = vehicle.Vehicle(source_strength=3, imag_source_strength=0)
                newvehicle.position = np.array([n % 5, int(n/5), 0])
                newvehicle.goal = case.vehicle_list[0].goal
                newvehicle.ID = "Drone "+str(n+1)
                newvehicle.personal_vehicle_dict = case.vehicle_list[0].personal_vehicle_dict
                newvehicle.arena = case.arena
                case.vehicle_list.append(newvehicle)
            self.targets = [case.vehicle_list[i].goal for i in range(self.NB_OF_DRONES)]
            self.velocities = {i: {'vx': 0, 'vy': 0, 'vz': 0} for i in range(self.NB_OF_DRONES)}
            self.rotation = [0.0 for i in range(self.NB_OF_DRONES)]
            self.action = {
                str(i): np.array([0.3, 0.3, 0.3, 0.3]) for i in range(self.NB_OF_DRONES)
            }
        # case.to_json()

        if ARGS.num_drones < len(case.vehicle_list):
            case.vehicle_list = case.vehicle_list[:ARGS.num_drones]

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
            drone_model=self.drone_model,
            num_drones=ARGS.num_drones,
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
        sys.stdout.flush()
        for id, obs in enumerate(obstacles):
            floor = np.array(obs["vertices"]).copy()
            floor[:, 2] = 0.0
            ceil = np.array(obs["vertices"]).copy()
            # ceil[:,2] = 3.0
            tmp = np.vstack((floor, ceil))
            polygons.append(tmp)

        # Create polygons in the simulation
        for polygon_vertices in polygons:
            polygon_id = p.createCollisionShape(p.GEOM_MESH, vertices=polygon_vertices)
            p.createMultiBody(baseCollisionShapeIndex=polygon_id)

        #### Initialize the controllers ############################
        self.ctrl = [INDIControl(drone_model=drone) for drone in self.drone_model]
        #### Run the simulation ####################################
        self.CTRL_EVERY_N_STEPS = int(np.floor(self.env.SIM_FREQ / ARGS.control_freq_hz))

    def update_simulation(self):

        #start timer
        global frameIndex

        # calculate time for env.step and print average after 500 frames 
        time_before_env_step = time.time()

        #### Step the simulation ###################################
        obs, reward, done, info = self.env.step(self.action)

        
        time_after_env_step = time.time()
        time_taken_pgflow[frameIndex] = (time_after_env_step - time_before_env_step)
        if frameIndex == 499:
            avg = sum(time_taken_pgflow)/500
            print("Time for env_step (dronesim) in ms: ", avg*1000)
            print("FPS max (dronesim): ", 1/avg)

        for i in range(self.NB_OF_DRONES):  ## Update the target of the drones in case user sent a new target
            vehicle = case.vehicle_list[i]
            vehicle.goal=self.targets[i]

        if self.droneFPVIndex == -1:
            for j in range(self.NB_OF_DRONES):
                self.rotation[j] = self.env.rpy[j, 2]
                # print(f"{obs[str(j)]["state"]}")
                pos = obs[str(j)]["state"][:3]
                case.vehicle_list[j].position = pos

        # calculate time for step_simulation and print average after 500 frames  
        time_before_step_simulation = time.time()

        step_simulation(case)

        time_after_step_simulation = time.time()
        time_taken_pgflow[frameIndex] = (time_after_step_simulation - time_before_step_simulation)
        if frameIndex == 499:
            avg = sum(time_taken_pgflow)/500
            print("Time for step_simulation (pgflow) in ms: ", avg*1000)
            print("FPS max (pgflow): ", 1/avg)
            frameIndex = 0
        else:
            frameIndex+=1

        #### Compute control for the current way point #############
        for j in range(self.env.NUM_DRONES):

            if j == self.droneFPVIndex:  ##### if the drone is controlled in FPV, target velocities and rotations are those received by OSC instead of PGFlow
                desired_vector = np.array([self.velocities[j]['vx'], self.velocities[j]['vy'], self.velocities[j]['vz']])
                self.action[str(j)], _, _ = self.ctrl[j].computeControlFromState(
                    control_timestep=self.CTRL_EVERY_N_STEPS * self.env.TIMESTEP,
                    state=obs[str(j)]["state"],
                    target_pos=np.hstack([obs[str(j)]["state"][:2], obs[str(j)]["state"][2]]),
                    target_vel=desired_vector * FPV_speed * self.actionStrength,
                    # target_acc=np.zeros(3),
                    target_rpy=self.rotation[j] * np.array([0, 0, 1])
                    # target_rpy_rates=np.zeros(3)
                )
                self.velocities[j] = {'vx': 0, 'vy': 0, 'vz': 0}     # reset desired vector after it has been processed by dronesim
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
                    target_vel=desired_vector * vehicle.max_speed,
                    # target_acc=np.zeros(3),
                    target_rpy=[0, 0, np.arctan2(desired_vector[1], desired_vector[0])]
                    # target_rpy_rates=np.zeros(3),
                )

        #### Printout ##############################################
        # self.env.render()

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
