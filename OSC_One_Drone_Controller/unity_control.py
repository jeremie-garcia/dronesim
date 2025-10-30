import sys
import os

# Comment --> for debuging Amania Computer
current_dir = os.path.dirname(os.path.abspath(__file__))
grandparent_dir = os.path.dirname(os.path.dirname(current_dir))
dronesim_path = os.path.join(grandparent_dir, "dronesim")
sys.path.insert(0, dronesim_path)

import pybullet as p
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt6.QtCore import QObject, QTimer

import argparse

from pythonosc.udp_client import SimpleUDPClient

from dronesim.control.INDIControl import INDIControl
from dronesim.envs.BaseAviary import DroneModel, Physics

# from dronesim.envs.CtrlAviary import CtrlAviary
from dronesim.envs.VelocityAviary import VelocityAviary
from dronesim.utils.Logger import Logger
from dronesim.utils.trajGen import *
from dronesim.utils.utils import str2bool, sync

from OSCServer import OSCServer, OSCThread

OSC_IP = "127.0.0.1"
OSC_SWARM_CONTROLLER_PORT = 3000
OSC_REMOTE_CONTROLLER_PORT = 3001
CONTROL_FREQ = 60  # in HZ
CONTROL_RATE = int(1000 / CONTROL_FREQ)  # in ms

OSC_SEND_FREQ = 20  # in Hz
OSC_SEND_RATE = int(1000 / OSC_SEND_FREQ)  # in ms

OSC_SIMULATION_STEPS_TO_SEND = 10  # number of control steps to send data
NB_OF_DRONES = 1


class OSC_Swarm_Controller(QObject):
    def __init__(self):
        super().__init__()

        self.ARGS = None
        self.osc_client = SimpleUDPClient(OSC_IP, OSC_REMOTE_CONTROLLER_PORT)

        self.osc_server = OSCServer(OSC_IP, OSC_SWARM_CONTROLLER_PORT)
        self.osc_server.message_received.connect(self.handle_osc_data)

        self.osc_thread = OSCThread(self.osc_server)
        self.osc_thread.start()

        print("OSC server started")
        
        self.env = None
        self.NB_OF_DRONES = NB_OF_DRONES

        self.neighbourhood_radius = 4
        self.formation_2D = False


        self.create_flyingsim()

        self.velocities = {i: {'vx': 0.0, 'vy': 0.0, 'vz': 0.0} for i in range(self.NB_OF_DRONES)}

        self.action = {str(i): np.array([float(self.velocities[i]['vx']), float(self.velocities[i]['vy']), float(self.velocities[i]['vz']), 0.5]) for i in range(self.NB_OF_DRONES)}

        self.simulation_timer = QTimer()
        self.simulation_timer.timeout.connect(self.update_simulation)

        self.data_send_timer = QTimer()
        self.data_send_timer.timeout.connect(self.send_simulation_data_via_osc)

        self.start_simulation()
       #print("simulation started")

    def send_osc(self, address, args):
        self.osc_client.send_message(address, args)

    # Method to handle incoming OSC data
    def handle_osc_data(self, addr, data):
        parts = addr.split('/')
        
        if len(parts) == 3 and parts[1] == 'drone' and parts[2] in ['vx', 'vy', 'vz']:
            axis = parts[2]
            for id_drone in range(self.NB_OF_DRONES):
                self.velocities[id_drone][axis] = data
                
        elif addr == "/drone/disperse" :
            self.neighbourhood_radius += 0.5

        elif addr == "/drone/gather":
            if self.neighbourhood_radius >= 2:
                self.neighbourhood_radius -= 0.5

        elif addr == "/drone/take_off":
            for i in range(self.NB_OF_DRONES):
                self.velocities[i]['vz'] = 1
                self.velocities[i]['vx'] = 0
                self.velocities[i]['vy'] = 0

            def stop_up():
                for i in range(self.NB_OF_DRONES):
                    self.velocities[i]['vz'] = 0
                self.action = {str(i): np.array([float(self.velocities[i]['vx']), float(self.velocities[i]['vy']), float(self.velocities[i]['vz']), 0.5]) for i in range(self.NB_OF_DRONES)}

            QTimer.singleShot(500, stop_up)

        elif addr == "/drone/landing":

            for i in range(self.NB_OF_DRONES):
                self.velocities[i]['vz'] = -1
                self.velocities[i]['vx'] = 0
                self.velocities[i]['vy'] = 0

        # update the action to perform
        self.action = {str(i): np.array([float(self.velocities[i]['vx']), float(self.velocities[i]['vy']), float(self.velocities[i]['vz']), 0.5]) for i in range(self.NB_OF_DRONES)}


    # Method to start the OSC server
    def start_osc_server(self):
        self.osc_server.start_server()

    # Method to stop the OSC server
    def stop_osc_server(self):
        self.osc_server.stop_server()

    def create_flyingsim(self):
        #### Define and parse (optional) arguments for the script ##
        parser = argparse.ArgumentParser(
            description="Helix flight script using CtrlAviary or VisionAviary and DSLPIDControl"
        )
        parser.add_argument(
            "--drone",
            default=["tello"],
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
            default=False,
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
        parser.add_argument (
            "--neighbourhood_radius",
            default = 2,
            type = float,
            help = "Neighbourhood radius for the drones (default: 2)"
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

        if ARGS.neighbourhood_radius:
            self.neighbourhood_radius = ARGS.neighbourhood_radius
        
        if ARGS.formation_2D:
            self.formation_2D = ARGS.formation_2D

        #### Initialize the simulation #############################
        H = 0.50
        H_STEP = 0.05
        R = 1.5

        AGGR_PHY_STEPS = (
            int(ARGS.simulation_freq_hz / ARGS.control_freq_hz) if ARGS.aggregate else 1
        )
        INIT_XYZS = np.array(
            [
                [
                    i % 5,
                    int(i / 5),
                    0,
                ]
                for i in range(ARGS.num_drones)
            ]
        )
        INIT_RPYS = np.array([[0.0, 0.0, 0.0] for i in range(ARGS.num_drones)])

        #### Create the environment
        self.env = VelocityAviary(
            drone_model=ARGS.num_drones * ARGS.drone,
            num_drones=ARGS.num_drones,
            initial_xyzs=INIT_XYZS,
            initial_rpys=INIT_RPYS,
            physics=Physics.PYB,
            neighbourhood_radius=self.neighbourhood_radius ,
            freq=ARGS.simulation_freq_hz,
            aggregate_phy_steps=AGGR_PHY_STEPS,
            gui=ARGS.gui,
            record=ARGS.record_video,
            obstacles=ARGS.obstacles,
            user_debug_gui=ARGS.user_debug_gui,
        )

        #### Obtain the PyBullet Client ID from the environment ####
        self.ARGS = ARGS

    def update_simulation(self):
        #### Step the simulation ###################################
        obs, reward, done, info = self.env.step(self.action)
        self.avoid_collisions_potential_fields(obs)

    def send_simulation_data_via_osc(self):
        for i in range(self.env.NUM_DRONES):
            self.send_osc("/drone", [i, self.env.pos[i, 0], self.env.pos[i, 1], self.env.pos[i, 2],
                                     self.env.rpy[i, 0],
                                     self.env.rpy[i, 1],
                                     self.env.rpy[i, 2]])
            # print("drone", i, self.env.pos[i, 0], self.env.pos[i, 1], self.env.pos[i, 2],   self.env.rpy[i, 0],
            #                          self.env.rpy[i, 1],
            #                          self.env.rpy[i, 2])

    def start_simulation(self):
        self.simulation_timer.start(CONTROL_RATE)
        self.data_send_timer.start(OSC_SEND_RATE)

    def stop_simulation(self):
        self.simulation_timer.stop()
        self.data_send_timer.stop()

    def close_env(self):
        ### Close the environment ###
        self.env.close()

    # EOF

    def avoid_collisions_potential_fields(self, obs):
        #reinitialize the velocities
        self.velocities = {i: {'vx': 0.0, 'vy': 0.0, 'vz': 0.0} for i in range(self.NB_OF_DRONES)}

        """Adjust the velocities of the drones using potential fields to avoid collisions."""
        k_att = 0.1  # Attraction constant
        k_rep = 0.8  # Repulsion constant
        repulsion_radius = self.neighbourhood_radius

        for i in range(self.NB_OF_DRONES):
            if self.env.pos[i, 2] < 0.5:
                continue
            # Initialize forces
            force_att = np.zeros(3)
            force_rep = np.zeros(3)


            #the goal position is the center of the swarm (average of all the drones)
            goal_position = np.array(np.mean(self.env.pos, axis=0))

            current_position = np.array([self.env.pos[i, 0], self.env.pos[i, 1], self.env.pos[i, 2]])
            force_att = k_att * (goal_position - current_position)

            for j in range(self.NB_OF_DRONES):
                if i != j:
                    pos_i = current_position
                    pos_j = np.array([self.env.pos[j, 0], self.env.pos[j, 1], self.env.pos[j, 2]])

                    # Calculate the distance between the drones
                    distance = np.linalg.norm(pos_i - pos_j)
                    if distance < repulsion_radius:
                        # Calculate the repulsive force
                        direction = pos_i - pos_j
                        direction = direction / np.linalg.norm(direction)  # Normalize the direction vector
                        force_rep += k_rep / (distance ** 2) * direction

                    elif distance < repulsion_radius + 0.5 :
                        force_att = 0

            # Calculate the total force
            total_force = force_att + force_rep

            # Adjust the velocities based on the forces
            self.velocities[i]['vx'] += total_force[0]
            self.velocities[i]['vy'] += total_force[1]
            if not self.formation_2D:
                self.velocities[i]['vz'] += total_force[2]

        self.action = {str(i): np.array([float(self.velocities[i]['vx']), float(self.velocities[i]['vy']), float(self.velocities[i]['vz']), 0.2]) for i in range(self.NB_OF_DRONES)}
        return


if __name__ == "__main__":
    app = QApplication(sys.argv)
    controller = OSC_Swarm_Controller()
    sys.exit(app.exec())
    controller.close_env()
