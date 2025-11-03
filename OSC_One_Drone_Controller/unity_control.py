import sys
import os

# Comment --> for debugging Amania Computer
current_dir = os.path.dirname(os.path.abspath(__file__))
grandparent_dir = os.path.dirname(os.path.dirname(current_dir))
dronesim_path = os.path.join(grandparent_dir, "dronesim")
sys.path.insert(0, dronesim_path)

import pybullet as p
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt6.QtCore import QObject, QTimer

import argparse
import ast

from pythonosc.udp_client import SimpleUDPClient

from dronesim.control.INDIControl import INDIControl
from dronesim.envs.BaseAviary import DroneModel, Physics
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

OSC_SIMULATION_STEPS_TO_SEND = 10
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

        # --- Only one drone ---
        self.velocities = {'vx': 0.0, 'vy': 0.0, 'vz': 0.0}
        self.action = {'0': np.array([0.0, 0.0, 0.0, 0.5])}

        self.simulation_timer = QTimer()
        self.simulation_timer.timeout.connect(self.update_simulation)

        self.data_send_timer = QTimer()
        self.data_send_timer.timeout.connect(self.send_simulation_data_via_osc)

        self.start_simulation()

    def send_osc(self, address, args):
        self.osc_client.send_message(address, args)

    # Method to handle incoming OSC data
    def handle_osc_data(self, addr, data):
        if addr == "/set_drone_velocities":
            data_array = ast.literal_eval(data)
            self.velocities['vx'] = float(data_array[0])
            self.velocities['vy'] = float(data_array[2])
            self.velocities['vz'] = float(data_array[1])  # Unity coordinates y<->z

        elif addr == "/drone/take_off":
            self.velocities['vz'] = 1
            self.velocities['vx'] = 0
            self.velocities['vy'] = 0

            def stop_up():
                self.velocities['vz'] = 0
                self.action = {'0': np.array(
                    [self.velocities['vx'], self.velocities['vy'], self.velocities['vz'], 0.5])}

            QTimer.singleShot(500, stop_up)

        elif addr == "/drone/landing":
            self.velocities['vz'] = -1
            self.velocities['vx'] = 0
            self.velocities['vy'] = 0

        # Update the action
        self.action = {'0': np.array(
            [self.velocities['vx'], self.velocities['vy'], self.velocities['vz'], 0.5])}

    # Method to start the OSC server
    def start_osc_server(self):
        self.osc_server.start_server()

    def stop_osc_server(self):
        self.osc_server.stop_server()

    def create_flyingsim(self):
        parser = argparse.ArgumentParser(
            description="Single drone flight script using VelocityAviary"
        )
        parser.add_argument("--drone", default=["tello"], type=list)
        parser.add_argument("--num_drones", default=1, type=int)
        parser.add_argument("--physics", default="pyb", type=Physics)
        parser.add_argument("--vision", default=False, type=str2bool)
        parser.add_argument("--gui", default=False, type=str2bool)
        parser.add_argument("--record_video", default=False, type=str2bool)
        parser.add_argument("--plot", default=False, type=str2bool)
        parser.add_argument("--user_debug_gui", default=False, type=str2bool)
        parser.add_argument("--aggregate", default=True, type=str2bool)
        parser.add_argument("--obstacles", default=False, type=str2bool)
        parser.add_argument("--simulation_freq_hz", default=240, type=int)
        parser.add_argument("--control_freq_hz", default=CONTROL_FREQ, type=int)
        parser.add_argument("--duration_sec", default=20000, type=int)
        parser.add_argument("--neighbourhood_radius", default=2, type=float)
        parser.add_argument("--formation_2D", default=False, type=str2bool)

        ARGS = parser.parse_args()

        #### Initialize the simulation for one drone ####
        INIT_XYZS = np.array([[0, 0, 3]])
        INIT_RPYS = np.array([[0.0, 0.0, 0.0]])

        AGGR_PHY_STEPS = int(ARGS.simulation_freq_hz / ARGS.control_freq_hz) if ARGS.aggregate else 1

        self.env = VelocityAviary(
            drone_model=ARGS.num_drones * ARGS.drone,
            num_drones=1,
            initial_xyzs=INIT_XYZS,
            initial_rpys=INIT_RPYS,
            physics=Physics.PYB,
            neighbourhood_radius=ARGS.neighbourhood_radius,
            freq=ARGS.simulation_freq_hz,
            aggregate_phy_steps=AGGR_PHY_STEPS,
            gui=ARGS.gui,
            record=ARGS.record_video,
            obstacles=ARGS.obstacles,
            user_debug_gui=ARGS.user_debug_gui,
        )

        self.ARGS = ARGS

    def update_simulation(self):
        obs, reward, done, info = self.env.step(self.action)
        self.avoid_collisions_potential_fields(obs)

    def send_simulation_data_via_osc(self):
        # --- One drone only ---
        self.send_osc("/send_drone_data", [
            self.env.pos[0, 0],
            self.env.pos[0, 2],
            self.env.pos[0, 1],
            self.env.rpy[0, 0],
            self.env.rpy[0, 2],
            self.env.rpy[0, 1],
        ])

    def start_simulation(self):
        self.simulation_timer.start(CONTROL_RATE)
        self.data_send_timer.start(OSC_SEND_RATE)

    def stop_simulation(self):
        self.simulation_timer.stop()
        self.data_send_timer.stop()

    def close_env(self):
        self.env.close()

    def avoid_collisions_potential_fields(self, obs):
        """Simplified version for one drone: does nothing."""
        self.velocities = {'vx': self.velocities['vx'],
                           'vy': self.velocities['vy'],
                           'vz': self.velocities['vz']}
        self.action = {'0': np.array(
            [self.velocities['vx'], self.velocities['vy'], self.velocities['vz'], 0.2])}
        return


if __name__ == "__main__":
    app = QApplication(sys.argv)
    controller = OSC_Swarm_Controller()
    sys.exit(app.exec())
    controller.close_env()