from pythonosc.udp_client import SimpleUDPClient

import argparse
import ast

import osc_protocol
from OSCServer import OSCServer, OSCThread
from swarmcontroller import SwarmController
from PlaceHomogeneousPointsInZone import generate_relaxed_points

# Configuration du parser d'arguments
parser = argparse.ArgumentParser(description='OscSwarmController Configuration')

parser.add_argument('--unity_editor', action="store_true", help='Set this flag if launching the simulation from Unity Editor or the current device')
parser.add_argument('--nb_drones', type=int, default=5, help='Number of drones in the simulation')
parser.add_argument('--gui', action="store_true", help='Set this flag to display the GUI')

args = parser.parse_args()

if not args.unity_editor: # Set the IP of the VR headset
    OSC_CLIENT_IP = "10.1.121.40" # ENAC_AUTH
else:
    OSC_CLIENT_IP = "127.0.0.1"

OSC_SERVER_IP = "0.0.0.0"
OSC_SWARM_CONTROLLER_PORT = 3000
OSC_REMOTE_CONTROLLER_PORT = 3001
OSC_SEND_FREQ = 60  # in Hz
OSC_SEND_RATE = int(1000 / OSC_SEND_FREQ)  # in ms
OSC_SIMULATION_STEPS_TO_SEND = 10  # number of control steps to send data

#### Class that inherits from SwarmController (Handles drones simulation) and adds OSC messages handling ####
class OscSwarmController(SwarmController):

    def __init__(self):

        # create client and server
        self.osc_client = SimpleUDPClient(OSC_CLIENT_IP, OSC_REMOTE_CONTROLLER_PORT)
        self.osc_server = OSCServer(OSC_SERVER_IP, OSC_SWARM_CONTROLLER_PORT)
        self.osc_server.message_received.connect(self.handle_osc_data)
        self.osc_thread = OSCThread(self.osc_server)
        self.osc_thread.start()
        # print("OSC server started")

        # timer to send data from simulation
        self.data_send_timer = QTimer()
        self.data_send_timer.timeout.connect(self.send_simulation_data_via_osc)

        self.nb_of_drones = args.nb_drones
        self.gui = args.gui
        print("gui : ", self.gui)

        # init parent class with simulation and threads for scene update
        super().__init__()

        # start thread for sending osc messages
        self.data_send_timer.start(OSC_SEND_RATE)

        self.send_num_drones_via_osc()

    def send_osc(self, address, args):
        self.osc_client.send_message(address, args)

    # Method to handle incoming OSC data
    def handle_osc_data(self, addr, data):

        if addr == osc_protocol.SET_DRONE_VELOCITIES:
            self.set_drone_velocities(data)

        elif addr == osc_protocol.SET_DRONE_ROTATION:
            self.set_drone_rotation(data)

        elif addr == osc_protocol.SET_DRONE_ROTATION_DELTA :
            self.set_drone_rotation_delta(data)

        elif addr == osc_protocol.LAUNCH_DRONE:
            self.launch_drone()

        elif addr == osc_protocol.SET_DRONE_TARGET:
            self.set_drone_target(data)

        elif addr == osc_protocol.SET_DRONE_TARGET_HEIGHT:
            self.modify_target_height(data)
        
        elif addr == osc_protocol.SET_FLEET_TARGET:
            self.set_fleet_target(data)

        elif addr == osc_protocol.SET_TARGET_MODE:
            self.set_target_mode(data)

        elif addr == osc_protocol.SET_DRONE_TRAJECTORY:
            self.set_drone_trajectory(data)

        elif addr == osc_protocol.RESET_TARGETS:
            self.reset_targets(data)

        elif addr == osc_protocol.SET_ZONE:
            self.set_zone(data)

        elif addr == osc_protocol.SET_PAUSE_DRONE :
            self.set_pause_drone(data)
        
        elif addr == osc_protocol.SET_PLAY_DRONE :
            self.set_play_drone(data)

        elif addr == osc_protocol.EXIT_FPV_MODE:  # command sent when the user quits FPV on the current selected drone
            self.rotation_delta[self.drone_fpv_index] = 0
            self.drone_fpv_index = -1
            

        elif addr == osc_protocol.DEBUG_MESSAGE:
            print("Debug message: ", data)

    # function that updates velocities with the values sent by the user, see osc_protocol.py to know what it should take as arguments
    def set_drone_velocities(self, data_string):
        data = self.to_array(data_string)
        id_drone = int(data[0])
        if id_drone != -1:
            self.velocities[id_drone]['vx'] = float(data[1])
            self.velocities[id_drone]['vz'] = float(data[2])
            self.velocities[id_drone]['vy'] = float(data[3])
            self.drone_fpv_index = id_drone
            # print("Drone ", id_drone, " velocities set to: ", self.velocities[id_drone])
        self.action_strength = float(data[4])

    def set_drone_rotation(self, data_string):
        data = self.to_array(data_string)
        drone_id = int(data[0])
        direction = float(data[1])
        rotationStrength = float(data[2])
        if direction == 1:
            self.rotation[drone_id] -= 0.02 * rotationStrength
        elif direction == -1:
            self.rotation[drone_id] += 0.02 * rotationStrength

    def set_drone_rotation_delta(self, data_string):
        data = self.to_array(data_string)
        drone_id = int(data[0])
        direction = float(data[1])
        rotationStrength = float(data[2])
        if direction == 1:
            self.rotation_delta[drone_id] = self.rotation_delta[drone_id] - rotationStrength * 0.02
        elif direction == -1:
            self.rotation_delta[drone_id] = self.rotation_delta[drone_id] + rotationStrength * 0.02
        # print("Drone ", drone_id, " rotation delta set to: ", self.rotation_delta[drone_id])

    def launch_drone(self):
        print("Take off drone")
        self.set_drone_state_to_launch()


    def set_fleet_target(self, data_string):
        data = self.to_array(data_string)
        xtarget = float(data[0])
        ztarget = float(data[1])
        ytarget = float(data[2])
        for i in range (self.nb_of_drones):    
            self.fleet_target = [xtarget, ytarget, ztarget]
            if self.target_mode == 0: self.vehicle_list[i].state=0
        print("new target for fleet: ", xtarget, ytarget, ztarget)

    def set_drone_target(self, data_string):
        data = self.to_array(data_string)
        drone_id = int(data[0])
        xtarget = float(data[1])
        ztarget = float(data[2])
        ytarget = float(data[3])
        if self.drone_targets[drone_id][2] != 0 :
            # print("target already set")
            self.drone_targets[drone_id] = [float(xtarget), float(ytarget), self.drone_targets[drone_id][2]]
        else:  
            print("no target already set")
            self.drone_targets[drone_id] = [float(xtarget), float(ytarget), float(ztarget)]
        self.rotation_delta[drone_id] = 0
        # Reset trajectory if target is set
        if self.trajectory_drone[drone_id] != -1 :
            self.trajectory_drone[drone_id] = -1
        if self.target_mode == 1: 
            self.vehicle_list[drone_id].state=0
        print("new target for drone", drone_id, ": ", self.drone_targets[drone_id])

    def set_drone_trajectory(self, data_string):
        data = self.to_array(data_string)
        drone_id = int(data[0])
        self.is_first_traj_point_reached[drone_id] = False
        trajectory_str = data[1]
        self.rotation_delta[drone_id] = 0
        
        try:
            trajectory_str = trajectory_str.replace(';', ',')
            trajectory_str = trajectory_str.replace('\'', '')
            trajectory = ast.literal_eval(trajectory_str)
        except Exception as e:
            print("Error parsing string:", e)
            trajectory = []
            return

        print("new trajectory for drone", drone_id, ": ", trajectory)
        if self.drone_targets[drone_id][2] != 0 :
            self.trajectory_drone[drone_id] = []
            for i in range(len(trajectory)):
                self.trajectory_drone[drone_id].append([float(trajectory[i][0]), float(trajectory[i][1]), self.drone_targets[drone_id][2]])
        else:
            self.trajectory_drone[drone_id] = trajectory

        # print("new trajectory for drone after modifying height", drone_id, ": ", self.trajectory_drone[drone_id])
        # Dernier point trajectoire = target + current height of drone
        last_point = self.trajectory_drone[drone_id][-1]
        self.drone_targets[drone_id] = [last_point[0], last_point[1], last_point[2]]
        if self.target_mode == 1: 
            self.vehicle_list[drone_id].state=0
    
    def modify_target_height(self, data_string):
        data = self.to_array(data_string)
        drone_id = int(data[0])
        if (self.is_drone_on_height_target(drone_id)) :
            height = float(data[1])
            current_height = self.drone_targets[drone_id][2]
            self.drone_targets[drone_id][2] = current_height + height
            if self.trajectory_drone[drone_id] != -1 :
                for i in range(len(self.trajectory_drone[drone_id])):
                    self.trajectory_drone[drone_id][i][2] += height
            # print("new target height for drone", drone_id, ": ", self.drone_targets[drone_id][2])


    def set_target_mode(self, data):
        mode = int(data[0])
        self.target_mode = mode
        for i in range(self.nb_of_drones):
            if (mode == 0 and self.is_fleet_target_set()) or (mode == 1 and self.is_individual_target_set(i)):
                self.vehicle_list[i].state=0
        print("new target mode: ", mode)

    def reset_targets(self, data):
        for i in range(self.nb_of_drones):
            self.drone_targets[i] = self.initial_drone_targets[i]
        self.fleet_target = self.initial_fleet_target
        print("targets reset")
    
    def set_pause_drone(self, data_string) :
        data = self.to_array(data_string)
        id_drone = int(data[0])
        self.pause_state[id_drone] = 1

    def set_play_drone(self, data_string) :
        data = self.to_array(data_string)
        id_drone = int(data[0])
        self.pause_state[id_drone] = 0

    def to_array(self, data):
        data = data[1:-1]
        return data.split(',')

    # Method to start the OSC server
    def start_osc_server(self):
        self.osc_server.start_server()

    # Method to stop the OSC server
    def stop_osc_server(self):
        self.osc_server.stop_server()

    def send_simulation_data_via_osc(self):
        for i in range(self.env.NUM_DRONES):
            self.send_osc(osc_protocol.SEND_DRONE_DATA, [i, self.env.pos[i, 0], self.env.pos[i, 1], self.env.pos[i, 2],
                                                         self.env.rpy[i, 0],
                                                         self.env.rpy[i, 1],
                                                         self.env.rpy[i, 2]])
            
    def send_num_drones_via_osc(self):
        self.send_osc(osc_protocol.SEND_NUM_DRONES, [self.nb_of_drones])

    def send_drone_end_trajectory(self, id):
        self.send_osc(osc_protocol.DRONE_END_TRAJECTORY, [id])

    def send_drone_reached_first_point_trajectory(self, id):
        print("drone on first point of its trajectory")
        self.send_osc(osc_protocol.DRONE_REACHED_FIRST_POINT_TRAJ, [id])

    def stop_simulation(self):
        self.simulation_timer.stop()
        self.data_send_timer.stop()

    def set_zone(self, data_string):
        print(data_string)
        relaxed_points = generate_relaxed_points(data_string, num_points=self.nb_of_drones)
        print(f"{relaxed_points}")
        self.send_osc(osc_protocol.SET_TARGETS_IN_ZONE, f"{relaxed_points}")


if __name__ == "__main__":
    from PyQt6.QtWidgets import QApplication
    from PyQt6.QtCore import QTimer
    import sys

    app = QApplication(sys.argv)
    controller = OscSwarmController()
    sys.exit(app.exec())
