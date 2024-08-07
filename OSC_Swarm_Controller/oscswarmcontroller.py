from pythonosc.udp_client import SimpleUDPClient

import osc_protocol
from OSCServer import OSCServer, OSCThread
from swarmcontroller import SwarmController

VR_HEADSET = True # set to True if using VR headset, False if using desktop version
if VR_HEADSET:
    # OSC_CLIENT_IP = "10.1.124.77" # ENAC_AUTH
    # OSC_CLIENT_IP = "192.168.1.102" # LII_AP_NO_INTERNET
    OSC_CLIENT_IP = "192.168.43.76" # Huawei P Smart Z
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

        # init parent class with simulation and threads for scene update
        super().__init__()

        # start thread for sending osc messages
        self.data_send_timer.start(OSC_SEND_RATE)

        self.send_num_drones_via_osc()

    def send_osc(self, address, args):
        self.osc_client.send_message(address, args)
        # print(f"Sent OSC message: {address} {args}")

    # Method to handle incoming OSC data
    def handle_osc_data(self, addr, data):

        if addr == osc_protocol.SET_DRONE_VELOCITIES:
            self.set_drone_velocities(data)

        elif addr == osc_protocol.SET_DRONE_ROTATION:
            self.set_drone_rotation(data)

        if addr == osc_protocol.SET_DRONE_TARGET:
            self.set_drone_target(data)
        
        elif addr == osc_protocol.SET_FLEET_TARGET:
            self.set_fleet_target(data)

        elif addr == osc_protocol.SET_TARGET_MODE:
            self.set_target_mode(data)

        elif addr == osc_protocol.RESET_TARGETS:
            self.reset_targets(data)

        elif addr == osc_protocol.EXIT_FPV_MODE:  # command sent when the user quits FPV on the current selected drone
            self.droneFPVIndex = -1

    # function that updates velocities with the values sent by the user, see osc_protocol.py to know what it should take as arguments
    def set_drone_velocities(self, data_string):
        data = self.to_array(data_string)
        id_drone = int(data[0])
        if id_drone != -1:
            self.velocities[id_drone]['vx'] = float(data[1])
            self.velocities[id_drone]['vz'] = float(data[2])
            self.velocities[id_drone]['vy'] = float(data[3])
            self.droneFPVIndex = id_drone
        self.actionStrength = float(data[4])

    def set_drone_rotation(self, data_string):
        data = self.to_array(data_string)
        drone_id = int(data[0])
        direction = float(data[1])
        rotationStrength = float(data[2])
        if direction == 1:
            self.rotation[drone_id] -= 0.03 * rotationStrength
        elif direction == -1:
            self.rotation[drone_id] += 0.03 * rotationStrength

    def set_fleet_target(self, data_string):
        data = self.to_array(data_string)
        xtarget = float(data[0])
        ztarget = float(data[1])
        ytarget = float(data[2])
        for i in range (self.NB_OF_DRONES):
            self.old_fleet_target = self.fleet_target    
            self.fleet_target = [xtarget, ytarget, ztarget]
            if self.target_mode == 0: self.vehicle_list[i].state=0
        print("new target for fleet: ", xtarget, ytarget, ztarget)

    def set_drone_target(self, data_string):
        data = self.to_array(data_string)
        drone_id = int(data[0])
        xtarget = float(data[1])
        ztarget = float(data[2])
        ytarget = float(data[3])
        self.drone_targets[drone_id] = [xtarget, ytarget, ztarget]
        if self.target_mode == 1: self.vehicle_list[drone_id].state=0
        print("new target for drone", drone_id, ": ", xtarget, ytarget, ztarget)

    def set_target_mode(self, data):
        mode = int(data[0])
        self.target_mode = mode
        for i in range(self.NB_OF_DRONES):
            if (mode == 0 and self.is_fleet_target_set()) or (mode == 1 and self.is_individual_target_set(i)):
                self.vehicle_list[i].state=0
        print("new target mode: ", mode)

    def reset_targets(self, data):
        for i in range(self.NB_OF_DRONES):
            self.drone_targets[i] = self.initial_drone_targets[i]
        self.fleet_target = self.initial_fleet_target
        print("targets reset")

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
        self.send_osc(osc_protocol.SEND_NUM_DRONES, [self.NB_OF_DRONES])

    def stop_simulation(self):
        self.simulation_timer.stop()
        self.data_send_timer.stop()

if __name__ == "__main__":
    from PyQt6.QtWidgets import QApplication
    from PyQt6.QtCore import QTimer
    import sys

    app = QApplication(sys.argv)
    controller = OscSwarmController()
    sys.exit(app.exec())
