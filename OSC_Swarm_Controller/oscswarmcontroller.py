from pythonosc.udp_client import SimpleUDPClient

import osc_protocol
from OSCServer import OSCServer, OSCThread
from swarmcontroller import SwarmController

VR_HEADSET = False
if VR_HEADSET:
    OSC_CLIENT_IP = "10.1.124.77"
else:
    OSC_CLIENT_IP = "127.0.0.1"

OSC_SERVER_IP = "0.0.0.0"
OSC_SWARM_CONTROLLER_PORT = 3000
OSC_REMOTE_CONTROLLER_PORT = 3001
OSC_SEND_FREQ = 180  # in Hz
OSC_SEND_RATE = int(1000 / OSC_SEND_FREQ)  # in ms
OSC_SIMULATION_STEPS_TO_SEND = 10  # number of control steps to send data


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

    def send_osc(self, address, args):
        self.osc_client.send_message(address, args)
        # print(f"Sent OSC message: {address} {args}")

    # Method to handle incoming OSC data
    def handle_osc_data(self, addr, data):
        parts = addr.split('/')
        print(f"addr: {addr}, parts: {parts}, data: {data}")

        if len(parts) == 4 and parts[1] == 'drone' and parts[2] in ['vx', 'vy', 'vz']:
            axis = parts[2]
            id_drone = int(parts[3])
            if id_drone != -1:
                self.velocities[id_drone][axis] = float(data)
                self.droneFPVIndex = id_drone

        elif len(parts) == 4 and parts[2] == 'target':
            coordinates = parts[3].split(',')
            xtarget = float(coordinates[0])
            ztarget = float(coordinates[1])
            ytarget = float(coordinates[2])
            self.target = [xtarget, ytarget, ztarget]
            print("new target: ", xtarget, ytarget, ztarget)

        elif len(parts) == 4 and parts[2] == 'rotate':
            id_drone = int(parts[3])
            if id_drone != -1:
                if float(data) == 1:
                    self.rotation[id_drone] -= 0.03
                elif float(data) == -1:
                    self.rotation[id_drone] += 0.03

        elif addr == osc_protocol.FORWARD:  # command meant to indicate that the drone is moving forward so that it
            # accelerates
            if self.forwardFrameCounter < 140:
                self.forwardFrameCounter += 1
            self.stopFrameCounter = 0

        elif addr == osc_protocol.RESET:  # command sent when the user quits FPV on the current selected drone
            self.droneFPVIndex = -1

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