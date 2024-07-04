import sys
import threading

from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QSlider
from PyQt6.QtCore import Qt, QObject, pyqtSignal
from pythonosc.udp_client import SimpleUDPClient
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import AsyncIOOSCUDPServer, ThreadingOSCUDPServer



# Adresse IP et port du serveur OSC
OSC_IP = "127.0.0.1"
OSC_SWARM_CONTROLLER_PORT = 3000
OSC_REMOTE_CONTROLLER_PORT = 3001

class DroneController(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Contrôleur de Drone")
        self.initUI()
        self.osc_client = SimpleUDPClient(OSC_IP, OSC_SWARM_CONTROLLER_PORT)

       #osc receiver stuff
        dp = Dispatcher()
        dp.map(
            "/*",
            self.handle_osc_message,
        )
        self.osc_server = ThreadingOSCUDPServer((OSC_IP, OSC_REMOTE_CONTROLLER_PORT), dp)

    def send_osc(self, address, value):
        self.osc_client.send_message(address, value)

    def start_osc_server(self):
        threading.Thread(target=self.osc_server.serve_forever, daemon=True).start()

    def handle_osc_message(self, addr, *data):
        print("OSC message ", addr, *data)

    def closeEvent(self, event):
        if self.osc_server:
            self.osc_server.shutdown()
        event.accept()

    def initUI(self):
        layout = QVBoxLayout()

        # Boutons
        btn_decollage = QPushButton("Décollage")
        btn_decollage.clicked.connect(lambda: self.send_osc("/take_off", 1))
        btn_atterrissage = QPushButton("Atterrissage")
        btn_atterrissage.clicked.connect(lambda: self.send_osc("/landing", 1))

        layout.addWidget(btn_decollage)
        layout.addWidget(btn_atterrissage)

        for label_text in ["/vx", "/vy", "/vz"]:
            label = QLabel(label_text)
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setMinimum(-2 * 100)
            slider.setMaximum(2 * 100)
            slider.setValue(0)
            #slider.setTickInterval(1)
            #slider.setTickPosition(QSlider.TickPosition.TicksBelow)

            # Label pour afficher la valeur courante du slider
            slider_label = QLabel("0")
            layout.addWidget(label)
            layout.addWidget(slider)
            layout.addWidget(slider_label)

            # Mettre à jour le label lorsque la valeur du slider change
            slider.valueChanged.connect(lambda value, l=slider_label: l.setText(str(value / 100)))
            slider.valueChanged.connect(lambda value, label=label_text: self.send_osc(label, value / 100))

        self.setLayout(layout)
        self.show()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DroneController()
    window.start_osc_server()
    sys.exit(app.exec())