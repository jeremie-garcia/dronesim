import sys
import threading
from PyQt6.QtCore import QObject, pyqtSignal, QThread, Qt
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from pythonosc import dispatcher, osc_server

class OSCServer(QObject):
    message_received = pyqtSignal(str, str)

    def __init__(self, address, port):
        super().__init__()
        self.address = address
        self.port = port
        self.dispatcher = dispatcher.Dispatcher()
        self.dispatcher.map("/*", self.handle_message)
        self.server = None

    def start_server(self):
        self.server = osc_server.ThreadingOSCUDPServer((self.address, self.port), self.dispatcher)
        self.server.serve_forever()

    def stop_server(self):
        if self.server:
            self.server.shutdown()

    def handle_message(self, address, *args):
        #print("received message from server", address, *args)
        self.message_received.emit(address, str(*args))

class OSCThread(QThread):
    def __init__(self, server):
        super().__init__()
        self.server = server

    def run(self):
        self.server.start_server()
