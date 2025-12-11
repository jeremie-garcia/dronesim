# DroneSim

DroneSim is a Python-based simulation environment designed to run **in parallel with a Unity project** through OSC messaging.  
Its main purpose is to receive control commands from Unity (targets, trajectories, velocity commands) and to return each drone’s position and rotation back to Unity in real time.

The project also includes a minimal GUI example folder used only to verify that the Python installation works correctly, although these examples are **not required** when using DroneSim with Unity.

---

## Installation

### 1. Clone the repository

    git clone <repository_url>
    cd dronesim

### 2. Install Python dependencies

All required packages are listed in `requirements.txt`.

    pip install -r requirements.txt

---

## Using DroneSim with Unity (OSC Communication)

The main component of this project is the OSC-based controller:

`OSC_Swarm_Controller/oscswarmcontroller.py`

This script acts as the communication layer between Unity and DroneSim.  
It receives OSC commands from Unity and sends back simulation data (positions and rotations) at each update step.

### Launching the OSC controller

    python OSC_Swarm_Controller/oscswarmcontroller.py

### OSC Swarm Controller Arguments


- `--unity_editor` : When this flag is provided, the simulator switches to **Unity Editor mode**. In this mode, OSC communication is configured to use the local machine, setting `OSC_CLIENT_IP = "127.0.0.1"`. This ensures that all OSC messages are exchanged locally between Unity and the Python simulation.

- `--nb_drones <int>` : Defines the **number of drones** to simulate. If not specified, the simulator uses a default value of **5** drones.

- `--gui` : Opens the **PyBullet visualization window**. This option is useful for debugging or standalone Python usage. When working with Unity, this flag is generally not used because Unity handles all rendering.


#### Examples:

    python oscswarmcontroller.py --unity_editor --nb_drones 5


---

## OSC Commands and Protocol

All OSC messages exchanged between Unity and the Python simulator are defined in:

`OSC_Swarm_Controller/osc_protocol.py`

This protocol contains the message structure used by the project, including:

- **Target commands** (single target for one drone)  
- **Trajectory commands** (lists of targets for path following for one drone)  
- **Velocity commands** (vx, vy, vz) for one drone
- **Rotation commands** (yaw rate) for one drone
- **Zone Initial Launch Points** list of (x, y) coordinates
- **Any additional control message shared with Unity**

The Python module receives these commands, updates the drones accordingly, and sends back:

- **Drone world positions**  
- **Drone rotations (Euler angles)**

All outgoing signals are sent as OSC messages following the same protocol file.

---

## GUI Example (optional)

The folder `simple_GUI_exemples/` contains lightweight scripts intended only to confirm that your Python setup works:

- `DroneControllerGUI.py`  
- `simple_swarm_controller.py`  

These examples run a small local simulation with velocity control and may use the PyBullet `--gui` visualization.  
They are **not needed** when DroneSim is used as a backend for Unity.

---

## Project Structure

    dronesim/
    │
    ├── OSC_Swarm_Controller/
    │   ├── oscswarmcontroller.py
    │   ├── osc_protocol.py
    │   └── ...
    │
    ├── simple_GUI_exemples/      # Optional test tools (not used with Unity)
    │   ├── DroneControllerGUI.py
    │   ├── simple_swarm_controller.py
    │
    ├── dronesim/
    │   └── ... (core simulation engine)
    │
    ├── requirements.txt
    └── README.md