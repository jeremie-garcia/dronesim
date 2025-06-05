import os
import sys
import time
import json
import argparse
import numpy as np
from math import pi

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QObject, QTimer

import pybullet as p

# Ajuster le chemin pour inclure le package 'dronesim'
current_dir = os.path.dirname(os.path.abspath(__file__))
grandparent_dir = os.path.dirname(os.path.dirname(current_dir))
dronesim_path = os.path.join(grandparent_dir, "dronesim")
sys.path.insert(0, dronesim_path)

from dronesim.control.INDIControl import INDIControl
from dronesim.envs.BaseAviary import DroneModel, Physics
from dronesim.envs.CtrlAviary import CtrlAviary
from dronesim.utils.trajGen import *  # Considérez l'importation explicite des fonctions nécessaires
from dronesim.utils.utils import str2bool

from pgflow import Cases, vehicle
from pgflow.arena import ArenaMap
from pgflow.utils.simulation_utils import step_simulation, set_new_attribute

CONTROL_FREQ = 30  # en Hz
CONTROL_RATE = int(1000 / CONTROL_FREQ)  # en ms
SIMULATION_FREQ_HZ = 240

"""Initialisation de la simulation de l'essaim de drones."""
FILENAME = 'VerticesData.json'  # Nom du cas depuis scenebuilder
case = Cases.get_case(FILENAME, "scenebuilder")

# Charger les polygones à partir du fichier texte
with open(FILENAME, "r") as f:
    data = json.load(f)

# Initialiser les positions des drones
num_drones = len(case.vehicle_list)
for i in range(num_drones):
    case.vehicle_list[i].position = np.array([i % 10, int(i / 10), 0])

# Paramètres de la carte et buildings
ArenaMap.size = 0.5  # Taille du panneau; plus il y a de panneaux, meilleure est la précision mais plus le calcul est important
ArenaMap.inflation_radius = 0.3  # Les sources sont éloignés des bâtiments pour plus d'anticipation 


case.mode = ''  # Le garder à 'none' est la meilleure option en termes de vitesse / autre mode 'fancy' pour l'anticipation des drones
case.building_detection_threshold = 4  # Distance pour que les drones voient les bâtiments (impact sur les performances) / conseil : 4
# --> Aucun impact sur la même simulation avec 10 ou 4
case.max_avoidance_distance = 4  # Distance pour que les drones se voient entre eux (impact sur les performances) / conseil : 4
set_new_attribute(case, 'source_strength', 1)  # Force de la source du véhicule (pas d'impact sur les performances)

TARGET_SPEED = 4  # en m/s
FPV_SPEED = 4  # Vitesse maximale plus élevée pour le FPV
set_new_attribute(case, 'max_speed', TARGET_SPEED)  # Vitesse maximale pour les drones (pas d'impact sur les performances)

# Pour le débogage des performances, imprime tous les 500 frames le temps pris par une étape de pgflow
frame_index = 0
time_taken_pgflow = [0] * 500




class SwarmController(QObject):
    """Contrôleur pour un essaim de drones."""

    def __init__(self):
        super().__init__()

        print("Serveur OSC démarré")
        sys.stdout.flush()

        self.waiting_for_launch = True

        self.currentState = 0

        self.env = None
        #self.nb_of_drones = 5
        self.vehicle_list = case.vehicle_list

        self.target_mode = 1  # 0 pour la flotte, 1 pour individuel
        self.drone_targets = [np.zeros(3) for _ in range(self.nb_of_drones)]
        self.fleet_target = np.zeros(3)
        self.initial_drone_targets = self.drone_targets.copy()
        #self.initial_fleet_target = self.fleet_target.copy()

        self.velocities = {i: {'vx': 0, 'vy': 0, 'vz': 0} for i in range(self.nb_of_drones)}

        self.drone_fpv_index = -1
        self.action_strength = 1

        self.drone_model = ["robobee"] * self.nb_of_drones

        self.rotation = [0.0 for _ in range(self.nb_of_drones)]
        self.rotation_delta = [0.0 for _ in range(self.nb_of_drones)]
        
        # Si state sur 0 : fonctionement normal / 1 : drone en mode pause
        self.pause_state = [0 for _ in range(self.nb_of_drones)]
        self.islaunching = False

        self.trajectory_drone = [-1 for _ in range(self.nb_of_drones)]

        self.create_flying_sim()

        self.action = {
            str(i): np.array([0.3, 0.3, 0.3, 0.3]) for i in range(self.nb_of_drones)
        }

        self.simulation_timer = QTimer()
        self.simulation_timer.timeout.connect(self.update_simulation)

        """Test with an initial target for the fleet"""
        self.fleet_target = np.array([20.33, 7.74, 8.2])

        self.start_simulation()

    def create_flying_sim(self):
        """Initialiser l'environnement de simulation."""
        R = 2
        
        # Vérifier si le nombre de drones (rentré en parametre) est le même que celui du fichier json
        if self.nb_of_drones != len(case.vehicle_list):
            # Si l'utilisateur veut plus de drones, nous en créons autant que nécessaire et les ajoutons à la liste des véhicules
            if self.nb_of_drones > len(case.vehicle_list):
                while len(case.vehicle_list) < self.nb_of_drones:
                    n = len(case.vehicle_list)
                    new_vehicle = vehicle.Vehicle(source_strength=3, imag_source_strength=0)
                    new_vehicle.position = np.array([n % 10, int(n / 10), 0])
                    new_vehicle.goal = case.vehicle_list[0].goal
                    new_vehicle.ID = "Drone " + str(n + 1)
                    new_vehicle.personal_vehicle_dict = case.vehicle_list[0].personal_vehicle_dict
                    new_vehicle.arena = case.arena
                    case.vehicle_list.append(new_vehicle)
                    self.vehicle_list = case.vehicle_list

                self.drone_targets = [np.zeros(3) for _ in range(self.nb_of_drones)]
                self.initial_drone_targets = [np.zeros(3) for _ in range(self.nb_of_drones)]
                self.velocities = {i: {'vx': 0, 'vy': 0, 'vz': 0} for i in range(self.nb_of_drones)}
                self.rotation = [0.0 for _ in range(self.nb_of_drones)]
                self.action = {
                    str(i): np.array([0.3, 0.3, 0.3, 0.3]) for i in range(self.nb_of_drones)
                }

            # Si l'utilisateur veut moins de drones, nous ignorons ceux en trop
            if self.nb_of_drones < len(case.vehicle_list):
                case.vehicle_list = case.vehicle_list[:self.nb_of_drones]
                self.vehicle_list = case.vehicle_list

        aggr_phy_steps = int(SIMULATION_FREQ_HZ / CONTROL_FREQ)

        init_xyzs = np.array([v.position for v in case.vehicle_list])
        init_rpys = np.array([[0.0, 0.0, 0.0] for _ in range(self.nb_of_drones)])
        init_vels = np.array([[0.0, 0.0, 0.0] for _ in range(self.nb_of_drones)])
        for v in case.vehicle_list:
            v.state = 1 # 1 for stationary / 0 for moving to the target

        # Initialiser une trajectoire circulaire
        period = 15
        num_wp = CONTROL_FREQ * period

        target_pos = np.zeros((num_wp, 3))
        for i in range(num_wp):
            angle = (i / num_wp) * (4 * np.pi) + np.pi / 2
            target_pos[i, :] = (
                R * np.cos(angle) + init_xyzs[0, 0],
                R * np.sin(angle) - R + init_xyzs[0, 1],
                0,
            )

        target_rpys = np.zeros((num_wp, 3))

        # Créer l'environnement
        self.env = CtrlAviary(
            drone_model=self.drone_model,
            num_drones=self.nb_of_drones,
            initial_xyzs=init_xyzs,
            initial_vels=init_vels,
            initial_rpys=init_rpys,
            physics=Physics.PYB,
            neighbourhood_radius=10,  # À vérifier si impact sur pgflow
            freq=SIMULATION_FREQ_HZ,
            aggregate_phy_steps=aggr_phy_steps,
            gui=self.gui,
            record=False,
            obstacles=False,
            user_debug_gui=False,
        )

        # Obtenir l'ID du client PyBullet depuis l'environnement
        pyb_client = self.env.getPyBulletClient()

        polygons = []
        obstacles = data["scenebuilder"]["buildings"]
        sys.stdout.flush()
        for obs in obstacles:
            floor = np.array(obs["vertices"]).copy()
            floor[:, 2] = 0.0
            ceil = np.array(obs["vertices"]).copy()
            tmp = np.vstack((floor, ceil))
            polygons.append(tmp)

        # Créer les polygones dans la simulation
        for polygon_vertices in polygons:
            polygon_id = p.createCollisionShape(p.GEOM_MESH, vertices=polygon_vertices)
            p.createMultiBody(baseCollisionShapeIndex=polygon_id)

        # Initialiser les contrôleurs
        self.ctrl = [INDIControl(drone_model=drone) for drone in self.drone_model]
        # Exécuter la simulation
        self.ctrl_every_n_steps = int(np.floor(self.env.SIM_FREQ / CONTROL_FREQ))

    def update_simulation(self):
        """Mettre à jour la simulation à chaque étape de temps."""

        # Avancer la simulation
        obs, reward, done, info = self.env.step(self.action)
        self.currentState = obs

        # Mettre à jour la cible des drones si une nouvelle cible est définie
        for i in range(self.nb_of_drones):
            vehicle = case.vehicle_list[i]
            if self.target_mode == 1 and self.is_individual_target_set(i):
                if self.trajectory_drone[i] != -1:
                    if vehicle.arrived(5) : 
                        #Retirer la premiere valeur de la liste des targets
                        if len(self.trajectory_drone[i]) > 1:
                            self.trajectory_drone[i] = self.trajectory_drone[i][1:]
                            #print("new trajectory for drone", i, ": ", self.trajectory_drone[i])
                        else:
                            self.trajectory_drone[i] = -1
                            self.send_drone_end_trajectory(i)
                    
                    if self.trajectory_drone[i] != -1:
                        vehicle.goal = self.trajectory_drone[i][0]
                    
                    else :
                        vehicle.goal = self.drone_targets[i]
                
                else:
                    vehicle.goal = self.drone_targets[i]
            elif self.target_mode == 0 and self.is_fleet_target_set():
                vehicle.goal = self.fleet_target
            else:
                vehicle.state = 1
        case.vehicle_list = self.vehicle_list

        # Mettre à jour les positions des drones dans pgflow
        for j in range(self.nb_of_drones):
            if j!= self.drone_fpv_index:
                case.vehicle_list[j].position = obs[str(j)]["state"][:3]

        step_simulation(case) # Step de PgFlow

        # Calculer le contrôle pour le point de cheminement actuel
        for j in range(self.env.NUM_DRONES):

            #print("drone number : ", j, " and state : ", case.vehicle_list[j].state)
            if j == self.drone_fpv_index:
                # Si le drone est contrôlé en FPV
                desired_vector = np.array([
                    self.velocities[j]['vx'],
                    self.velocities[j]['vy'],
                    self.velocities[j]['vz']
                ])
                self.action[str(j)], _, _ = self.ctrl[j].computeControlFromState(
                    control_timestep=self.ctrl_every_n_steps * self.env.TIMESTEP,
                    state=obs[str(j)]["state"],
                    target_pos=np.hstack([obs[str(j)]["state"][:3]]),
                    target_vel=desired_vector * FPV_SPEED * self.action_strength,
                    target_rpy=self.rotation[j] * np.array([0, 0, 1])
                )
                # Réinitialiser le vecteur désiré après son traitement
                self.velocities[j] = {'vx': 0, 'vy': 0, 'vz': 0}
            
            # Si le drone est en pause on lui met un vecteur nul
            elif self.pause_state[j] == 1 :
                desired_vector = np.array([0, 0, 0])
                self.action[str(j)], _, _ = self.ctrl[j].computeControlFromState(
                    control_timestep=self.ctrl_every_n_steps * self.env.TIMESTEP,
                    state=obs[str(j)]["state"],
                    target_pos=np.hstack([obs[str(j)]["state"][:3]]),
                    target_vel=desired_vector * FPV_SPEED * self.action_strength,
                    target_rpy=self.rotation[j] * np.array([0, 0, 1])
                )

            # For the launch, the drone goes up to the target height and only then goes to the target position
            elif self.islaunching:
                desired_vector = np.array([0, 0, 2])
                self.action[str(j)], _, _ = self.ctrl[j].computeControlFromState(
                    control_timestep=self.ctrl_every_n_steps * self.env.TIMESTEP,
                    state=obs[str(j)]["state"],
                    target_pos=np.hstack([obs[str(j)]["state"][:3]]),
                    target_vel=desired_vector * TARGET_SPEED,
                    target_rpy=np.array([0, 0, 0])
                )
                if obs[str(j)]["state"][2] > self.drone_targets[j][2]:
                    self.islaunching = False 

            else :
                vehicle = case.vehicle_list[j]

                if self.waiting_for_launch :
                    target_pos = obs[str(j)]["state"][:3]
                    target_vel = np.array([0, 0, 0])
                    target_rpy = [0, 0, 0]
                
                else :
                    desired_vector = vehicle.desired_vectors[-1]
                    desired_vector = np.hstack([desired_vector, 0])

                    # Controler la hauteur du drone
                    # if obs[str(j)]["state"][2] < self.drone_targets[j][2]:
                    #     target_pos = np.hstack([obs[str(j)]["state"][:2], obs[str(j)]["state"][2] + 1])
                    # else:
                    #     target_pos = np.hstack([obs[str(j)]["state"][:2], obs[str(j)]["state"][2]])

                    if self.is_drone_on_height_target(j):
                        target_pos = np.hstack([obs[str(j)]["state"][:2], obs[str(j)]["state"][2]])
                    elif obs[str(j)]["state"][2] < self.drone_targets[j][2]:
                        target_pos = np.hstack([obs[str(j)]["state"][:2], obs[str(j)]["state"][2] + 5])
                    elif obs[str(j)]["state"][2] > self.drone_targets[j][2]:
                        target_pos = np.hstack([obs[str(j)]["state"][:2], obs[str(j)]["state"][2] - 5])


                    target_vel = desired_vector * vehicle.max_speed
                    if vehicle.state == 0: #update the rotation only if the drone is moving
                        self.rotation[j] = np.arctan2(desired_vector[0], desired_vector[1])

                    target_rpy = (self.rotation[j] + self.rotation_delta[j]) * np.array([0, 0, 1]) 

                    #target_rpy = [0, 0, np.arctan2(desired_vector[1], desired_vector[0])]
                    #target_rpy = [0, 0, pi]
                    # if j == 0:
                        # print("Drone ", j)
                        # print("target_pos : ", target_pos)
                        # print("target_vel : ", target_vel)
                        # print("target_rpy : ", target_rpy)
                        # print("desired_vector : ", desired_vector)
                        # print("obs state : ", obs[str(j)]["state"])
                        # print()

                # Calculer la commande de contrôle
                self.action[str(j)], _, _ = self.ctrl[j].computeControlFromState(
                    control_timestep=self.ctrl_every_n_steps * self.env.TIMESTEP,
                    state=obs[str(j)]["state"],
                    target_pos=target_pos, 
                    target_vel=target_vel,
                    target_rpy=target_rpy
                )


    def is_individual_target_set(self, i):
        """Vérifier si la cible individuelle pour le drone i est définie."""
        return np.any(self.drone_targets[i])

    def is_fleet_target_set(self):
        """Vérifier si la cible de la flotte est définie."""
        return np.any(self.fleet_target)

    def start_simulation(self):
        """Démarrer le minuteur de simulation."""
        self.simulation_timer.start(CONTROL_RATE)

    def stop_simulation(self):
        """Arrêter le minuteur de simulation."""
        self.simulation_timer.stop()

    def close_env(self):
        """Fermer l'environnement de simulation."""
        self.env.close()

    def set_drone_state_to_launch(self):
        self.waiting_for_launch = False
        self.islaunching = True
        for v in case.vehicle_list:
            v.state = 0

    def is_drone_on_height_target(self, i):
        height = self.currentState[str(i)]["state"][2]
        target_height = self.drone_targets[i][2]
        # return true if height is less or more 3 than the target height
        return height <= target_height + 3 and height >= target_height - 3


if __name__ == "__main__":
    app = QApplication(sys.argv)
    controller = SwarmController()
    app.aboutToQuit.connect(controller.stop_simulation)
    app.aboutToQuit.connect(controller.close_env)
    app.exec()
