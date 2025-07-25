######### Server messages #########
# message that the server can accept (from unity clients)

###
# OSC address to notify simulation that the user has exited FPV mode (droneFPVIndex reset to -1)
###
EXIT_FPV_MODE = "/reset"


###
# OSC address for setting drone velocities
# parameters: id, vx, vy, vz, actionStrength
# id: drone id
# vx, vy, vz: velocity in m/s
# actionStrength: to what extent the controller was pressed
# example: /set_drone_velocities [1, 0.314159, 0.2653589, 0.7932384]
###
SET_DRONE_VELOCITIES= "/set_drone_velocities"

###
# OSC address for setting drone rotation
# parameters: id, direction, actionStrength
# id: drone id
# direction: 1 or -1, to rotate to the right or to the left
# rotationStrength: to what extent the controller was pressed
# example: /set_drone_rotation [1, 1, 0.626433]
###
SET_DRONE_ROTATION= "/set_drone_rotation"

###
# OSC address for setting drone rotation delta
# parameters: id, direction, actionStrength
# id: drone id
# strenght: 1 or 2
# example: /set_velocity_strength [1, 2]
###
SET_DRONE_VELOCITY_STRENGTH= "/set_velocity_strength"


###
# OSC address for setting drone velocity factor
# parameters: id, velocity_strengh
# id: drone id
# direction: 1 or -1, to rotate to the right or to the left
# rotationStrength: to what extent the controller was pressed
# example: /set_drone_rotation [1, 1, 0.626433]
###
SET_DRONE_ROTATION_DELTA= "/set_drone_rotation_delta"



###
# OSC address for setting drone target
# parameters: id, x, y, z
# id: drone id
# x, y, z: target position in world coordinates
# example: /set_drone_target [1, 0.8327950, 0.28841971, 0.693993]
###
SET_DRONE_TARGET = "/set_drone_target"

###
# OSC address for setting drone trajectory
# parameters: id,
# id: drone id
# 
###
SET_DRONE_TRAJECTORY = "/set_drone_trajectory"

###
# OSC address for setting drone target
# parameters: id, height
# id: drone id
# height : the target modification height
# example: /set_drone_target [1, -5]
###
SET_DRONE_TARGET_HEIGHT = "/set_drone_target_height"


###
# OSC address for setting target to entire fleet
# parameters: x, y, z
# x, y, z: target position in world coordinates
# example: /set_fleet_target [0.7510582, 0.0974944, 0.781640]
###
SET_FLEET_TARGET= "/set_fleet_target"

###
# OSC address for setting target mode
# parameters: mode
# mode: integer value representing the target mode, 0 for fleet, 1 for individual
# example: /set_target_mode 0
###
SET_TARGET_MODE= "/set_target_mode"

###
# OSC address for resetting drone targets
# parameters: none
###
RESET_TARGETS = "/reset_targets"

###
# OSC address for sending debug messages
# parameters: message
###
DEBUG_MESSAGE = "/debug_message"

###
# OSC address for sending drone landing command
###
LAUNCH_DRONE = "/launch_drone"

###
# OSC address for getting the zone coordinates and placing target points
# parameters: list of coordinates in str
###
SET_ZONE = "/set_zone"

###
# OSC address for sending the target of each drones inside the selected zone
# parameters: list of coordinates in str
###
SET_TARGETS_IN_ZONE = "/set_targets_in_zone"

###
# OSC address for make pause on one drone
# parameter : id of the drone
###
SET_PAUSE_DRONE = "/set_pause_drone"

###
# OSC address for make pause on one drone
# parameter : id of the drone
###
SET_PLAY_DRONE = "/set_play_drone"






######### Answer messages #########
#  message that the server can send to unity clients

###
# OSC address for sending drone data
# parameters: x, y, z , rx, ry, rz
# x, y, z: position in meters
# rx, ry, rz: rotation in radians
###
SEND_DRONE_DATA = "/send_drone_data"

###
# OSC addresses for sending number of drones selected by the user ('--num_drones' command line argument)
# parameters: n
# n: number of drones
###
SEND_NUM_DRONES = "/send_num_drones"

###
# OSC address for sending when the drone end trajectory
# parameters: id
###
DRONE_END_TRAJECTORY = "/send_end_trajectory"

###
# OSC address for sending when the drone reached the first point of the trajectory
# parameters: id
###
DRONE_REACHED_FIRST_POINT_TRAJ = "/send_reached_first_point_trajectory"