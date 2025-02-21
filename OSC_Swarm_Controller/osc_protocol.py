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
# OSC address for setting drone target
# parameters: id, x, y, z
# id: drone id
# x, y, z: target position in world coordinates
# example: /set_drone_target [1, 0.8327950, 0.28841971, 0.693993]
###
SET_DRONE_TARGET = "/set_drone_target"

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