######### Server messages #########
# message that the server can accept (from unity clients)

EXIT_FPV_MODE = "/reset"
FORWARDtoremove = "/forward"

###
# OSC address for setting drone velocities
# parameters: id, vx, vy, vz
# id: drone id
# vx, vy, vz: velocity in m/s
# example: /set_drone_velocities [1, 0.314159, 0.2653589, 0.7932384]
###
SET_DRONE_VELOCITIES= "/set_drone_velocities"

###
# OSC address for setting drone target
# parameters: id, x, y, z
# id: drone id
# x, y, z: target position in world coordinates
# example: /set_drone_target [1, 0.626433, 0.8327950, 0.28841971]
###
SET_DRONE_TARGET = "/set_drone_target"

###
# OSC address for setting target to entire fleet
# parameters: x, y, z
# x, y, z: target position in world coordinates
# example: /set_fleet_target [0.693993, 0.7510582, 0.0974944]
###
SET_FLEET_TARGET= "/set_fleet_target"

######### Answer messages #########
#  message that the server can send to unity clients

###
# OSC addresses for sending data
# parameters: x, y, z , rx, ry, rz
# x, y, z: position in meters
# rx, ry, rz: rotation in radians
###
SEND_DRONE_DATA = "/drone"