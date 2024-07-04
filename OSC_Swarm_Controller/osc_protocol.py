### Server messages
# message that the server can accept (from unity clients)
# parameters: 

RESET = "/drone/reset"
FORWARD = "/drone/forward"

### Answer messages
#
#  message that the server can send to unity clients

###
# OSC addresses for sending data
# parameters: x, y, z , rx, ry, rz
# x, y, z: position in meters
# rx, ry, rz: rotation in radians
###
SEND_DRONE_DATA = "/drone"