import enum

# Should we apply deadband correction to the controller inputs?
CONTROLLER_CORRECT_FOR_DEADBAND = True
# The deadband for the controller joysticks
CONTROLLER_DEADBAND = 0.3
# The port the controller is connected to
CONTROLLER_PORT = 0

@enum.unique
class CANID(enum.IntEnum):
    # CAN IDs for motors and other devices
    FRONT_LEFT_DRIVE = 12
    FRONT_LEFT_STEER = 3
    FRONT_LEFT_STEER_ENCODER = 10
    
    FRONT_RIGHT_DRIVE = 5   
    FRONT_RIGHT_STEER = 6
    FRONT_RIGHT_STEER_ENCODER = 1
    
    REAR_RIGHT_DRIVE = 9
    REAR_RIGHT_STEER = 11
    REAR_RIGHT_STEER_ENCODER = 4

    REAR_LEFT_DRIVE = 8
    REAR_LEFT_STEER = 2
    REAR_LEFT_STEER_ENCODER = 7
    
    # Pigeon2 IMU
    PIGEON = 20