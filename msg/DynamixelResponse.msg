# DynamixelResponse.msg
# Response message from Dynamixel operations

uint8 command      # Original command type
uint8[] ids        # Dynamixel IDs that responded
uint8[] error      # Error status for each ID
uint8[] data       # Response data payload