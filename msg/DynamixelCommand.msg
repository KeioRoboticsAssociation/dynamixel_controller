# DynamixelCommand.msg
# Command message for Dynamixel operations

uint8 command      # Command type (PING, READ_DATA, WRITE_DATA, SYNC_READ, SYNC_WRITE, etc.)
uint8[] ids        # Dynamixel IDs to target
uint16 address     # Register address
uint16 length      # Data length
uint8[] data       # Data payload for write operations