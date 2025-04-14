# DynamixelControllerConstants.msg
# =============================
# Instruction codes (命令コード)
# -----------------------------
uint8 PING            = 1
uint8 READ_DATA       = 2
uint8 WRITE_DATA      = 3
uint8 REG_WRITE       = 4
uint8 ACTION          = 5
uint8 FACTORY_RESET   = 6
uint8 REBOOT          = 8
uint8 SYNC_READ       = 130    # 0x82 in hex
uint8 SYNC_WRITE      = 131    # 0x83 in hex
uint8 BULK_READ       = 146    # 0x92 in hex
uint8 BULK_WRITE      = 147    # 0x93 in hex

# EEPROM領域のアドレス（読み出し専用または読み書き）
# -----------------------------
uint8 MODEL_NUMBER         = 0     # 0x00
uint8 MODEL_INFORMATION    = 2     # 0x02
uint8 FIRMWARE_VERSION     = 6     # 0x06
uint8 ID                   = 7     # 0x07
uint8 BAUD_RATE            = 8     # 0x08
uint8 RETURN_DELAY_TIME    = 9     # 0x09
uint8 DRIVE_MODE           = 10    # 0x0A
uint8 OPERATING_MODE       = 11    # 0x0B
uint8 SECONDARY_ID         = 12    # 0x0C
uint8 PROTOCOL_TYPE        = 13    # 0x0D

uint8 HOMING_OFFSET        = 20    # 0x14
uint8 MOVING_THRESHOLD     = 24    # 0x18
uint8 TEMPERATURE_LIMIT    = 31    # 0x1F
uint8 MAX_VOLTAGE_LIMIT    = 32    # 0x20
uint8 MIN_VOLTAGE_LIMIT    = 34    # 0x22
uint8 PWM_LIMIT            = 36    # 0x24
uint8 CURRENT_LIMIT        = 38    # 0x26
uint8 VELOCITY_LIMIT       = 40    # 0x28
uint8 MAX_POSITION_LIMIT   = 44    # 0x2C
uint8 MIN_POSITION_LIMIT   = 48    # 0x30

# RAM領域のアドレス（読み出し専用または読み書き）
# -----------------------------
uint8 TORQUE_ENABLE        = 64    # 0x40
uint8 LED                  = 65    # 0x41
uint8 STATUS_RETURN_LEVEL  = 68    # 0x44
uint8 REGISTERED_INSTRUCTION = 69   # 0x45
uint8 HARDWARE_ERROR_STATUS  = 70   # 0x46
uint8 VELOCITY_I_GAIN      = 76    # 0x4C
uint8 VELOCITY_P_GAIN      = 78    # 0x4E
uint8 POSITION_D_GAIN      = 80    # 0x50
uint8 POSITION_I_GAIN      = 82    # 0x52
uint8 POSITION_P_GAIN      = 84    # 0x54
uint8 FEEDFORWARD_2_GAIN   = 88    # 0x58
uint8 FEEDFORWARD_1_GAIN   = 90    # 0x5A
uint8 BUS_WATCHDOG         = 98    # 0x62
uint8 GOAL_PWM             = 100   # 0x64
uint8 GOAL_CURRENT         = 102   # 0x66
uint8 GOAL_VELOCITY        = 104   # 0x68
uint8 PROFILE_ACCELERATION = 108   # 0x6C
uint8 PROFILE_VELOCITY     = 112   # 0x70
uint8 GOAL_POSITION        = 116   # 0x74
uint8 REALTIME_TICK        = 120   # 0x78
uint8 MOVING               = 122   # 0x7A
uint8 PRESENT_PWM          = 124   # 0x7C
uint8 PRESENT_CURRENT      = 126   # 0x7E
uint8 PRESENT_VELOCITY     = 128   # 0x80
uint8 PRESENT_POSITION     = 132   # 0x84
uint8 PRESENT_INPUT_VOLTAGE = 144  # 0x90
uint8 PRESENT_TEMPERATURE  = 146   # 0x92
