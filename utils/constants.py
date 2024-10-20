# Constants we may need
# Just starting with the minimum stuff we need
# The math conversions are under units.py

#######################################################################################
## FIELD DIMENSIONS
#######################################################################################

FIELD_X_M = 16.54 # "Length"
FIELD_Y_M = 8.21  # "Width"

#######################################################################################
## CAN ID'S
#######################################################################################

# Reserved_CANID = 0
# Reserved_CANID = 1
DT_FR_WHEEL_CANID = 2
DT_FR_AZMTH_CANID = 3
DT_FL_WHEEL_CANID = 4
DT_FL_AZMTH_CANID = 5
DT_BR_WHEEL_CANID = 6
DT_BR_AZMTH_CANID = 7
DT_BL_AZMTH_CANID = 8
DT_BL_WHEEL_CANID = 9
# Unused_CANID = 10
# Unused_CANID = 11
# Unused_CANID = 12
# Unused_CANID = 13
# Unused_CANID = 14
# Unused_CANID = 15
# Unused_CANID = 16




#######################################################################################
## PWM Bank
#######################################################################################

# Unused = 0
# Unused = 1
# Unused = 2
# Unused = 3
# Unused = 4
# Unused = 5
# Unused = 6
# Unused = 7
# Unused = 8
LED_STACK_LIGHT_CTRL_PWM = 9


#######################################################################################
## DIO Bank
#######################################################################################

DT_BR_AZMTH_ENC_PORT = 0
DT_FL_AZMTH_ENC_PORT = 1
DT_BL_AZMTH_ENC_PORT = 2
DT_FR_AZMTH_ENC_PORT = 3
# Unused = 4
# Unused = 5
# Unused = 6
# Unused = 7
FIX_ME_LED_PIN = 8
HEARTBEAT_LED_PIN = 9
