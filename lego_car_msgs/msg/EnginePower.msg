#The Lego Car can be operated in two ways. TANKMODE and CARMODE
#In TANKMODE you control the two chain-drives of the car. 
#left_engine = Left chain-drive
#right_engine = Right chain-drive
#-------------------------------
#In CARMODE you control the model like a car with a steering_angle and a speed(throttle)
#-------------------------------
#The mode is used for the subscriber who uses this variable to see which variables/Mode is used

uint8 TANKMODE = 0
uint8 CARMODE = 1

Header header
uint8 mode
int16 left_engine
int16 right_engine
int16 speed
float32 steering_angle
