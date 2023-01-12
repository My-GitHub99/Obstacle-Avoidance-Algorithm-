import sim
import sys
import numpy as np
import cv2
import time

print('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5) # Connect to CoppeliaSim
if clientID != -1:
    print('Connected to remote API server')
else:
    print('Connection not successful')
    sys.exit('Connection timed out')



# Motor's speeds (Global variables)
FR = 0
FL = 0
BR = 0
BL = 0
min_distance_to_wall=0.25

# delay function
def delay(i):
 for i in range(0,i):
            print(i)


 # object handlers for motors,sensors and vision module initialization

error_code_FR, FR_motor = sim.simxGetObjectHandle(clientID, 'wheel_FR', sim.simx_opmode_blocking)
error_code_FL, FL_motor = sim.simxGetObjectHandle(clientID, 'wheel_FL', sim.simx_opmode_blocking)
error_code_BR, BR_motor = sim.simxGetObjectHandle(clientID, 'wheel_BR', sim.simx_opmode_blocking)
error_code_BL, BL_motor = sim.simxGetObjectHandle(clientID, 'wheel_BL', sim.simx_opmode_blocking)
error_code_cam,cam = sim.simxGetObjectHandle(clientID, 'camera', sim.simx_opmode_blocking)
error_code_sensor_center, proximity_center = sim.simxGetObjectHandle(clientID, 'Proximity_center', sim.simx_opmode_blocking)
error_code_sensor_right, proximity_right = sim.simxGetObjectHandle(clientID, 'Proximity_right', sim.simx_opmode_blocking)
error_code_sensor_left, proximity_left = sim.simxGetObjectHandle(clientID, 'Proximity_left', sim.simx_opmode_blocking)
print(error_code_FR, error_code_FL, error_code_BR, error_code_BL, error_code_sensor_center, error_code_sensor_right,error_code_sensor_left,error_code_cam)

# Adjust thresholding BRG values for red
lower_red = np.array([0, 0, 100])
upper_red = np.array([50, 50, 255])


#Adjust Blob detector parameters and define it.
params = cv2.SimpleBlobDetector_Params()
det = cv2.SimpleBlobDetector_create(params)

# Basic linear and rotation speed
rspeed=2
mspeed=20

global doctor_flag
doctor_seen_flag=0
global doctor_arrive_flag
doctor_arrive_flag=0
global table_flag
table_flag=0
global detection_decision
detection_decision=0
global obstacles_counter
obstacles_counter=0

# Reading date from sensors and camera
returnCode, proximity_center_state, detection_point_front, front_object_handler, i = sim.simxReadProximitySensor( clientID,  proximity_center , sim.simx_opmode_streaming)
returnCode, proximity_right_state, detection_point_right, right_object_hander, i = sim.simxReadProximitySensor( clientID, proximity_right, sim.simx_opmode_streaming)
returnCode, proximity_left_state, detection_point_left, left_object_handler, i = sim.simxReadProximitySensor( clientID, proximity_left, sim.simx_opmode_streaming)
returncode, resolution,image = sim.simxGetVisionSensorImage(clientID, cam,0, sim.simx_opmode_streaming)


# creating a function to set target velocity of joints
def set_joint_speed():
    error_code1 = sim.simxSetJointTargetVelocity(clientID, FR_motor, FR, sim.simx_opmode_blocking)
    error_code2 = sim.simxSetJointTargetVelocity(clientID, FL_motor, FL, sim.simx_opmode_blocking)
    error_code3 = sim.simxSetJointTargetVelocity(clientID, BR_motor, BR, sim.simx_opmode_blocking)
    error_code4 = sim.simxSetJointTargetVelocity(clientID, BL_motor, BL, sim.simx_opmode_blocking)

print("start to create functions")
# here we are going to create 4 functions responsible for assigning suitable values to the motors for the robot to to: move forward, turn right, turn left, move backward
# Robot Moving forward function
def move_forward():
 global FR
 FR=5
 global FL
 FL=5
 global BR
 BR=5
 global BL
 BL=5
 return
# robot turning left
def move_left():
    global FR
    FR = 1
    global FL
    FL = -1
    global BR
    BR = 1
    global BL
    BL = -1
    return
# robot turning left
def move_right():
    global FR
    FR = -3.75
    global FL
    FL = 3.75
    global BR
    BR = -3.75
    global BL
    BL = 3.75
    return
# robot stop
def stop():
    global FR
    FR = 0
    global FL
    FL = 0
    global BR
    BR = 0
    global BL
    BL = 0
    return
# robot moving backward
def move_backward():
    global FR
    FR = -2.5
    global FL
    FL = -2.5
    global BR
    BR = -2.5
    global BL
    BL = -2.5
    return
print("functions created")
# this function returns the suitable values of motors to turn: right,left or forward depending on the sensors readings
print(" processing sensor's reading")
def motors_values_return(proximity_center_state, proximity_right_state, proximity_left_state ):
# obscticle is detected ahead
 if proximity_center_state == True and doctor_arrive_flag!=1 and obstacles_counter==0:
  delay(5000)
  stop()
  set_joint_speed()
  print("stoping")
  delay(150000)

# right is blocked== turn left
  if proximity_right_state == True and proximity_left_state == False:
         print("Blocked right>>>>> moving left")
         move_left()
         set_joint_speed()
         delay(100000)


# left is blocked== turn right
  elif proximity_right_state == False and proximity_left_state == True :
         print("Blocked left>>>>> moving right")
         move_right()
         set_joint_speed()
         delay(150000)

# neither left nor right are blocked == turn left as a default
  elif proximity_right_state == False and proximity_left_state == False:
         print("left and right are clear>>>>>move left by default")
         move_backward()
         set_joint_speed()
         delay(250000)
         move_left()
         set_joint_speed()
         delay(150000)
  # objects on right,left and ahead
  elif proximity_left_state == True and proximity_center_state == True:
      print("object from all sides>>>>> moving backward")
      move_backward()
      set_joint_speed()
      delay(100000)
      print("moving write")
      move_right()
      set_joint_speed()
      delay(150000)




while True:
 returnCode1, proximity_center_state, detection_point_front, front_object_handler, x = sim.simxReadProximitySensor(clientID, proximity_center, sim.simx_opmode_buffer)
 returnCode2, Proximity_right_state, detection_point_right, right_object_handler, x = sim.simxReadProximitySensor( clientID, proximity_right, sim.simx_opmode_buffer)
 returnCode3, Proximity_left_state, detection_point_left, left_object_handler, x = sim.simxReadProximitySensor( clientID, proximity_left, sim.simx_opmode_buffer)
 returncode4, resolution, image = sim.simxGetVisionSensorImage(clientID, cam, 0, sim.simx_opmode_buffer)



 #image processing

 returncode4, resolution, image = sim.simxGetVisionSensorImage(clientID, cam, 0, sim.simx_opmode_buffer)
 img = np.array(image, dtype=np.uint8)
 if resolution:
     img = img.reshape([resolution[0], resolution[1], 3])
     img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
     img = cv2.flip(img, 0)
     mask = cv2.inRange(img, lower_red, upper_red)  # thresholding for red
     #mask = cv2.bitwise_not(mask)  # Inverse the mask as Blob detector works on the dark areas
     cv2.imshow('target', mask)
     cv2.imshow('Objects Detected', img)
     mask = cv2.GaussianBlur(mask, (5,5),0)  # filtering after thresholding
     contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE,   cv2.CHAIN_APPROX_SIMPLE)  # detecting contor of seen objects
     keypts = det.detect(mask)
     for cnt in contours:
         area = cv2.contourArea(cnt)  # calculating contour area
         approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True),True)  # approximation for number of the drawn contour points
         x = approx.ravel()[0]  # x position of contour in image
         y = approx.ravel()[1]  # y position of contour in image

         # rotating and searching for the doctor

         if area > 100:
             cv2.drawContours(img, [approx], 0, (255, 0, 0),3)
             cv2.imshow('Objects Detected', img)
             print(len(approx))

             # arriving at the doctor case
             if len(approx) == 5 and proximity_center_state ==True and doctor_seen_flag==1:
                 print("arrived at the doctor location...... executing a task")
                 doctor_arrive_flag = 1
                 stop()
                 set_joint_speed()
                 delay(150000)



             # searching for the doctor
             elif len(approx)<5 and proximity_center_state==False:
                 doctor_seen_flag = 0
                 print("nothing found>>>searching for the doctor")
                 error_code1 = sim.simxSetJointTargetVelocity(clientID, FR_motor, 1, sim.simx_opmode_blocking)
                 error_code2 = sim.simxSetJointTargetVelocity(clientID, FL_motor, -1, sim.simx_opmode_blocking)
                 error_code3 = sim.simxSetJointTargetVelocity(clientID, BR_motor, 1, sim.simx_opmode_blocking)
                 error_code4 = sim.simxSetJointTargetVelocity(clientID, BL_motor, -1, sim.simx_opmode_blocking)




             # seeing Doctor, the robot moves towrads him
             elif len(approx)>5 and proximity_center_state==False:
                 cv2.putText(img, "doctor", (x, y), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (50, 100, 0))
                 print("doctor found>>>>going to him")
                 doctor_seen_flag=1
                 obstacle_flag=0
                 error_code1 = sim.simxSetJointTargetVelocity(clientID, FR_motor, 2.5, sim.simx_opmode_blocking)
                 error_code2 = sim.simxSetJointTargetVelocity(clientID, FL_motor, 2.5, sim.simx_opmode_blocking)
                 error_code3 = sim.simxSetJointTargetVelocity(clientID, BR_motor, 2.5, sim.simx_opmode_blocking)
                 error_code4 = sim.simxSetJointTargetVelocity(clientID, BL_motor, 2.5, sim.simx_opmode_blocking)

             #seeing the doctor, but an obsticle is sensed ahead
             elif  proximity_center_state == True and doctor_seen_flag :
                 motors_values_return(proximity_center_state, proximity_right_state, proximity_left_state)
                 obstacles_counter=obstacles_counter+1

             # not seeing the doctor, but an obsticle is sensed..... robot moves away from it
             elif doctor_seen_flag!=1 and proximity_center_state==True:
                motors_values_return(proximity_center_state, proximity_right_state, proximity_left_state)

             #seeing table,the robot acknowladges seeing it and continues searching
             elif len(approx)==5 and proximity_center_state==False and doctor_seen_flag==0:
                 cv2.putText(img, "Table", (x, y), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (50, 100, 0))
                 print("Table Found>>>>ignore it")
                 error_code1 = sim.simxSetJointTargetVelocity(clientID, FR_motor, 0.2, sim.simx_opmode_blocking)
                 error_code2 = sim.simxSetJointTargetVelocity(clientID, FL_motor, -0.2, sim.simx_opmode_blocking)
                 error_code3 = sim.simxSetJointTargetVelocity(clientID, BR_motor, 0.2, sim.simx_opmode_blocking)
                 error_code4 = sim.simxSetJointTargetVelocity(clientID, BL_motor, -0.2, sim.simx_opmode_blocking)
                 table_flag==1

             #seeing table, and also the robot senses obsticle ahead.... robot moves away from it
             elif len(approx)==5 and proximity_center_state==True and doctor_seen_flag==0:
                 motors_values_return(proximity_center_state, proximity_right_state, proximity_left_state)
                 table_flag == 1


             print(table_flag,obstacles_counter,doctor_seen_flag,doctor_arrive_flag)




 c = cv2.waitKey(1)





