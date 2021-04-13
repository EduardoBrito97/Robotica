import sys
import time
import numpy as np
import math
import matplotlib as mpl

def main(client_id, vrep):
    PI=math.pi  #pi=3.14..., constant

    #retrieve motor  handles
    error_code, left_motor_handle = vrep.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
    error_code, right_motor_handle = vrep.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)

    sensor_h = [] #empty list for handles
    sensor_val = np.array([]) #empty array for sensor measurements

    #orientation of all the sensors: 
    sensor_loc = np.array([-PI/2, -50/180.0*PI,-30/180.0*PI,-10/180.0*PI,10/180.0*PI,30/180.0*PI,50/180.0*PI,PI/2,PI/2,130/180.0*PI,150/180.0*PI,170/180.0*PI,-170/180.0*PI,-150/180.0*PI,-130/180.0*PI,-PI/2]) 

    #for loop to retrieve sensor arrays and initiate sensors
    for x in range(1,16+1):
            error_code,sensor_handle = vrep.simxGetObjectHandle(client_id, 'Pioneer_p3dx_ultrasonicSensor'+str(x), vrep.simx_opmode_oneshot_wait)
            sensor_h.append(sensor_handle) #keep list of handles        
            error_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = vrep.simxReadProximitySensor(client_id, sensor_handle, vrep.simx_opmode_streaming)                
            sensor_val = np.append(sensor_val, np.linalg.norm(detected_point)) #get list of values            

    t = time.time()

    while (time.time()-t) < 60:
        #Loop Execution
        sensor_val=np.array([])    
        for x in range(1,16+1):
            error_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = vrep.simxReadProximitySensor(client_id, sensor_h[x-1], vrep.simx_opmode_buffer)                
            sensor_val = np.append(sensor_val, np.linalg.norm(detected_point)) #get list of values

        #controller specific
        sensor_sq = sensor_val[0:8] * sensor_val[0:8] #square the values of front-facing sensors 1-8
            
        min_ind = np.where(sensor_sq==np.min(sensor_sq))
        min_ind = min_ind[0][0]
        
        if sensor_sq[min_ind]<0.2:
            steer = -1/sensor_loc[min_ind]
        else:
            steer = 0
                
        v = 1	#forward velocity
        kp = 0.5	#steering gain
        vl = v + kp * steer
        vr = v - kp * steer
        print("V_l =", vl)
        print("V_r =", vr)

        error_code = vrep.simxSetJointTargetVelocity(client_id, left_motor_handle, vl, vrep.simx_opmode_streaming)
        error_code = vrep.simxSetJointTargetVelocity(client_id, right_motor_handle, vr, vrep.simx_opmode_streaming)

        time.sleep(0.2) #loop executes once every 0.2 seconds (= 5 Hz)

    #Post ALlocation
    error_code = vrep.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, vrep.simx_opmode_streaming)
    error_code = vrep.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, vrep.simx_opmode_streaming)