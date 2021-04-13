import sys
import time
import numpy as np
import math
import matplotlib as mpl

vrep = None
client_id = None
PI = math.pi

L = 0.33

def get_object_pos(object_name):
    _, object_handle = vrep.simxGetObjectHandle(client_id, object_name, vrep.simx_opmode_oneshot_wait)
    _, object_pos = vrep.simxGetObjectPosition(client_id, object_handle, -1, vrep.simx_opmode_streaming)

    _, orientation = vrep.simxGetObjectOrientation(client_id, object_handle, -1, vrep.simx_opmode_oneshot_wait)

    theta = orientation[2]
    object_pos[2] = theta
    return object_pos

def move_to_target(target):
    _, left_motor_handle = vrep.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
    _, right_motor_handle = vrep.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)

    robot_pos = get_object_pos('Pioneer_p3dx')
    print("Posição Robô: X = {:.2f}, Y = {:.2f}, Teta = {:.2f}".format(robot_pos[0], robot_pos[1], robot_pos[2]))

    k_p = 4.0
    k_a = 14.0
    k_b = -9.0

    delta_x = target[0] - robot_pos[0]
    delta_y = target[1] - robot_pos[1]
    ro = ((delta_x ** 2) + (delta_y ** 2)) ** (1/2)
    alpha = -robot_pos[2] + math.atan2(delta_y, delta_x)

    v = k_p * ro

    if (alpha <= (-PI/2)):
        v = -v
        alpha += PI
    elif(alpha > (PI / 2)):
        v = -v
        alpha -= PI

    beta = -robot_pos[2] - alpha + target[2]
    w = (k_a * alpha) + (k_b * beta)

    if (ro < 0.08):
        v = 0

    lw_half = ((L*w)/2) 
    vl = v - lw_half
    vr = v + lw_half

    vrep.simxSetJointTargetVelocity(client_id, left_motor_handle, vl, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(client_id, right_motor_handle, vr, vrep.simx_opmode_streaming)

def main(client_id_connected, vrep_lib):
    global vrep, client_id
    vrep = vrep_lib
    client_id = client_id_connected

    target_pos = None

    sensor_h = [] #empty list for handles
    sensor_val = np.array([]) #empty array for sensor measurements

    #orientation of all the sensors: 
    sensor_loc = np.array([-PI/2, -50/180.0*PI,-30/180.0*PI,-10/180.0*PI,10/180.0*PI,30/180.0*PI,50/180.0*PI,PI/2,PI/2,130/180.0*PI,150/180.0*PI,170/180.0*PI,-170/180.0*PI,-150/180.0*PI,-130/180.0*PI,-PI/2]) 

    #for loop to retrieve sensor arrays and initiate sensors
    for x in range(1,16+1):
            _, sensor_handle = vrep.simxGetObjectHandle(client_id, 'Pioneer_p3dx_ultrasonicSensor'+str(x), vrep.simx_opmode_oneshot_wait)
            sensor_h.append(sensor_handle) #keep list of handles        
            _, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = vrep.simxReadProximitySensor(client_id, sensor_handle, vrep.simx_opmode_streaming)                
            sensor_val = np.append(sensor_val, np.linalg.norm(detected_point)) #get list of values            

    t = time.time()

    while (time.time() - t) < 60:
        #Loop Execution
        sensor_val=np.array([])    
        for x in range(1,16+1):
            error_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = vrep.simxReadProximitySensor(client_id, sensor_h[x-1], vrep.simx_opmode_buffer)                
            sensor_val = np.append(sensor_val, np.linalg.norm(detected_point)) #get list of values

        #controller specific
        sensor_sq = sensor_val[0:8] * sensor_val[0:8] #square the values of front-facing sensors 1-8
            
        min_ind = np.where(sensor_sq==np.min(sensor_sq))
        min_ind = min_ind[0][0]
        
        old_target = target_pos
        target_pos = get_object_pos('Target#')

        if old_target != target_pos:
            print("Objetivo atualizado para: X = {:.2f}, Y = {:.2f}, Teta = {:.2f}".format(target_pos[0], target_pos[1], target_pos[2]))

        move_to_target(target_pos)

        time.sleep(0.01) #loop executes once every 0.05 seconds (= 20 Hz)