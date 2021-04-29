import sys
import time
import numpy as np
import math
import matplotlib as mpl
import logging
from graph import Graph

vrep = None
client_id = None
PI = math.pi

L = 0.33

def to_180_range(angle):
    angle = math.fmod(angle,2*PI)
    if(angle < -PI):
        angle += (2*PI)
    elif(angle> PI):
        angle -= (2*PI)
    return angle

def is_far_enough(sens_1, sens_2):
    if(sens_1 < 0.01):
        return sens_2 > 0.30 or sens_2 < 0.01
    elif(sens_2 < 0.01):
        return sens_1 > 0.30 or sens_1 < 0.01 
    
    mean = (sens_1 + sens_2)/2
    return mean > 0.30

def between_walls(sens_1, sens_2, sens_3, sens_4):
    sup = 0.37
    inf = 0.02
    return (sens_1 < sup and sens_2 < sup and sens_3 < sup and sens_4 < sup) and (sens_1 > inf and sens_2 > inf and sens_3 > inf and sens_4 > inf)

def turn(sens_l_1, sens_l_2, sens_r_1, sens_r_2, orientation_before, orientation_now, target_before):
    k_w = 0.05
    vl = 0
    vr = 0

    orientation_now = to_180_range(orientation_now)
    orientation_before = to_180_range(orientation_before)

    if between_walls(sens_l_1, sens_l_2, sens_r_1, sens_r_2):
        print('is a endpoint')
        vl = 0
        vr = 0
    elif is_far_enough(sens_l_1, sens_l_2):
        vl = -1
        vr = 1
    elif is_far_enough(sens_r_1, sens_r_2):
        vl = 1
        vr = -1
    target = abs(abs(orientation_before) - abs(orientation_now)) - (PI/2)

    #print_sensors("Esquerda", sens_l_1, sens_l_2)
    #print_sensors("Direita", sens_r_1, sens_r_2)

    #print('before : ',orientation_before)
    #print('now: ',orientation_now)
    #print('target ',target)

    set_speed(vl*k_w*abs(target), vr*k_w*abs(target))

    diference =  abs(target_before) - abs(target)
    target_before = target

    return (diference <= 0), target_before 

def get_object_pos(object_name):
    _, object_handle = vrep.simxGetObjectHandle(client_id, object_name, vrep.simx_opmode_oneshot_wait)
    _, object_pos = vrep.simxGetObjectPosition(client_id, object_handle, -1, vrep.simx_opmode_streaming)

    _, orientation = vrep.simxGetObjectOrientation(client_id, object_handle, -1, vrep.simx_opmode_oneshot_wait)

    theta = orientation[2]
    object_pos[2] = theta
    return object_pos

def move_to_target(target):
    robot_pos = get_object_pos('Pioneer_p3dx')

    k_p = 0.8
    k_a = 1.4

    delta_x = target[0] - robot_pos[0]
    delta_y = target[1] - robot_pos[1]
    ro = ((delta_x ** 2) + (delta_y ** 2)) ** (1/2)
    alpha = -robot_pos[2] + math.atan2(delta_y, delta_x)

    v = k_p * ro
    if alpha <= (-PI / 2):
        v = -v
        alpha += PI
    elif alpha > (PI / 2):
        v = -v
        alpha -= PI

    # Beta não importa, pois não precisamos saber a orientação
    w = (k_a * alpha)

    if w > 0.01:
        v = 0
    else:
        w = 0

    lw_half = ((L*w)/2) 
    vl = v - lw_half
    vr = v + lw_half

    set_speed(vl, vr)

def set_speed(vl, vr):
    _, left_motor_handle = vrep.simxGetObjectHandle(client_id, 'LeftMotor#', vrep.simx_opmode_oneshot_wait)
    _, right_motor_handle = vrep.simxGetObjectHandle(client_id, 'RightMotor#', vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(client_id, left_motor_handle, vl, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(client_id, right_motor_handle, vr, vrep.simx_opmode_streaming)

def update_graph(robot_pos, graph, last_vertex):
    vertex = str(robot_pos[0]) + "," + str(robot_pos[1])
    graph.add_vertex(vertex)
    if last_vertex:
        graph.add_edge((vertex, last_vertex))
    last_vertex = vertex
    print(str(graph))
    return last_vertex

def get_ultrassom_values(vrep, client_id, sensor_h):
    sensor_val = np.array([])
    for x in range(1, 16 + 1):
        _, _, detected_point, _, _ = vrep.simxReadProximitySensor(client_id, sensor_h[x-1], vrep.simx_opmode_buffer)                
        sensor_val = np.append(sensor_val, np.linalg.norm(detected_point))
    return sensor_val

def get_sensor_front(sensor_val):
    sens_f_1 = sensor_val[3]
    sens_f_2 = sensor_val[4]
    return sens_f_1, sens_f_2

def get_sensor_back(sensor_val):
    sens_b_1 = sensor_val[11]
    sens_b_2 = sensor_val[12]
    return sens_b_1, sens_b_2

def get_sensor_left(sensor_val):
    sens_l_1 = sensor_val[0]
    sens_l_2 = sensor_val[15]
    return sens_l_1, sens_l_2

def get_sensor_right(sensor_val):
    sens_r_1 = sensor_val[7]
    sens_r_2 = sensor_val[8]
    return sens_r_1, sens_r_2

def print_sensors(sensor, sens_1, sens_2):
    print(str(sensor) + " para: 1 = {:.2f}, 2 = {:.2f}".format(sens_1, sens_2))

def main(client_id_connected, vrep_lib):
    global vrep, client_id
    vrep = vrep_lib
    client_id = client_id_connected

    done_turn = True
    orientation_before = None
    connection = False
    last_vertex = None
    is_there_an_opening_left = False
    is_there_an_opening_right = False
    wall_toggle = False
    open_vertices = []

    # Pegando os handles dos sensores ultrassom
    sensor_h = [] # Atenção! Não utilizar o sensor_h dá um delay para pegar os valores dos sensores
    sensor_val = np.array([]) # Inicializando o array de valores

    graph = Graph()
    for x in range(1, 16 + 1):
        _, sensor_handle = vrep.simxGetObjectHandle(client_id, 'Pioneer_p3dx_ultrasonicSensor' + str(x), vrep.simx_opmode_oneshot_wait)
        sensor_h.append(sensor_handle) 

        _, _, detected_point, _, _ = vrep.simxReadProximitySensor(client_id, sensor_handle, vrep.simx_opmode_streaming)                
        sensor_val = np.append(sensor_val, np.linalg.norm(detected_point))
            
    while True:
        sensor_val = get_ultrassom_values(vrep, client_id, sensor_h)

        sens_f_1, sens_f_2 = get_sensor_front(sensor_val)

        #sens_b_1, sens_b_2 = get_sensor_back(sensor_val)

        #print_sensors("Frente", sens_f_1, sens_f_2)
        #print_sensors("Atras", sens_b_1, sens_b_2)

        robot_pos = get_object_pos('Pioneer_p3dx')

        if is_far_enough(sens_f_1, sens_f_2) and done_turn:
            set_speed(1, 1)
            orientation_before = robot_pos[2]
            target_before = 10

            sens_l_1, sens_l_2 = get_sensor_left(sensor_val)
            is_there_an_opening_left = (sens_l_1 < 0.02 or sens_l_1 > 0.6) and (sens_l_2 < 0.02 or sens_l_2 > 0.6)

            sens_r_1, sens_r_2 = get_sensor_right(sensor_val)
            is_there_an_opening_right = (sens_r_1 < 0.02 or sens_r_1 > 0.6) and (sens_r_2 < 0.02 or sens_r_2 > 0.6)

            if (is_there_an_opening_left or is_there_an_opening_right) and not wall_toggle:
                last_vertex = update_graph(robot_pos, graph, last_vertex)
                open_vertices.append(last_vertex)
                wall_toggle = True
            elif not (is_there_an_opening_left or is_there_an_opening_right):
                wall_toggle = False

            #print_sensors("Esquerda", sens_l_1, sens_l_2)
            #print_sensors("Direita", sens_r_1, sens_r_2)
        else:
            orientation_now = robot_pos[2]

            if abs(abs(orientation_before) - abs(orientation_now)) < 0.001:
                connection = True
            done_turn,target_before = turn(sens_l_1,sens_l_2,sens_r_1,sens_r_2,orientation_before,orientation_now,target_before)

        if connection:
            connection = False
            last_vertex = update_graph(robot_pos, graph, last_vertex)
    
    #time.sleep(0.01) # Loop executa numa taxa de 20 Hz