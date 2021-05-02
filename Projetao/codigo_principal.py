import sys
import time
import numpy as np
import math
import matplotlib as mpl
import logging
from graph import Graph
from enum import Enum
from scipy.spatial.distance import euclidean

vrep = None
client_id = None
PI = math.pi

L = 0.33
last_detected_lim = 7

class State(Enum):
    FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    ENDPOINT_RETURN = 4
    DEBUG = 5

def to_180_range(angle):
    angle = math.fmod(angle,2*PI)
    if(angle < -PI):
        angle += (2*PI)
    elif(angle> PI):
        angle -= (2*PI)
    return angle

def is_far_enough(sens_1, sens_2):
    lim = 0.4
    if(sens_1 < 0.01):
        return sens_2 > lim or sens_2 < 0.01
    elif(sens_2 < 0.01):
        return sens_1 > lim or sens_1 < 0.01 
    
    mean = (sens_1 + sens_2)/2
    return mean > lim

def is_between_walls(sens_l_1, sens_l_2, sens_r_1, sens_r_2):
    return sens_l_1 and sens_l_2 and sens_r_1 and sens_r_2

def turn(sens_l_1, sens_l_2, sens_r_1, sens_r_2, orientation_before, orientation_now, target_before, prior_left = True):
    k_w = 0.33
    vl = 0
    vr = 0

    orientation_now = to_180_range(orientation_now)
    orientation_before = to_180_range(orientation_before)

    if prior_left:
        if not sens_l_1 and not sens_l_2:
            vl = -1
            vr = 1
        elif not sens_r_1 and not sens_r_2:
            vl = 1
            vr = -1
    else:
        if not sens_r_1 and not sens_r_2:
            vl = 1
            vr = -1
        elif not sens_l_1 and not sens_l_2:
            vl = -1
            vr = 1

    target = abs(abs(orientation_before) - abs(orientation_now)) - (PI/2)

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

def set_target_pos(target_pos):
    _, target_handle = vrep.simxGetObjectHandle(client_id, "Target#", vrep.simx_opmode_oneshot_wait)
    _, target_pos_orig = vrep.simxGetObjectPosition(client_id, target_handle, -1, vrep.simx_opmode_streaming)
    target_pos = (target_pos[0], target_pos[1], target_pos_orig[2]) # Adicionamos o z do target para não mudar a altura

    vrep.simxSetObjectPosition(client_id, target_handle, -1, target_pos, vrep.simx_opmode_oneshot_wait)

def move_to_target(target):
    k_p = 0.8
    k_a = 1.4

    robot_pos = get_object_pos('Pioneer_p3dx')
    delta_x = target[0] - robot_pos[0]
    delta_y = target[1] - robot_pos[1]
    ro = ((delta_x ** 2) + (delta_y ** 2)) ** (1/2)

    orientation_now = to_180_range(robot_pos[2])
    alpha = -orientation_now + math.atan2(delta_y, delta_x)

    v = k_p * ro
    # Beta não importa, pois não precisamos saber a orientação
    w = (k_a * alpha)

    if abs(w) > 0.01:
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

def update_graph(robot_pos, graph, last_vertex, min_dist = 1.0):
    vertex = (robot_pos[0], robot_pos[1], robot_pos[2])
    if graph.add_vertex(vertex, min_dist):
        if last_vertex: 
            graph.add_edge((vertex, last_vertex))
            graph.add_edge((last_vertex, vertex))
        logging.getLogger("Robot").warning(str(graph))
        last_vertex = vertex
    return last_vertex

def get_ultrassom_values(sensor_h):
    sensor_val = np.array([])
    sensor_detect = []
    for x in range(1, 16 + 1):
        _, was_detected, detected_point, _, _ = vrep.simxReadProximitySensor(client_id, sensor_h[x-1], vrep.simx_opmode_buffer)                
        sensor_val = np.append(sensor_val, np.linalg.norm(detected_point))
        sensor_detect.append(was_detected)
    return sensor_val, sensor_detect

def get_sensors_handlers():
    sensor_h = []
    for x in range(1, 16 + 1):
        _, sensor_handle = vrep.simxGetObjectHandle(client_id, 'Pioneer_p3dx_ultrasonicSensor' + str(x), vrep.simx_opmode_oneshot_wait)
        sensor_h.append(sensor_handle)
        vrep.simxReadProximitySensor(client_id, sensor_handle, vrep.simx_opmode_streaming)
    return sensor_h

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
    if type(sens_1) == float:
        logging.getLogger("Robot").warning(str(sensor) + " para: 1 = {:.2f}, 2 = {:.2f}".format(sens_1, sens_2))
    else:
        logging.getLogger("Robot").warning(str(sensor) + " para: 1 = " + str(sens_1) + ", 2 = " + str(sens_2))

def get_front_mobile_mean(sensor_val, mobile_mean_front_1, mobile_mean_front_2, sensor_detect):
    sens_f_1, sens_f_2 = get_sensor_front(sensor_val)
    if len(mobile_mean_front_1) >= 3:
        mobile_mean_front_1.pop(0)
    if len(mobile_mean_front_2) >= 3:
        mobile_mean_front_2.pop(0)

    detect_f_1, detect_f_2 = get_sensor_front(sensor_detect)
    if detect_f_1:            
        mobile_mean_front_1.append(sens_f_1)
    else:
        mobile_mean_front_1.append(0.8)

    if detect_f_2:            
        mobile_mean_front_2.append(sens_f_2)
    else:
        mobile_mean_front_2.append(0.8)

    sens_f_1 = sum(mobile_mean_front_1)/len(mobile_mean_front_1)
    sens_f_2 = sum(mobile_mean_front_2)/len(mobile_mean_front_2)
    return sens_f_1, sens_f_2

def update_midpoints(last_detected_right, last_detected_lim, detected_right, robot_pos, graph, last_vertex, midpoints, last_detected_left, detected_left):
    if sum(last_detected_right) == last_detected_lim and not detected_right:
        curr_vertex = update_graph(robot_pos, graph, last_vertex)
        if curr_vertex != last_vertex:
            logging.getLogger("Robot").warning('Midpoint right')
            midpoints.append((robot_pos[0], robot_pos[1], robot_pos[2]))
            last_vertex = curr_vertex
            detected_right = True
            if sum(last_detected_left) == last_detected_lim:
                logging.getLogger("Robot").warning('Midpoint left')
                midpoints.append((robot_pos[0], robot_pos[1], robot_pos[2]))
                detected_left = True

    elif sum(last_detected_right) != last_detected_lim:
        detected_right = False

    if sum(last_detected_left) == last_detected_lim and not detected_left:
        curr_vertex = update_graph(robot_pos, graph, last_vertex)
        if curr_vertex != last_vertex:
            logging.getLogger("Robot").warning('Midpoint left')
            midpoints.append((robot_pos[0], robot_pos[1], robot_pos[2]))
            last_vertex = curr_vertex
            detected_left = True
    elif sum(last_detected_left) != last_detected_lim:
        detected_left = False
    return last_vertex

def update_last_detected(last_detected_left, last_detected_lim, last_detected_right, sens_r_1, sens_r_2, sens_l_1, sens_l_2):
    if len(last_detected_left) >= last_detected_lim:
        last_detected_left.pop(0)

    if len(last_detected_right) >= last_detected_lim:
        last_detected_right.pop(0)

    if sens_r_1 and sens_r_2:
        last_detected_right.append(0)
    else:
        last_detected_right.append(1)

    if sens_l_1 and sens_l_2:
        last_detected_left.append(0)
    else:
        last_detected_left.append(1)

def delete_unnecessary_midpoint(robot_pos, graph, last_vertex, midpoints):
    robot_pos = (robot_pos[0], robot_pos[1], robot_pos[2])
    if last_vertex in midpoints and euclidean(robot_pos, last_vertex) < 0.8:
        neighbors = graph._graph_dict[last_vertex]

        neighbor = None
        if neighbors and len(neighbors) > 0:
            neighbor = neighbors[0]

        graph._graph_dict.pop(last_vertex)
        midpoints.pop(-1)

        logging.getLogger("Robot").warning('Midpoint removed: ' + str(last_vertex))
        last_vertex = neighbor
    return last_vertex

def turn_to_target(robot_angle, target_angle):
    k_w = 0.33

    robot_angle = to_180_range(robot_angle)
    target_angle = to_180_range(target_angle)

    target = abs(abs(robot_angle) - abs(target_angle))
    set_speed(-1*k_w*abs(target), 1*k_w*abs(target))

def main(client_id_connected, vrep_lib):
    global vrep, client_id
    vrep = vrep_lib
    client_id = client_id_connected

    orientation_before = None
    last_vertex = None
    midpoints = []
    visited_midpoints = []
    vertex_index = 0
    
    mobile_mean_front_1 = []
    mobile_mean_front_2 = []

    last_detected_right = []
    detected_right = False
    
    last_detected_left = []
    detected_left = False
    
    graph = Graph()
    state = State.FORWARD

    # Atenção! Não utilizar o sensor_h dá um delay para pegar os valores dos sensores
    sensor_h = get_sensors_handlers()
    sensor_val, sensor_detect = get_ultrassom_values(sensor_h)
            
    while True:
        sensor_val, sensor_detect = get_ultrassom_values(sensor_h)
        robot_pos = get_object_pos('Pioneer_p3dx')
        
        if state == State.FORWARD:
            sens_f_1, sens_f_2 = get_front_mobile_mean(sensor_val, mobile_mean_front_1, mobile_mean_front_2, sensor_detect)

            
            state, orientation_before, target_before, sens_l_1, sens_l_2, sens_r_1, sens_r_2, last_vertex = move_forward(sens_f_1,
                                                                                                                        sens_f_2,
                                                                                                                        robot_pos, 
                                                                                                                        graph, 
                                                                                                                        last_vertex, 
                                                                                                                        midpoints, 
                                                                                                                        last_detected_left, 
                                                                                                                        last_detected_right, 
                                                                                                                        sensor_detect, 
                                                                                                                        detected_right, 
                                                                                                                        detected_left,
                                                                                                                        visited_midpoints)

        elif state == State.TURN_LEFT or state == State.TURN_RIGHT:
            orientation_now = robot_pos[2]
            done_turn, target_before = turn(sens_l_1, sens_l_2, sens_r_1, sens_r_2, orientation_before, orientation_now, target_before, prior_left= (state == State.TURN_LEFT))

            if done_turn:
                state = State.FORWARD
                last_detected_right = []
                last_detected_left = []

        elif state == State.ENDPOINT_RETURN:
            if len(midpoints) > 0:
                target = midpoints[-1]

                # Chegamos no objetivo, precisamos rodar para alinhar com o ângulo que estávamos quando chegamos, dobrar e seguir em frente
                if euclidean((robot_pos[0], robot_pos[1]), (target[0], target[1])) <= 0.1:
                    vertex_index, state = midpoint_arrival_treat(robot_pos,
                                                                    target,
                                                                    visited_midpoints,
                                                                    sensor_detect, 
                                                                    midpoints, 
                                                                    vertex_index)

                    # Resetando os valores para dobrar
                    if state == State.TURN_LEFT or state == State.TURN_RIGHT:
                        sens_l_1, sens_l_2 = get_sensor_left(sensor_detect)
                        sens_r_1, sens_r_2 = get_sensor_right(sensor_detect)
                        orientation_before = robot_pos[2]
                        target_before = 10
                        last_vertex = target

                # Ainda não chegamos no objetivo, precisamos andar até lá
                else:
                    state, vertex_index = move_to_next_target(graph, last_vertex, target, vertex_index, robot_pos)
            else:
                logging.getLogger("Robot").warning('There is nowhere to go.')
                set_speed(0, 0)

        elif state == State.DEBUG:
            set_speed(0, 0)
        else:
            logging.getLogger("Robot").warning('State not supported')
            set_speed(0, 0)

def move_forward(sens_f_1, sens_f_2, robot_pos, graph, last_vertex, midpoints, last_detected_left, last_detected_right, sensor_detect, detected_right, detected_left, visited_midpoints):
    state = State.FORWARD

    # sensores laterais contém apenas booleanos (detectou ou não)
    sens_l_1, sens_l_2 = get_sensor_left(sensor_detect)
    sens_r_1, sens_r_2 = get_sensor_right(sensor_detect)
    orientation_before = robot_pos[2]
    target_before = 10

    if not is_far_enough(sens_f_1, sens_f_2):
        state = State.TURN_LEFT
        if is_between_walls(sens_l_1, sens_l_2, sens_r_1, sens_r_2):
            state = State.ENDPOINT_RETURN
            logging.getLogger("Robot").warning("Endpoint")
        else:
            logging.getLogger("Robot").warning("Turnpoint")
            last_vertex = delete_unnecessary_midpoint(robot_pos, graph, last_vertex, midpoints)
        last_vertex = update_graph(robot_pos, graph, last_vertex, min_dist=0.2)

        if not sens_l_1 and not sens_l_2 and not sens_r_1 and not sens_r_2:
            logging.getLogger("Robot").warning("Open Turnpoint")
            midpoints.append(last_vertex)
            visited_midpoints.append(last_vertex)
    else:
        set_speed(1, 1)
        update_last_detected(last_detected_left, last_detected_lim, last_detected_right, sens_r_1, sens_r_2, sens_l_1, sens_l_2)
        last_vertex = update_midpoints(last_detected_right, last_detected_lim, detected_right, robot_pos, graph, last_vertex, midpoints, last_detected_left, detected_left)
    return state, orientation_before, target_before, sens_l_1, sens_l_2, sens_r_1, sens_r_2, last_vertex

def midpoint_arrival_treat(robot_pos, target, visited_midpoints, sensor_detect, midpoints, vertex_index):
    state = State.ENDPOINT_RETURN

    # Chegamos no objetivo, precisamos rodar
    if abs(abs(robot_pos[2]) - abs(target[2])) > 0.01:
        turn_to_target(robot_pos[2], target[2])
    elif target in visited_midpoints:
        state = State.TURN_RIGHT
        logging.getLogger("Robot").warning("Reaching midpoint second time. Turning right.")
    else:
        visited_midpoints.append(target)
        midpoints.pop(-1)
        vertex_index = 0

        state = State.TURN_LEFT
        set_target_pos((-3.325,4.875))
        logging.getLogger("Robot").warning("Arrived target")
    return vertex_index, state

def move_to_next_target(graph, last_vertex, target, vertex_index, robot_pos):
    state = State.ENDPOINT_RETURN

    # Pegamos o caminho mais curto do último vértice (aka de onde começamos a voltar)
    # até a primeira encruzilhada que passamos
    path = graph.get_shortest_path(last_vertex, target)
    if not path or len(path) == 0:
        state = None
        logging.getLogger("Robot").warning("There is no path to: " + str(target))
        return state, vertex_index
    target = path[vertex_index] # agora vamos para o próximo objetivo

    if euclidean((robot_pos[0], robot_pos[1]), (target[0], target[1])) <= 0.1:
        vertex_index += 1
        set_speed(0, 0)
    else:
        set_target_pos(target)
        move_to_target(target)
    return state, vertex_index