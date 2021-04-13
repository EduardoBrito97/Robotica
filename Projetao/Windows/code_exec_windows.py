import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

import signal
import time
import sim as vrep
import codigo_principal

client_id = -1

def run_program():
    vrep.simxFinish(-1)
    
    global client_id

    client_id = vrep.simxStart('127.0.0.1', 19997, True, True, 2000, 5)

    if client_id != -1:
        print('Connected to remote API server')
    else:
        print('Connection not successful')
        sys.exit('Could not connect')

    vrep.simxStartSimulation(client_id, vrep.simx_opmode_oneshot_wait)

    codigo_principal.main(client_id, vrep)

# Dando um override no ctrl c pra parar a simulação antes de parar o script
def exit_gracefully(signum, frame):
    # restore the original signal handler as otherwise evil things will happen
    signal.signal(signal.SIGINT, original_sigint)

    vrep.simxPauseSimulation(client_id, vrep.simx_opmode_oneshot_wait)
    vrep.simxFinish(client_id)
    sys.exit(1)

    # restore the exit gracefully handler here
    signal.signal(signal.SIGINT, exit_gracefully)

if __name__ == '__main__':
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exit_gracefully)
    run_program()