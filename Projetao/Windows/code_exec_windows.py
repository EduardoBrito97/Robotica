import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

import sim as vrep
import codigo_principal

codigo_principal.test()

vrep.simxFinish(-1)

clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 2000, 5)

if clientID != -1:
    print('Connected to remote API server')
else:
    print('Connection not successful')
    sys.exit('Could not connect')

vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)
codigo_principal.main(clientID, vrep)