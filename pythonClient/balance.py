# Make sure to have the server side running in CoppeliaSim:
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')
    sim.simxSynchronous(clientID,False)
    ret = sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    time.sleep(2)
    ret = sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)
    print('started simulation')
    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    ret, motor_hand = sim.simxGetObjectHandle(clientID, "motor",    sim.simx_opmode_blocking)
    ret, cyl_hand   = sim.simxGetObjectHandle(clientID, "Cylinder", sim.simx_opmode_blocking)
    ret, rob_hand   = sim.simxGetObjectHandle(clientID, "Cuboid",   sim.simx_opmode_blocking)

    # Now retrieve streaming data (i.e. in a non-blocking fashion):
    startTime=time.time()
    ret, [i,j,k] = sim.simxGetObjectOrientation(clientID, rob_hand, -1, sim.simx_opmode_streaming) # Initialize streaming
    ret, [x,y,z] = sim.simxGetObjectPosition(clientID, cyl_hand, -1, sim.simx_opmode_streaming) # Initialize streaming
    speed=50
    a = 1
    b = 10
    while time.time()-startTime < 100:
        ret, [i,j,k]=sim.simxGetObjectOrientation(clientID, rob_hand, -1, sim.simx_opmode_buffer) # Initialize streaming
        ret, [x,y,z]=sim.simxGetObjectPosition(clientID, rob_hand, -1, sim.simx_opmode_buffer) # Initialize streaming
        if ret == sim.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
            print(y)
            torq = 0*j + b*y
            torq = max(-0.01,torq)
            torq = min(0.01,torq)
            sim.simxSetJointForce(clientID, motor_hand, abs(torq), sim.simx_opmode_oneshot)
            print(torq)
            if(torq>0): speed = 900
            elif(torq<0): speed = -900
            else: speed = 0
            sim.simxSetJointTargetVelocity(clientID, motor_hand, speed, sim.simx_opmode_oneshot)
        time.sleep(0.01)

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
