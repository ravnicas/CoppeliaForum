# This example illustrates how to use the path/motion
# planning functionality from a remote API client.
#
# Load the demo scene 'motionPlanningServerDemo.ttt' in CoppeliaSim 
# then run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

import sim

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,-500000,5) # Connect to CoppeliaSim, set a very large time-out for blocking commands
if clientID!=-1:
    print ('Connected to remote API server')

    emptyBuff = bytearray()

    # Start the simulation:
    sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot_wait)

    # Load a robot instance:    res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'loadRobot',[],[0,0,0,0],['d:/coppeliaRobotics/qrelease/release/test.ttm'],emptyBuff,sim.simx_opmode_oneshot_wait)
    #    robotHandle=retInts[0]
    
    # Retrieve some handles:
    res,robotHandle=sim.simxGetObjectHandle(clientID,'IRB4600#',sim.simx_opmode_oneshot_wait)
    res,target1=sim.simxGetObjectHandle(clientID,'testPose1#',sim.simx_opmode_oneshot_wait)
    res,target2=sim.simxGetObjectHandle(clientID,'testPose2#',sim.simx_opmode_oneshot_wait)
    res,target3=sim.simxGetObjectHandle(clientID,'testPose3#',sim.simx_opmode_oneshot_wait)
    res,target4=sim.simxGetObjectHandle(clientID,'testPose4#',sim.simx_opmode_oneshot_wait)

    # Retrieve the poses (i.e. transformation matrices, 12 values, last row is implicit) of some dummies in the scene
    res,retInts,target1Pose,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'getObjectPose',[target1],[],[],emptyBuff,sim.simx_opmode_oneshot_wait)
    res,retInts,target2Pose,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'getObjectPose',[target2],[],[],emptyBuff,sim.simx_opmode_oneshot_wait)
    res,retInts,target3Pose,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'getObjectPose',[target3],[],[],emptyBuff,sim.simx_opmode_oneshot_wait)
    res,retInts,target4Pose,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'getObjectPose',[target4],[],[],emptyBuff,sim.simx_opmode_oneshot_wait)

    # Get the robot initial state:
    res,retInts,robotInitialState,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'getRobotState',[robotHandle],[],[],emptyBuff,sim.simx_opmode_oneshot_wait)
    
    # Some parameters:
    approachVector=[0,0,1] # often a linear approach is required. This should also be part of the calculations when selecting an appropriate state for a given pose
    maxConfigsForDesiredPose=10 # we will try to find 10 different states corresponding to the goal pose and order them according to distance from initial state
    maxTrialsForConfigSearch=300 # a parameter needed for finding appropriate goal states
    searchCount=2 # how many times OMPL will run for a given task
    minConfigsForPathPlanningPath=400 # interpolation states for the OMPL path
    minConfigsForIkPath=100 # interpolation states for the linear approach path
    collisionChecking=1 # whether collision checking is on or off

    # Do the path planning here (between a start state and a goal pose, including a linear approach phase):
    inInts=[robotHandle,collisionChecking,minConfigsForIkPath,minConfigsForPathPlanningPath,maxConfigsForDesiredPose,maxTrialsForConfigSearch,searchCount]
    inFloats=robotInitialState+target1Pose+approachVector
    res,retInts,path,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,'remoteApiCommandServer',sim.sim_scripttype_childscript,'findPath_goalIsPose',inInts,inFloats,[],emptyBuff,sim.simx_opmode_oneshot_wait)
    
