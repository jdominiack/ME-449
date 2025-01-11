import modern_robotics as mr
import numpy as np
import logging
import os

#sets up logging
logger = logging.getLogger('CapstoneLog')
logging.basicConfig(filename='Dominiack_Joshua_best_log.log', encoding='utf-8', level=logging.INFO)

M = np.array([[1, 0, 0,  0.033],
              [0, 1, 0,      0],
              [0, 0, 1, 0.6546],
              [0, 0, 0,      1]])

B1 = np.array([0,0,1,0,0.033,0])
B2 = np.array([0,-1,0,-0.5076,0,0])
B3 = np.array([0,-1,0,-0.3526,0,0])
B4 = np.array([0,-1,0,-0.2176,0,0])
B5 = np.array([0,0,1,0,0,0])

l = 0.235
w = 0.15
r = 0.0475

#Calculates the psuedo-inverse of the H matrix
F = r/4 * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                    [1, 1, 1, 1],
                    [-1, 1, -1, 1]])

F6 = np.array([[0, 0, 0, 0],
               [0, 0, 0, 0]])
F6 = np.vstack((F6, F))
F6 = np.vstack((F6, np.array([[0,0,0,0]])))

#Function Parameters:
InitConfig = [0.3, -0.2, -0.4, 0, -0.25, -0.7, 0, 0, 0, 0, 0, 0, 0]

TseInitial = np.array([[0, 0, 1, 0],
                       [0, 1, 0, 0],
                       [-1, 0, 0, 0.5],
                       [0, 0, 0, 1]])

TscInitial = np.array([[1, 0, 0, 1],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0.025],
                       [0, 0, 0, 1]])

TscFinal = np.array([[0, 1, 0, 0],
                     [-1, 0, 0, -1],
                     [0, 0, 1, 0.025],
                     [0, 0, 0, 1]])

Kp = np.eye(6) * 3
Ki = np.eye(6) * 0.6

#Primary log output for new run and inputs
logger.info(
    f"Program '{os.path.basename(__file__)}' called with inputs:\n"
    f"  Tsci:\n'{TscInitial}'\n"
    f"  Tscf:\n'{TscFinal}'\n"
    f"  InitConfig:\n'{InitConfig}'\n"
    f"  TseInitial:\n'{TseInitial}'\n"
    f"  Kp:\n'{Kp}'\n"
    f"  Ki:\n'{Ki}'"
)

def MobileManipulator(TscInitial, TscFinal, InitConfig, TseInitial, Kp, Ki):

    dt = 0.01

    TceGrasp = np.array([[0, 0, 1, 0],
                         [0, 1, 0, 0],
                         [-1, 0, 0, 0.01],
                         [0, 0, 0, 1]])

    TceStand = np.array([[0, 0, 1, 0],
                         [0, 1, 0, 0],
                         [-1, 0, 0, 0.075],
                         [0, 0, 0, 1]])
    
    k=1

    #Trajectory generation
    logger.info('Generating Trajectories')
    refTrajectory = TrajectoryGenerator(TseInitial, TscInitial, TscFinal, TceGrasp, TceStand, k)

    currentState = InitConfig

    #Initializes arrays for to record csv file info
    robotStates = np.zeros(np.shape(refTrajectory))
    robotStates[0,:] = InitConfig 

    errorLog = np.zeros((6, np.shape(refTrajectory)[0]))

    #Initializes the feedback controller integral term
    Itot = [0, 0, 0, 0, 0, 0]

    logger.info('Initiating feedback control loop planning')
    for i in range(np.shape(refTrajectory)[0]-1):

        desiredConfig = refTrajectory[i, :]
        nextDesiredConfig = refTrajectory[i+1, :]

        #Takes the configs and turns them into Tse transform mats
        X = ConfigtoTse(currentState)
        Xd = TrajtoTse(desiredConfig)
        Xdnext = TrajtoTse(nextDesiredConfig)

        #Feedback control
        controls, error, Itot = FeedbackControl(X, Xd, Xdnext, currentState, Kp, Ki, Itot, dt)

        #Gets the next state based on controls, appends grabber state from trajs 
        nextState = NextState(currentState, controls, dt, 15)
        nextState = np.append(nextState, refTrajectory[i][-1])

        #Stores csv info for iteration
        robotStates[i+1, :] = nextState
        errorLog[:,i] = np.round(error, 4).T

        currentState = nextState

    return robotStates, errorLog

#Converts config in the form phi, x, y ... to a Tse mat
def ConfigtoTse(c):

    Tsb = np.array([[np.cos(c[0]), -np.sin(c[0]), 0,   c[1]],
                    [np.sin(c[0]),  np.cos(c[0]), 0,   c[2]],
                    [           0,             0, 1, 0.0963],
                    [           0,             0, 0,      1]])
    
    Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0,      0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0,      1]])
    
    jointAngles = c[3:8]

    T0e = PoE(jointAngles)

    Tse = Tsb @ Tb0 @ T0e

    return Tse

#Calculates product of exp given arm joint angles
def PoE(thetaList):
    e1 = mr.MatrixExp6(mr.VecTose3(B1)*thetaList[0])
    e2 = mr.MatrixExp6(mr.VecTose3(B2)*thetaList[1])
    e3 = mr.MatrixExp6(mr.VecTose3(B3)*thetaList[2])
    e4 = mr.MatrixExp6(mr.VecTose3(B4)*thetaList[3])
    e5 = mr.MatrixExp6(mr.VecTose3(B5)*thetaList[4])
    T = M @ e1 @ e2 @ e3 @ e4 @ e5
    return T

#Converts config in the form r11, r12, r13 ... to a Tse mat
def TrajtoTse(t):
    Tse = np.array([[t[0], t[1], t[2], t[9]],
                    [t[3], t[4], t[5], t[10]],
                    [t[6], t[7], t[8], t[11]],
                    [   0,    0,    0,    1]])
    return Tse


def TrajectoryGenerator(TseInitial, TscInitial, TscFinal, TceGrasp, TceStand, k):

    trajList = []

    #Traj 1: Initial config to standoff config
    TseStand = TscInitial @ TceStand
    trajList.append(mr.ScrewTrajectory(TseInitial, TseStand, 6, 6/(k*0.01), 3))

    #Traj 2: Move the gripper down to grasp position
    TseGrasp = TscInitial @ TceGrasp
    trajList.append(mr.CartesianTrajectory(TseStand, TseGrasp, 1, 1/(k*0.01), 3))

    #Traj 3: Closing the gripper
    trajList.append(1)

    #Traj 4: Move gripper back to standoff config
    trajList.append(mr.CartesianTrajectory(TseGrasp, TseStand, 1, 1/(k*0.01), 3))

    #Traj 5: Move to standoff config above final config
    TseFinalStand = TscFinal @ TceStand
    trajList.append(mr.ScrewTrajectory(TseStand, TseFinalStand, 7, 7/(k*0.01), 3))

    #Traj 6: Move gripper to final configuration
    TseFinalGrasp = TscFinal @ TceGrasp
    trajList.append(mr.CartesianTrajectory(TseFinalStand, TseFinalGrasp, 1, 1/(k*0.01), 3))

    #Traj 7: Opening the gripper
    trajList.append(0)

    #Traj 8: Move the gripper back to the "standoff" config
    trajList.append(mr.CartesianTrajectory(TseFinalGrasp, TseFinalStand, 1, 1/(k*0.01), 3))

    output = np.empty((0,13))
    clawOpen = True
    
    for i in trajList:
       

        if i == 1:
            #gets the last row of the output and chages the grabber state to closed
            grabStatus = output[-1]
            grabStatus[-1] = 1

            #copies the last row 100 times with the new grabber state to give the grabber time to close
            for j in range(int(1/(k*0.01))): 
                 output = np.vstack([output, grabStatus])
            clawOpen = False

        elif i == 0:
            #gets the last row of the output and chages the grabber state to open
            openStatus = output[-1]
            openStatus[-1] = 0

            #copies the last row 100 times with the new grabber state to give the grabber time to open
            for j in range(int(1/(k*0.01))): 
                 output = np.vstack([output, openStatus])
            clawOpen = True

        else:

            #iterates through each trajectory's SE3 matrices and adds them to the output
            for j in i:
                    output = np.vstack([output, getRow(j, clawOpen)])

    return output

#Converts T mats to the desired row format
def getRow(Mat, open):

    if open == True:
        row = [Mat[0, 0], Mat[0, 1], Mat[0, 2], Mat[1, 0], Mat[1,1], Mat[1,2], Mat[2,0], Mat[2,1], Mat[2,2], Mat[0,3], Mat[1, 3], Mat[2, 3], 0]
    else:
        row = [Mat[0, 0], Mat[0, 1], Mat[0, 2], Mat[1, 0], Mat[1,1], Mat[1,2], Mat[2,0], Mat[2,1], Mat[2,2], Mat[0,3], Mat[1, 3], Mat[2, 3], 1]

    return row


def FeedbackControl(X, Xd, Xdnext, q, Kp, Ki, Itot, dt):

    #Finds ee error from desired position
    XerrMat = mr.MatrixLog6(mr.TransInv(X) @ Xd)
    Xerror = mr.se3ToVec(XerrMat)

    #Takes in the summed integral term parameter, adds error*dt
    Integral = Itot+Xerror*dt

    #Finds desired twist in ee frame
    VdMat = (1/dt) * mr.MatrixLog6(mr.TransInv(Xd) @ Xdnext)
    Vd = mr.se3ToVec(VdMat)

    #Feedforward + PI control iteration step, calcs next ee twist
    V = mr.Adjoint(mr.TransInv(X) @ Xd) @ Vd + Kp@Xerror + Ki@Integral

    #Finds T0e for jacobian calculation 
    Tsb = np.array([[np.cos(q[0]), -np.sin(q[0]), 0, q[1]],
                    [np.sin(q[0]),  np.cos(q[0]), 0, q[2]],
                    [           0,             0, 1, 0.0963],
                    [           0,             0, 0, 1]])
    
    Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0,      0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0,      1]])
    
    Ts0 = Tsb @ Tb0
    Tse = X
    T0e = mr.TransInv(Ts0) @ Tse

    jointAngles = q[3:8]

    #Calculates jacobian for robot arm
    Jarm = Jb(jointAngles)

    #Calculates jacobian for the robot chassis
    Jbase = mr.Adjoint(mr.TransInv(T0e) @ mr.TransInv(Tb0)) @ F6

    #Combines Jarm and Jbase 
    Je = np.hstack((Jbase, Jarm))
 
    #Takes Je pseudo-inverse and calculates control velocities
    qdot = np.linalg.pinv(Je, 1e-4) @ V
    qdot = np.round(qdot, 1)

    #outputs control velocities, ee error, and the current integral term
    return qdot, Xerror, Integral

#Calculates body jacobian from joint angles
def Jb(thetaList):
    e1 = mr.MatrixExp6(-mr.VecTose3(B1)*thetaList[0])
    e2 = mr.MatrixExp6(-mr.VecTose3(B2)*thetaList[1])
    e3 = mr.MatrixExp6(-mr.VecTose3(B3)*thetaList[2])
    e4 = mr.MatrixExp6(-mr.VecTose3(B4)*thetaList[3])
    e5 = mr.MatrixExp6(-mr.VecTose3(B5)*thetaList[4])
    
    Jb1 = mr.Adjoint(e5 @ e4 @ e3 @ e2) @ B1
    Jb2 = mr.Adjoint(e5 @ e4 @ e3) @ B2
    Jb3 = mr.Adjoint(e5 @ e4) @ B3
    Jb4 = mr.Adjoint(e5) @ B4
    Jb5 = B5

    return np.array([Jb1, Jb2, Jb3, Jb4, Jb5]).T

#Finds next state of system from current state and commanded vel
def NextState(q, qdot, dt, maxAngVel):

    #Velocity limiting
    for i in range(len(qdot)):
        if qdot[i] > maxAngVel:
            qdot[i] = maxAngVel
        elif qdot[i] < -maxAngVel:
            qdot[i] = -maxAngVel
    
    theta = q[3:8]
    wheelAngles = q[8:12]
    phi = q[0]
    x = q[1]
    y = q[2]

    thdot = qdot[4:10]
    u = qdot[0:4]

    #Calculates new states assuming constant velocity for dt sec
    thetaNew = theta + thdot*dt
    newWheels = wheelAngles + u*dt

    #Uses pseudo-inverse of H matrix to find b frame twist, calculates odometry
    Vb = F @ u*dt

    Vb6 = np.array([0, 0, Vb[0], Vb[1], Vb[2], 0])
    
    Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                    [np.sin(phi),  np.cos(phi), 0, y],
                    [          0,            0, 1, 0.0963],
                    [          0,            0, 0, 1]])

    Tbbnew = mr.MatrixExp6(mr.VecTose3(Vb6))

    Tsbnew = Tsb @ Tbbnew

    phiNew = np.arccos(Tsbnew[0,0])
    xNew = Tsbnew[0, 3]
    yNew = Tsbnew[1, 3]

    
    nextState = np.array([phiNew, xNew, yNew])
    nextState = np.append(nextState, thetaNew)
    nextState = np.append(nextState, newWheels)

    return nextState

#Calls primary function
result = MobileManipulator(TscInitial, TscFinal, InitConfig, TseInitial, Kp, Ki)

logger.info('Generating state csv.')
np.savetxt("robotStates.csv", result[0], delimiter = ",")

logger.info('Generating error csv.')
np.savetxt("robotError.csv", result[1], delimiter = ",")

logger.info('Done.')