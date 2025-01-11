import modern_robotics as mr
import numpy as np 

'''
Simply running this program will generate the csv file used for my video in your active directory. The function parameters I used are listed below.
'''

#Sets function parameters
TseInitial = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0.5],
                       [0, 0, 0, 1]])

TscInitial = np.array([[1, 0, 0, 1],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0.025],
                       [0, 0, 0, 1]])

TscFinal = np.array([[0, 1, 0, 0],
                     [-1, 0, 0, -1],
                     [0, 0, 1, 0.025],
                     [0, 0, 0, 1]])

TceGrasp = np.array([[0, 0, 1, 0],
                     [0, 1, 0, 0],
                     [-1, 0, 0, 0.01],
                     [0, 0, 0, 1]])

TceStand = np.array([[0, 0, 1, 0],
                     [0, 1, 0, 0],
                     [-1, 0, 0, 0.075],
                     [0, 0, 0, 1]])

k = 1

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

trajectory = TrajectoryGenerator(TseInitial, TscInitial, TscFinal, TceGrasp, TceStand, k)
    

np.savetxt("milestone2.csv", trajectory, delimiter = ",")