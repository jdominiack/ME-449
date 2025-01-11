import modern_robotics as mr
import numpy as np

#q = chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state

l = 0.235
w = 0.15
r = 0.0475

F = r/4 * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                    [1, 1, 1, 1],
                    [-1, 1, -1, 1]])

q = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

#Change this to be u then thdot i think
qdot = np.array([0, 0, 0, 0, 0, 10, 10, 10, 10])

def NextState(q, qdot, dt, maxAngVel):

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

    thdot = qdot[0:5]
    u = qdot[5:9]

    thetaNew = theta + thdot*dt
    newWheels = wheelAngles + u*dt

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
    nextState = np.append(nextState, 0)

    return nextState

result = np.append(q, 0)

for i in range(100):
    newq = NextState(q, qdot, 0.01, 10)
    result = np.vstack((result, newq))
    q = newq

print(result[-1, :])

np.savetxt("mile1.csv", result, delimiter = ",")


    


    