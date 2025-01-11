import modern_robotics as mr
import numpy as np

B1 = np.array([0,0,1,0,0.033,0])
B2 = np.array([0,-1,0,-0.5076,0,0])
B3 = np.array([0,-1,0,-0.3526,0,0])
B4 = np.array([0,-1,0,-0.2176,0,0])
B5 = np.array([0,0,1,0,0,0])

l = 0.235
w = 0.15
r = 0.0475

F = r/4 * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                    [1, 1, 1, 1],
                    [-1, 1, -1, 1]])

F6 = np.array([[0, 0, 0, 0],
               [0, 0, 0, 0]])
F6 = np.vstack((F6, F))
F6 = np.vstack((F6, np.array([[0,0,0,0]])))

q = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0.2, -1.6, 0])
thetaList = np.array([0, 0, 0.2, -1.6, 0])

Tb0 = np.array([[1,0,0,0.1662],
                [0,1,0,0],
                [0,0,1,0.0026],
                [0,0,0,1]])

X = np.array([[ 0.170, 0, 0.985, 0.387],
              [     0, 1,     0,     0],
              [-0.985, 0, 0.170, 0.570],
              [     0, 0,     0,     1]])

Xd = np.array([[ 0, 0, 1, 0.5],
               [ 0, 1, 0,   0],
               [-1, 0, 0, 0.5],
               [ 0, 0, 0,   1]])

Xdnext = np.array([[ 0, 0, 1, 0.6],
                   [ 0, 1, 0,   0],
                   [-1, 0, 0, 0.3],
                   [ 0, 0, 0,   1]])

Kp = np.zeros((6,6))
Ki = np.zeros((6,6))

dt = 0.01

def FeedbackControl(X, Xd, Xdnext, Kp, Ki, dt):

    XerrMat = mr.MatrixLog6(mr.TransInv(X) @ Xd)
    Xerror = mr.se3ToVec(XerrMat)

    Ierror = [0, 0, 0, 0, 0, 0]
    Ierror += Xerror*dt

    print(np.shape(Xerror))
    print(Ierror)

    VdMat = (1/dt) * mr.MatrixLog6(mr.TransInv(Xd) @ Xdnext)

    print('Vd:')
    Vd = mr.se3ToVec(VdMat)
    print(Vd)

    print('V:')
    V = mr.Adjoint(mr.TransInv(X) @ Xd) @ Vd + Kp@Xerror + Ki@Ierror
    print(mr.Adjoint(mr.TransInv(X) @ Xd) @ Vd)
    print(V)

    print('Xerror:')
    print(Xerror)

    Tsb = np.array([[np.cos(q[0]), -np.sin(q[0]), 0, q[1]],
                    [np.sin(q[0]),  np.cos(q[0]), 0, q[2]],
                    [           0,             0, 1, 0.0963],
                    [           0,             0, 0, 1]])
    
    Ts0 = Tsb @ Tb0
    Tse = X

    T0e = mr.TransInv(Ts0) @ Tse

    Jarm = Jb(thetaList)
    Jbase = mr.Adjoint(mr.TransInv(T0e) @ mr.TransInv(Tb0)) @ F6

    print('Je:')
    Je = np.hstack((Jbase, Jarm))
    print(Je)

    print('qdot:')
    qdot = np.linalg.pinv(Je, 1e-4) @ V
    qdot = np.round(qdot, 1)
    print(qdot)

    return qdot


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

result = FeedbackControl(X, Xd, Xdnext, Kp, Ki, dt)