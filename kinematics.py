import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH

# 1. Define the Robot
mm = 1e-3
robot = DHRobot([
    RevoluteDH(a=0*mm,       alpha=0,        d=287.87*mm, offset=0),
    RevoluteDH(a=20.174*mm,  alpha=-np.pi/2, d=0,         offset=-np.pi/2),
    RevoluteDH(a=260.986*mm, alpha=0,        d=0,         offset=0),
    RevoluteDH(a=19.219*mm,  alpha=0,        d=260.753*mm, offset=0),
    RevoluteDH(a=0*mm,       alpha=np.pi/2,  d=0,         offset=0),
    RevoluteDH(a=0*mm,       alpha=-np.pi/2, d=74.745*mm, offset=np.pi)
])


def forward_kinematics(joints):
    return robot.fkine(joints)



def inverse_kinematics(trasform, joints):
    return robot.ikine_LM(trasform, q0=joints)


