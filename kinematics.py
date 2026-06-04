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
    return robot.fkine(joints).A



def inverse_kinematics(Tep_target, current_joints):
    return robot.ets().ikine_LM(
    Tep=Tep_target,
    q0=current_joints,  # Start searching right where the robot is standing
    ilimit=30,                # Number of gradient steps allowed
    slimit=1,                 # CRITICAL: Disables the random reposition restarts
    tol=1e-6,                 # Precision tolerance
    joint_limits=True,        # Keep things constrained safely inside limits
    method="sugihara",
    
    # --- The "Heavy Weights" Strategy ---
    kq=10.0,                  # High gain forcing optimization into the null space
    ps=0.05,                  # Sharp safety barrier buffer near structural bounds
    pi=0.8                    # Kicks the tracking priority into hyperdrive early
)


