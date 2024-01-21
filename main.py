from DaHuanFingerControl.ControlGripper import SetCmd
from DaHuanFingerControl.ControlRoot import ControlRoot
from UR_control.URControl import URControl as Robot
from UR_control.URControl import rpy2rot_vec
import numpy as np
import time

robot = Robot()
tcp = rpy2rot_vec((np.pi / 2, 0, np.pi / 2))
tcp2 = rpy2rot_vec((np.pi / 2, -np.pi / 2, np.pi / 2))

CR = ControlRoot()
cs = SetCmd(CR)
cs.HandInit()
cs.InitFeedback()
human_pose = [-0.25, -0.7562937886520618, 0.4550195419732867]


def robot_moveL_with_static_rpy(pose, x, z, finger_pos):
    robot.control_c.moveL((pose[0], pose[1], pose[2], tcp[0], tcp[1], tcp[2]), speed=0.03,
                          acceleration=0.1, asynchronous=False)
    cs.Position(1000)
    time.sleep(0.5)
    robot.control_c.moveL((x, pose[1], pose[2], tcp[0], tcp[1], tcp[2]), speed=0.03,
                          acceleration=0.1, asynchronous=False)
    cs.Position(finger_pos)
    time.sleep(2)
    robot.control_c.moveL((x, pose[1], z, tcp[0], tcp[1], tcp[2]), speed=0.03,
                          acceleration=0.1, asynchronous=False)
    robot.control_c.moveL((pose[0], pose[1], z, tcp[0], tcp[1], tcp[2]), speed=0.03,
                          acceleration=0.1, asynchronous=False)
    # cs.Position(1000)
    robot.control_c.moveL((human_pose[0], human_pose[1], human_pose[2], tcp[0], tcp[1], tcp[2]), speed=0.03,
                          acceleration=0.1, asynchronous=False)
    time.sleep(2)


init_pose = [-0.18403260153599, -0.7062937886520618, 0.5050195419732867]

robot.control_c.moveL((init_pose[0], init_pose[1], init_pose[2], tcp[0], tcp[1], tcp[2]), speed=0.03,
                      acceleration=0.1, asynchronous=False)
x_init = -0.25
pose1 = [x_init, -0.48381472961018573, 0.4587295304342077, ]
# -0.06340571481115573, -0.48381472961018573, 0.4587295304342077,
# -0.0866444379333002, -0.48381713660980097, 0.47898841330847836
# pose = 200
robot_moveL_with_static_rpy(pose1, x=-0.06340571481115573, z=0.47898841330847836, finger_pos=200)
# exit()
pose2 = [x_init, -0.6573053787678614, 0.6014474654541312]
# -0.04708070453119846, -0.6573053787678614, 0.6014474654541312
# [-0.04707429604645738, -0.6572898389610627, 0.649748684720083,
# pose = 500
robot_moveL_with_static_rpy(pose2, x=-0.04708070453119846, z=0.649748684720083, finger_pos=500)
exit()

pose3 = [-0.10403260153599, -0.7062937886520618, 0.5050195419732867 + 0.05, ]
#
robot_moveL_with_static_rpy(pose2)
pose4 = [-0.10403260153599, -0.7062937886520618, 0.5050195419732867 + 0.05, ]


robot.control_c.moveL((init_pose[0], init_pose[1], init_pose[2], tcp[0], tcp[1], tcp[2]), speed=0.03,
                      acceleration=0.1, asynchronous=False)
