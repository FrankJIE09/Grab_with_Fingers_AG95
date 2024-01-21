# 主要功能是控制机器人进行运动控制。
# 程序中使用了yaml模块读取配置文件，rtde_control控制机器人的移动和姿态控制，rtde_io控制机器人的数字输入输出，rtde_receive接收来自机器人的实时数据。
# FTReading模块用于读取机器人的力矩传感器数据。
# 该程序中定义了URControl类，其中包含机器人的IP地址、连接控制器、接收器、IO接口和力矩传感器读取接口等。get_robot()函数用于返回机器人本身，方便其他程序调用。
# Created by Jie Yu
import rtde_control
import rtde_io
import rtde_receive

from scipy.spatial.transform import Rotation as Rt


# 姿态数据:由UR使用的rot_vec转换为rpy
def rot_vec2rpy(pose):
    r = Rt.from_rotvec(pose[3:6])
    rpy = r.as_euler('xyz', degrees=False)
    return rpy


# 姿态数据:由rpy转换为UR使用的rot_vec
def rpy2rot_vec(rpy):
    r = Rt.from_euler('xyz', rpy)
    rot_vec = r.as_rotvec()
    return rot_vec




class URControl:
    def __init__(self):
        IP = '192.168.1.10'
        self.control_c = rtde_control.RTDEControlInterface(IP)
        self.receive_r = rtde_receive.RTDEReceiveInterface(IP)
        self.io_control = rtde_io.RTDEIOInterface(IP)
        return

    def get_robot(self):
        return self


if __name__ == '__main__':
    import numpy as np

    robot = URControl()
    pose = robot.receive_r.getActualTCPPose()
    print(pose)
    r = Rt.from_rotvec(pose[3:6])
    print(rot_vec2rpy(pose))
    # tcp = rpy2rot_vec((-np.pi-np.pi/2, 0, 0))
    tcp = rpy2rot_vec((np.pi/2, -np.pi/2, np.pi/2))
    # -0.016139876756485903, -0.8102090933335253, 0.3722249608780756
    # init_pose = -0.10403260153599, -0.7062937886520618, 0.5050195419732867,
    #
    # robot.control_c.moveL((init_pose[0], init_pose[1], init_pose[2], tcp[0], tcp[1], tcp[2]), speed=0.03,
    #                       acceleration=0.1, asynchronous=False)
