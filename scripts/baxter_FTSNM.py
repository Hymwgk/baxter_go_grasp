#!/usr/bin/env python
#coding=utf-8
"""辛建斌老师的一些实验 
    moveit_ik_demo.py - Version 0.1 2014-01-14
    
    Use inverse kinemtatics to move the end effector to a specified pose
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyleft (c) 2014 Patrick Goebel.  All lefts reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
import tf
import math
import numpy as np
from math import pi
import copy

import copy
from moveit_msgs.msg import RobotTrajectory,DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import os
import time

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler,quaternion_multiply,quaternion_from_matrix
#from autolab_core import RigidTransform,transformations
#from pyquaternion import Quaternion
#os.system(". /home/wgk/catkin_ws/baxter.sh")
import pickle
import os
import argparse

#解析命令行参数
parser = argparse.ArgumentParser(description='Baxter FTSNM')
parser.add_argument('--arm', type=str, default='r')
parser.add_argument('--mode', type=str, default='generate')

args = parser.parse_args()


def move_to_target(movegroup,target_L,target_R,lable='Next'):

    left_current_pose = movegroup.get_current_pose(end_effector_link='left_gripper').pose
    right_current_pose = movegroup.get_current_pose(end_effector_link='right_gripper').pose

    dis_diff_L =np.linalg.norm(np.array([left_current_pose.position.x - target_L.position.x,
                        left_current_pose.position.y - target_L.position.y,
                        left_current_pose.position.z - target_L.position.z]))
    dis_pose_L =np.linalg.norm(np.array([left_current_pose.orientation.x - target_L.orientation.x,
                        left_current_pose.orientation.y - target_L.orientation.y,
                        left_current_pose.orientation.z - target_L.orientation.z,
                        left_current_pose.orientation.w - target_L.orientation.w]))
                        
    dis_diff_R =np.linalg.norm(np.array([right_current_pose.position.x - target_R.position.x,
                    right_current_pose.position.y - target_R.position.y,
                    right_current_pose.position.z - target_R.position.z]))
    dis_pose_R =np.linalg.norm(np.array([right_current_pose.orientation.x - target_R.orientation.x,
                        right_current_pose.orientation.y - target_R.orientation.y,
                        right_current_pose.orientation.z - target_R.orientation.z,
                        right_current_pose.orientation.w - target_R.orientation.w]))

    print(dis_diff_L,dis_pose_L)

    if dis_diff_L>0.01 and dis_pose_L >0.02:
        left_target_pose = target_L
        movegroup.set_pose_target(left_target_pose, end_effector_link='left_gripper')

    if dis_diff_R>0.01 and dis_pose_R >0.02:
        right_target_pose = target_R
        movegroup.set_pose_target(right_target_pose, end_effector_link='right_gripper')


    #执行规划
    plan = movegroup.plan()

    if not plan.joint_trajectory.points:
        print("[ERROR] No trajectory found")
    else:#执行规划
        print("Move to "+lable)
        #movegroup.go(wait=True)


def moveToJointGoalOneArm(movegroup,joint_goal,arm="left_arm",lable='Next'):
    if arm == "left_arm":
        current_joint = movegroup.get_current_joint_values()
        gripper = 'left_gripper'
    else:
        current_joint = movegroup.get_current_joint_values()
        gripper = 'right_gripper'

    dis_pose =np.linalg.norm(np.array(joint_goal)-np.array(current_joint))
    print(current_joint)

    if dis_pose<0.008:
        return 1 #已经到位
    else:
        movegroup.set_joint_value_target(joint_goal)
        #执行规划
        plan = movegroup.plan()

        if not plan.joint_trajectory.points:
            #print("[ERROR] No trajectory found")
            return 0
        else:#执行规划
            #print('Trajectory found')
            #movegroup.go(wait=True)
            return 2

def moveToJointGoalBothArm(movegroup,joint_goal,arm="left_arm",lable='Next'):

    current_joint = movegroup.get_current_joint_values()   #[l(7),r(7)]
    #print(current_joint) #
    #raise SystemError("debug")

    dis_pose =np.linalg.norm(np.array(joint_goal)-np.array(current_joint))
    print(dis_pose)

    if dis_pose<0.01:
        return 1 #已经到位
    else:
        movegroup.set_joint_value_target(joint_goal)
        #执行规划
        plan = movegroup.plan()

        if not plan.joint_trajectory.points:
            #print("[ERROR] No trajectory found")
            return 0
        else:#执行规划
            #print('Trajectory found')
            #movegroup.go(wait=True)
            return 2


def moveToPoseGoalOneArm(movegroup,target_,arm="left_arm",lable='Next'):

    # 读取初始姿态
    if arm == "left_arm":
        current_pose = movegroup.get_current_pose(end_effector_link='left_gripper').pose
        gripper = 'left_gripper'
    else:
        current_pose = movegroup.get_current_pose(end_effector_link='right_gripper').pose
        gripper = 'right_gripper'

    dis_diff =np.linalg.norm(np.array([current_pose.position.x - target_.position.x,
                        current_pose.position.y - target_.position.y,
                        current_pose.position.z - target_.position.z]))
    dis_pose =np.linalg.norm(np.array([current_pose.orientation.x - target_.orientation.x,
                        current_pose.orientation.y - target_.orientation.y,
                        current_pose.orientation.z - target_.orientation.z,
                        current_pose.orientation.w - target_.orientation.w]))
                        

    #print(dis_diff_L,dis_pose_L)

    if dis_diff<0.01 and dis_pose <0.02:
        return 1,0
    else:
        target_pose = target_
        movegroup.set_pose_target(target_pose, end_effector_link=gripper)
        #执行规划
        plan = movegroup.plan()

        if not plan.joint_trajectory.points:
            #print("[ERROR] No trajectory found")
            return 0,0
        else:#执行规划
            #print('Trajectory found')
            #movegroup.go(wait=True)
            return 2,plan




def moveit_baxter_example():
    # initialize moveit_commander and rospy.
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('moveit_baxter_example', anonymous=True)

    # Instantiate a RobotCommander object.  This object is
    # an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("both_arms")

    time.sleep(2)
    

    # 设置初始姿态
    left_current_pose = group.get_current_pose(end_effector_link='left_gripper').pose
    right_current_pose = group.get_current_pose(end_effector_link='right_gripper').pose
    #print(left_current_pose.orientation,right_current_pose.orientation)
    current_poses_path = 'currentPose.pickle'
    current_poses =[left_current_pose,right_current_pose]
    print('.......')
    type(left_current_pose.position)
    if not os.path.exists(current_poses_path):
        with open(current_poses_path, 'wb') as f:
            pickle.dump(current_poses, f)
    
    with open(current_poses_path, 'rb') as f:
        print('Loading Home')
        Home_poses= pickle.load(f)#


    Home_pose_L = Home_poses[0]
    Home_pose_R = Home_poses[1]
    '''    Home_pose_L.position.x = 0.6
    Home_pose_L.position.y = 0.2
    Home_pose_L.position.z = 0.1
    Home_pose_L.orientation.x = -0.27
    Home_pose_L.orientation.y = 0.65
    Home_pose_L.orientation.z = 0.27
    Home_pose_L.orientation.w = 0.65    
    Home_pose_R.position.x = 0.6
    Home_pose_R.position.y = -0.2
    Home_pose_R.position.z = 0.1
    Home_pose_R.orientation.x = 0.26
    Home_pose_R.orientation.y = 0.65
    Home_pose_R.orientation.z = -0.26
    Home_pose_R.orientation.w = 0.65'''




    #print("Home_pose_L",Home_pose_L.position)
    scale=0.1
    H1 = Home_poses[0]#[0.6,0.2,0.1]
    H1 =copy.deepcopy( Home_poses[0])
    H1.position.y+=0.2 #设置左手初始位置的偏移距离
    H1.position.x+=0.05 #设置左手初始位置的偏移距离

    D = copy.deepcopy(H1)#[0.6,0.2,0.1]
    D.position.y -=scale*2 #[0.6,0,0.1]

    C = copy.deepcopy(D)
    C.position.x+=scale*0.9 #[0.69,0,0.1] 


    H2 = Home_poses[1]#[0.6,-0.2,0.1]
    B = Pose()
    B.position.x=H2.position.x- 0.09
    B.position.y=H2.position.y+ 0.05#[0.51,-0.15,0.1]
    B.position.z = H2.position.z
    B.orientation =H2.orientation

    A =Pose()
    A.position.x = B.position.x+ 0.18#[0.69,-0.15,0.1]
    A.position.y=B.position.y
    A.position.z=B.position.z
    A.orientation = B.orientation

    #print('H1',H1.position,'D' ,D.position,'C' ,C.position,  'H2' , H2.position,'B' ,B.position,'A ',A.position)
    
    move_to_target(group,H1,right_current_pose)

    #设置左右手末端执行器姿态
    left_target_pose = D
    #left_target_pose.position.x = left_current_pose.position.x - 0.1  # 0.1m = 10 cm
    #left_target_pose.position.y = left_current_pose.position.y - 0.2
    #left_target_pose.position.z = left_current_pose.position.z - 0.1

    right_target_pose = right_current_pose
    #right_target_pose.position.x = right_current_pose.position.x + 0.2
    #right_target_pose.position.z = right_current_pose.position.z + 0.2

    #设置左右手姿态
    group.set_pose_target(left_target_pose, end_effector_link='left_gripper')
    group.set_pose_target(right_target_pose, end_effector_link='right_gripper')

    #执行规划
    plan = group.plan()

    if not plan.joint_trajectory.points:
        print("[ERROR] No trajectory found")
    else:#执行规划
        print(plan.joint_trajectory.joint_names)
        print(plan.joint_trajectory.header)
        #print(plan.joint_trajectory.points)

        group.go(wait=True)
        print("执行H1")

    #设置左右手末端执行器姿态

    left_target_pose = D

    right_target_pose = right_current_pose

    #设置左右手姿态
    group.set_pose_target(left_target_pose, end_effector_link='left_gripper')
    group.set_pose_target(right_target_pose, end_effector_link='right_gripper')

    #执行规划
    plan = group.plan()

    if not plan.joint_trajectory.points:
        print("[ERROR] No trajectory found")
    else:#执行规划
        #group.go(wait=True)
        print("执行H1---D")


    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)



def generateAndRecordOneArm(arm = "left_arm"):
    
    # initialize moveit_commander and rospy.
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('moveit_baxter_example', anonymous=True)

    # 设置初始姿态
    if arm == "left_arm" or arm == "right_arm":
        pass
    else:
        raise NameError("No arm named {}".format(arm))

    # Instantiate a RobotCommander object.  This object is
    # an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander(arm)

    time.sleep(2)
    print("对{}执行规划".format(arm))
    

    # 设置初始姿态
    if arm == "left_arm":
        current_pose = group.get_current_pose(end_effector_link='left_gripper').pose
    elif arm == "right_arm":
        current_pose = group.get_current_pose(end_effector_link='right_gripper').pose

    #print(left_current_pose.orientation,right_current_pose.orientation)
    current_poses_path =os.path.join(sys.path[0],'InitialPose_{}.pickle'.format(arm)) 
    plans_path = os.path.join(sys.path[0],'plans_{}.pickle'.format(arm))
    home_path = os.path.join(sys.path[0],'home_{}.pickle'.format(arm))

    #current_pose_singal_arm =current_pose
    if  not os.path.exists(current_poses_path):
        with open(current_poses_path, 'wb') as f:
            pickle.dump(current_pose, f)
    
    with open(current_poses_path, 'rb') as f:
        print('Loading Initial')
        Initial_pose= pickle.load(f)#

    #设置双臂目标点
    waypoints = []
    scale = 0.2
    if arm == "left_arm":
        H1 =copy.deepcopy(Initial_pose)
        H1.position.y+=0.2 #设置左手初始位置的偏移距离
        H1.position.x+=0.05 #设置左手初始位置的偏移距离



        D = copy.deepcopy(H1)#[0.6,0.2,0.1]
        D.position.y -=scale*2 #[0.6,0,0.1]

        C = copy.deepcopy(D)
        C.position.x+=scale*0.9 #[0.69,0,0.1] 

        waypoints.append(H1)
        waypoints.append(D)
        waypoints.append(C)
        waypoints.append(H1)

    elif arm == "right_arm":
        H2 = copy.deepcopy(Initial_pose)#[0.6,-0.2,0.1]
        H2.position.y-=0.2 #设置右手初始位置的偏移距离
        H2.position.x+=0.05 #设置右手初始位置的偏移距离
        
        B = copy.deepcopy(H2)#[0.6,-0.2,0.1]
        B.position.x-=scale* 0.9
        B.position.y+=scale*0.5#[0.51,-0.15,0.1]

        A = copy.deepcopy(B)
        A.position.x += scale*1.8#[0.69,-0.15,0.1]

        waypoints.append(H2)
        waypoints.append(B)
        waypoints.append(A)
        waypoints.append(H2)




    #先从Initial  移动到HOME
    case,plan  = moveToPoseGoalOneArm(group,waypoints[0],arm)#返回真  就是找到轨迹    
    if case==2:
        print(arm+" Home pose Trajectory found")

        if group.go(wait=True):
            time.sleep(2)#等待稳定
            joint_value = group.get_current_joint_values()#获取当前角度值
            #print("Home关节角:",joint_value)
            with open(home_path,'wb') as f:#保存
                pickle.dump(joint_value, f)

    elif case==1:
        print(arm+" already in home position")
    else:
        raise SystemError(arm+" Home pose  trajectory  not found")

    time.sleep(2)

    plans =[]
    '''
    for i  in range(1,len(waypoints)):#规划多个点
        case ,plan=moveToPoseGoalOneArm(group,waypoints[i],arm)#返回真  就是找到轨迹
        if case==2:
            print(arm+" pose  {} Trajectory found".format(i))
           
            start = time.time()                           
            flag = group.execute(plan, wait=True)
            end = time.time()
            duration = end-start
            if flag:
                print("Executing time: %s" % duration)
                plans.append(plan)
            else:
                raise SystemError("Execute failed")

        elif case==1:
            print(arm+" already in target pose")
        else:
            print(arm+" pose  {} No trajectory found".format(i))
    
        time.sleep(2)

    '''    
    for i  in range(1,len(waypoints)):#规划多个点
        (plan, fraction) = group.compute_cartesian_path(
                                        waypoints[i:i+1],   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        start = time.time()                           
        flag = group.execute(plan, wait=True)
        end = time.time()
        duration = end-start
        if flag:
            print("Executing time: %s" % duration)
            plans.append((plan,duration))
        else:
            raise SystemError("Execute failed")


    with open(plans_path,'wb') as f:
        pickle.dump(plans, f)

    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)



def scaleTrajectorySpeed(traj,set_time):
    '''对轨迹速度进行减缓和加速
    '''
    new_traj = RobotTrajectory()
    new_traj = traj

    n_joints = len(traj.joint_trajectory.joint_names)#轨迹中有n个关节
    n_points = len(traj.joint_trajectory.points)#轨迹中有n个中间点

    #计算轨迹执行时间
    divisor = 10**9
    t_1 = traj.joint_trajectory.points[-1].time_from_start.secs + float(traj.joint_trajectory.points[-1].time_from_start.nsecs)/divisor
    v_1 = np.array(traj.joint_trajectory.points[-2].velocities)
    a_1 =np.array(traj.joint_trajectory.points[-1].accelerations)
    dt_1 = -v_1/a_1
    dt_1_max =np.max(dt_1,axis=0)
    end_time = t_1+dt_1_max

    time_k = end_time/set_time#k越大，速度越快
    if time_k<0:
        print("Pre raw time :{}, ignor set time {}".format(end_time,set_time))
        time_k = 1.0
    else:
        print("Pre raw time :{}, set time :{}".format(end_time,set_time))


    points = []
    #print(time_k)
    for i in range(n_points):
        point = JointTrajectoryPoint()#创建中间点
        #print("Point: {} ".format(i),traj.joint_trajectory.points[i].time_from_start.secs," ",traj.joint_trajectory.points[i].time_from_start.nsecs)
        #last_i = i-1
        if i>0:
            #计算上一位点的启动时间
            divisor = float(10**9)
            t_last = traj.joint_trajectory.points[i-1].time_from_start.secs + float(traj.joint_trajectory.points[i-1].time_from_start.nsecs)/divisor
            #计算本位点的启动时间（上一位点的终止时间）
            t_now = traj.joint_trajectory.points[i].time_from_start.secs + float(traj.joint_trajectory.points[i].time_from_start.nsecs)/divisor
            #计算上一位点启动到终止的要求运行时间，并所缩放为新的运行时间
            dt = t_now - t_last 
            #计算扩大后的间隔时间
            dt_new = dt/time_k

            #将新的上一位点启动时间，转换为 整数+小数
            t_last_new = points[i-1].time_from_start.secs + float(points[i-1].time_from_start.nsecs)/divisor

            #计算新的本时刻启动时间   
            t_now_new = t_last_new+dt_new
            t_now_new_sec= int(t_now_new)
            t_now_new_nsec = math.modf(t_now_new)[0]*divisor

            point.time_from_start.secs = t_now_new_sec
            point.time_from_start.nsecs = t_now_new_nsec
        
        else:
            point.time_from_start = traj.joint_trajectory.points[i].time_from_start

        #print("New",point.time_from_start.secs,point.time_from_start.nsecs)
        #print("Old", traj.joint_trajectory.points[i].time_from_start.secs, traj.joint_trajectory.points[i].time_from_start.nsecs)


        #point.time_from_start = traj.joint_trajectory.points[i].time_from_start /spd
        #print("Point: {} ".format(i),point.time_from_start.secs," ",point.time_from_start.nsecs)
        point.velocities = list(traj.joint_trajectory.points[i].velocities)
        point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
        point.positions = traj.joint_trajectory.points[i].positions

        for j in range(n_joints):
            point.velocities[j] = point.velocities[j] * time_k
            point.accelerations[j] = point.accelerations[j] * time_k

        #points[i] = point#更新点配置
        points.append(point)

    new_traj.joint_trajectory.points = points     
    return   new_traj


def connectBothPlans(traj_l,traj_r):
    '''将左右臂的轨迹合并为同一个轨迹，由于左臂和右臂规划的时候，使用的分别是两个group
    这里将两个group的轨迹合并成为一个group
    '''
    #分别计算两个轨迹的点数
    n_points_r = len(traj_r.joint_trajectory.points)
    n_points_l = len(traj_l.joint_trajectory.points)
    max_points_num = n_points_l+n_points_r
    print(n_points_l,n_points_r)
    new_traj = copy.deepcopy(traj_l)
    #设置新轨迹的关节名称
    new_traj.joint_trajectory.joint_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2', 
        'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
    #先不设置head


    #拷贝两条轨迹，将会被插值拓展
    new_traj_l = copy.deepcopy(traj_l)
    new_traj_r = copy.deepcopy(traj_r)

    #合并一下，容易处理


    #开始插值
    divisor = float(10**9)
    offset_r = 0
    offset_l =0

    compare_l_id = 1
    compare_r_id = 1


    #print(new_traj_l.joint_trajectory.points[0])
    #print(type(new_traj_l.joint_trajectory.points[0].positions))
    #print(new_traj_l.joint_trajectory.points[0].positions)
    for i in range(max_points_num):
        point_s = JointTrajectoryPoint()#新增某个单臂点

        if i>0 and compare_l_id <n_points_l  and compare_r_id <n_points_r:#从第二点开始
            #分别找双臂的位点启动时间
            t_l = traj_l.joint_trajectory.points[compare_l_id].time_from_start.secs + float(traj_l.joint_trajectory.points[compare_l_id].time_from_start.nsecs)/divisor
            t_r = traj_r.joint_trajectory.points[compare_r_id].time_from_start.secs + float(traj_r.joint_trajectory.points[compare_r_id].time_from_start.nsecs)/divisor
            #双臂位点的位置
            p_l = np.array(traj_l.joint_trajectory.points[compare_l_id].positions)
            p_r = np.array(traj_r.joint_trajectory.points[compare_r_id].positions)
            #双臂位点的速度
            v_l = np.array(traj_l.joint_trajectory.points[compare_l_id].velocities)
            v_r = np.array(traj_r.joint_trajectory.points[compare_r_id].velocities)
            #上一时刻双臂位点的启动时间
            t_l_last = traj_l.joint_trajectory.points[compare_l_id-1].time_from_start.secs + float(traj_l.joint_trajectory.points[compare_l_id-1].time_from_start.nsecs)/divisor
            t_r_last = traj_r.joint_trajectory.points[compare_r_id-1].time_from_start.secs + float(traj_r.joint_trajectory.points[compare_r_id-1].time_from_start.nsecs)/divisor
            #上一时刻双臂位点的位置
            p_l_last = np.array(traj_l.joint_trajectory.points[compare_l_id-1].positions)
            p_r_last  = np.array(traj_r.joint_trajectory.points[compare_r_id-1].positions)
            #上一时刻双臂位点的速度
            v_l_last = np.array(traj_l.joint_trajectory.points[compare_l_id-1].velocities)
            v_r_last = np.array(traj_r.joint_trajectory.points[compare_r_id-1].velocities)

            #如果当前左臂位点启动快于右臂位点
            if t_l<t_r  : #给右臂插值
                #计算时间差值
                dt = t_l-t_r_last
                dt_r = t_r-t_r_last
                #计算比例
                k = dt/dt_r
                #右臂前后位置差
                d_p = p_r - p_r_last
                #右臂前后速度差
                d_v = v_r - v_r_last

                #插出的新位置 和速度
                p_r_new = list(p_r_last + d_p*k)
                v_r_new = list(v_r_last + d_v*k)

                point_s.time_from_start=traj_l.joint_trajectory.points[compare_l_id].time_from_start
                point_s.positions = p_r_new
                point_s.velocities = v_r_new
                point_s.accelerations = traj_r.joint_trajectory.points[compare_r_id-1].accelerations
                
                #将新增点 插入 轨迹中
                new_traj_r.joint_trajectory.points.insert(i,point_s)
                #offset_r += 1
                compare_l_id+=1

            if t_l>t_r: #给左臂插值
                #计算时间差值
                dt = t_r-t_l_last
                dt_l = t_l-t_l_last
                #计算比例
                k = dt/dt_l
                #左臂前后位置差
                d_p = p_l - p_l_last
                #右臂前后速度差
                d_v = v_l - v_l_last

                #插出的新位置 和速度
                p_l_new = list(p_l_last + d_p*k)
                v_l_new = list(v_l_last + d_v*k)

                point_s.time_from_start=traj_r.joint_trajectory.points[compare_r_id].time_from_start
                point_s.positions = p_l_new
                point_s.velocities = v_l_new
                point_s.accelerations = traj_l.joint_trajectory.points[compare_l_id-1].accelerations
                
                #将新增点 插入 轨迹中
                new_traj_l.joint_trajectory.points.insert(i,point_s)

                #offset_l += 1
                compare_r_id+=1

        elif compare_l_id >=n_points_l  and compare_r_id <n_points_r: #一个轨迹已经超出原有轨迹点数了
            point_s.time_from_start=traj_r.joint_trajectory.points[compare_r_id].time_from_start
            point_s.positions = traj_l.joint_trajectory.points[-1].positions#位置不再变化
            point_s.velocities = traj_l.joint_trajectory.points[-1].velocities# 都是0
            point_s.accelerations = traj_l.joint_trajectory.points[-1].accelerations#没有用了
            #将新增点 插入 轨迹中
            new_traj_l.joint_trajectory.points.append(point_s)

            compare_r_id+=1

        elif compare_r_id >=n_points_r  and compare_l_id <n_points_l: #一个轨迹已经超出原有轨迹点数了

            point_s.time_from_start=traj_l.joint_trajectory.points[compare_l_id].time_from_start
            point_s.positions = traj_r.joint_trajectory.points[-1].positions#位置不再变化
            point_s.velocities = traj_r.joint_trajectory.points[-1].velocities# 都是0
            point_s.accelerations = traj_r.joint_trajectory.points[-1].accelerations#没有用了
            #将新增点 插入 轨迹中
            new_traj_r.joint_trajectory.points.append(point_s)
            compare_l_id+=1
            
        elif compare_r_id >=n_points_r  and compare_l_id >=n_points_l:
            break


    #插值完毕，两条轨迹现在的点数相同
    if len(new_traj_l.joint_trajectory.points)!=  len(new_traj_r.joint_trajectory.points):
        print(len(new_traj_l.joint_trajectory.points),len(new_traj_r.joint_trajectory.points))
        raise SystemError("插值失败！")

    print(len(new_traj_l.joint_trajectory.points),len(new_traj_r.joint_trajectory.points))


    #合并两条轨迹
    points_final =[]
    for i in range(len(new_traj_l.joint_trajectory.points)):
        point_b = JointTrajectoryPoint()#双臂轨迹的新点
        point_b.time_from_start=new_traj_l.joint_trajectory.points[i].time_from_start
        point_b.positions = list(new_traj_l.joint_trajectory.points[i].positions) + list(new_traj_r.joint_trajectory.points[i].positions)
        point_b.velocities = list(new_traj_l.joint_trajectory.points[i].velocities) + list(new_traj_r.joint_trajectory.points[i].velocities)
        point_b.accelerations = list(new_traj_l.joint_trajectory.points[i].accelerations) + list(new_traj_r.joint_trajectory.points[i].accelerations)
        points_final.append(point_b)

    new_traj.joint_trajectory.points = points_final
    return   new_traj


def connectOneArmPlans(trajs):
    '''将单条机械臂的多条轨迹拼接为一条'''

    #计算每条轨迹的持续时间
    divisor = float(10**9)
    end_times = []
    for traj in trajs:
        #traj =trajs[i]
        t_1 = traj.joint_trajectory.points[-1].time_from_start.secs + float(traj.joint_trajectory.points[-1].time_from_start.nsecs)/divisor
        v_1 = np.array(traj.joint_trajectory.points[-2].velocities)
        a_1 =np.array(traj.joint_trajectory.points[-1].accelerations)
        dt_1 = -v_1/a_1
        dt_1_max =np.max(dt_1,axis=0)
        end_time = t_1+dt_1_max
        end_times.append(end_time)
    end_times = np.array(end_times)


    #new_traj = RobotTrajectory()
    new_traj = copy.deepcopy(trajs[0])#主要是为了拷贝head、joints_name
    new_traj_points = trajs[0].joint_trajectory.points

    for m in range(len(trajs)):#对每条轨迹
        traj =trajs[m]
        #对条轨迹中的每个点
        
        if m>0:#更改第2条轨迹之后的值
            #计算上一条轨迹的结束时间
            n_points = len(traj.joint_trajectory.points)
            for i in range(n_points):
                point = copy.deepcopy(traj.joint_trajectory.points[i])#创建中间点

                #计算上一位点的启动时间
                t_now = point.time_from_start.secs + float(point.time_from_start.nsecs)/divisor
                t_now_new = t_now+np.sum(end_times[:m])#end_times[m-1]
                t_now_new_sec= int(t_now_new)
                t_now_new_nsec = math.modf(t_now_new)[0]*divisor

                point.time_from_start.secs = t_now_new_sec
                point.time_from_start.nsecs = t_now_new_nsec

                new_traj_points.append(point)
        else:
            pass#第一条轨迹不处理

    new_traj.joint_trajectory.points = new_traj_points     
    return   new_traj




def playOneArm(arm):
    # initialize moveit_commander and rospy.
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('moveit_baxter_example', anonymous=True)

    # 设置初始姿态
    if arm == "left_arm" or arm == "right_arm":
        pass
    else:
        raise NameError("No arm named {}".format(arm))

    # Instantiate a RobotCommander object.  This object is
    # an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander(arm)

    time.sleep(2)
    print("对{}执行规划".format(arm))
    
    '''
    Initial_poses_path =os.path.join(sys.path[0],'InitialPose_'+arm+'.pickle') 
    with open(Initial_poses_path, 'rb') as f:
        print('Loading Initial done')
        Initial_pose= pickle.load(f)#
    '''
    plans_path = os.path.join(sys.path[0],'plans_{}.pickle'.format(arm))
    home_path = os.path.join(sys.path[0],'home_{}.pickle'.format(arm))
    with open(home_path, 'rb') as f:
        Home_pose= pickle.load(f)#
        print('Loading Initial done')


    #先从Initial 移动到HOME
    case  = moveToJointGoalOneArm(group,Home_pose,arm)#返回真  就是找到轨迹    
    if case==2:
        print(arm+" Home pose Trajectory found;  Move to home")
        group.go(wait=True)

    elif case==1:
        print(arm+" already in home position")
    else:
        raise SystemError(arm+" Home pose  trajectory  not found")

    time.sleep(2)


    with open(plans_path,'rb') as f:
        plans = pickle.load(f)


    new_trajs=[]
    set_times=[5,4,10] #设置目标运动时间，设置为-1可以跳过设置
    for index,plan  in  enumerate(plans):#规划多个点
        #traj_time = plan[1]
        set_time = float(set_times[index])
        print(plan[0].joint_trajectory.points)
        traj =scaleTrajectorySpeed(plan,set_time)

        start = time.time()               
        #flag = group.execute(traj, wait=True)
        flag = 1 #不执行  只变速
        end = time.time()

        duration = end-start

        if flag:
            print("Executing time: %s" % duration)
            new_trajs.append(traj)
        else:
            raise SystemError("Execute failed")

    #moveToJointGoalOneArm(group,Home_pose,arm)
    #将多个轨迹按顺序拼接为一条轨迹
    con_traj = connectOneArmPlans(new_trajs)
    group.execute(con_traj, wait=True)


    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

def playBothArm(arm):
    # initialize moveit_commander and rospy.
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('moveit_baxter_example', anonymous=True)

    # 设置初始姿态
    if arm == "both_arms":
        pass
    else:
        raise NameError("We need set both arms in \"play   b\" mode ")

    # Instantiate a RobotCommander object.  This object is
    # an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander(arm)

    time.sleep(2)
    print("对{}执行规划".format(arm))
    
    '''
    Initial_poses_path =os.path.join(sys.path[0],'InitialPose_'+arm+'.pickle') 
    with open(Initial_poses_path, 'rb') as f:
        print('Loading Initial done')
        Initial_pose= pickle.load(f)#
    '''
    plans_path_l = os.path.join(sys.path[0],'plans_left_arm.pickle')
    plans_path_r = os.path.join(sys.path[0],'plans_right_arm.pickle')

    home_path_l = os.path.join(sys.path[0],'home_left_arm.pickle')
    home_path_r = os.path.join(sys.path[0],'home_right_arm.pickle')
    with open(home_path_l, 'rb') as f:
        Home_pose_l= pickle.load(f)#
        print('Loading home left done')
    with open(home_path_r, 'rb') as f:
        Home_pose_r= pickle.load(f)#
        print('Loading home right done')


    #先从Initial 移动到HOME
    case  = moveToJointGoalBothArm(group,Home_pose_l+Home_pose_r,arm)#返回真  就是找到轨迹    
    if case==2:
        print(arm+" Home pose Trajectory found;  Move to home")
        group.go(wait=True)

    elif case==1:
        print(arm+" already in home position")
    else:
        raise SystemError(arm+" Home pose  trajectory  not found")

    time.sleep(2)


    with open(plans_path_l,'rb') as f:
        plans_l = pickle.load(f)
    with open(plans_path_r,'rb') as f:
        plans_r = pickle.load(f)


    set_times_r=[2,3,5]
    set_times_l=[3,6,5]

    new_trajs_r = []
    new_trajs_l = []
    #对右臂多个轨迹进行速度调整
    for index,plan  in  enumerate(plans_r):#规划多个点
        set_time = float(set_times_r[index])
        traj =scaleTrajectorySpeed(plan[0],set_time)
        new_trajs_r.append(traj)

    #对左臂多个轨迹进行速度调整
    for index,plan  in  enumerate(plans_l):#规划多个点
        set_time = float(set_times_l[index])
        traj =scaleTrajectorySpeed(plan,set_time)
        new_trajs_l.append(traj)

    #将单臂多个轨迹按顺序拼接为一条轨迹
    con_traj_r = connectOneArmPlans(new_trajs_r)
    con_traj_l = connectOneArmPlans(new_trajs_l)

    print('开始合并轨迹')

    #再将左右臂轨迹合并成为单个轨迹
    traj_both_arm = connectBothPlans(con_traj_l,con_traj_r)
    #执行最终的轨迹
    group.execute(traj_both_arm, wait=True)
    print('done')

    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)





if __name__ == '__main__':
    if args.arm=='l':
        arm = "left_arm"
    elif args.arm=='r':
        arm = "right_arm"
    elif args.arm=='b':
        arm = "both_arms"
    else:
        raise NameError("Arm Name Error!")
        
    try:
        if args.mode =="generate" and arm != "both_arms":
            generateAndRecordOneArm(arm)#生成单臂轨迹
        elif args.mode =="play" and arm != "both_arms":
            playOneArm(arm)#播放单臂轨迹，用于debug
        elif args.mode =="play" and arm == "both_arms":
            playBothArm(arm)#将两个单臂轨迹结合在一起，进行调度之后播放
            #moveit_baxter_example()

        
    except rospy.ROSInterruptException:
        pass

