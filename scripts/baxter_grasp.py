#!/usr/bin/env python
#coding=utf-8
"""
    moveit_ik_demo.py - Version 0.1 2014-01-14
    使得机械臂，先在初始状态，然后移动一下机械臂，然后再回到初始状态，停止
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
from moveit_msgs.msg import RobotTrajectory,DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler,quaternion_multiply,quaternion_from_matrix
from autolab_core import RigidTransform,transformations
from pyquaternion import Quaternion
try:
    from gpd_grasp_msgs.msg import GraspConfig,GraspConfigList
except ImportError:
    print("Please install grasp msgs from https://github.com/TAMS-Group/gpd_grasp_msgs in your ROS workspace")
    exit()


class MoveItDemo:
   
    def lookupTransform(self,tf_listener, target, source):
        tf_listener.waitForTransform(target, source, rospy.Time(), rospy.Duration(4.0)) #等待时间为4秒

        trans, rot = tf_listener.lookupTransform(target, source, rospy.Time())
        euler = tf.transformations.euler_from_quaternion(rot)

        source_target = tf.transformations.compose_matrix(translate = trans, angles = euler)
        return source_target
    def getTfFromMatrix(self,matrix):
        scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(matrix)
        return trans, tf.transformations.quaternion_from_euler(*angles), angles


    def quater_multi_vec(self,quater,vec):
        quater_=tf.transformations.quaternion_inverse(quater)
        vec_quater=np.c_[vec,[0]]
        temp=quaternion_multiply(quater,vec_quater)
        temp=quaternion_multiply(temp,quater_)
        return temp[:3]

    
    def Callback(self,data): 
        """
        根据接收的夹爪抓取姿态，计算预抓取夹爪的位置姿态
        """

        #data是GraspConfigList,data.grasps是GraspConfig[]类型,
        #data.grasps[0]是list中第一个GraspConfig类型的数据，代表的最优的那个抓取配置
        self.grasp_config=data.grasps[0]
        #最终抓取姿态
        self.grasp_pose=Pose()
        #预抓取姿态
        self.pre_grasp_pose=Pose()

        #以下是读取grasp的pose，需要注意的是，此时pose的参考系是谁？是桌面标签参考坐标系，并不是panda_link0
        #读取grasp pose的三个方向向量，转换为ndarray形式
        approach=np.array([self.grasp_config.approach.x,self.grasp_config.approach.y,self.grasp_config.approach.z])#接近轴
        binormal=np.array([self.grasp_config.binormal.x,self.grasp_config.binormal.y,self.grasp_config.binormal.z])#合并轴
        axis=np.array([self.grasp_config.axis.x,self.grasp_config.axis.y,self.grasp_config.axis.z])#撸轴
        #进行方向向量归一化
        approach=approach/np.linalg.norm(approach)
        binormal=binormal/np.linalg.norm(binormal)
        axis=axis/np.linalg.norm(axis)
        #读取夹爪底部中心点的坐标
        goal_trans=np.array([self.grasp_config.bottom.x,self.grasp_config.bottom.y,self.grasp_config.bottom.z])#原点
        scale=np.array([0,0,0,1])
        temp=np.array([0,0,0])
        goal_rot=np.hstack([axis,binormal,approach,temp]).reshape(4,3).T
        goal_rot=np.vstack([goal_rot,scale])

        #将旋转坐标系转换为相对于marker的四元数旋转
        R = tf.transformations.rotation_matrix(0.123, (1, 2, 3))
        goal_quater=quaternion_from_matrix(R)
        #print(goal_quater)


        #link8  2gripper 2   先将goal旋转到base坐标系下，再根据gripper与link8关系，将goal旋转到link8，求出目标状态下link8在base下的姿态
        base2goal_rot=quaternion_multiply(self.base2marker_rot,goal_quater)
        base2goal_trans=self.base2marker_trans+goal_trans
        base2goal_trans=self.quater_multi_vec(self.base2marker_rot,base2goal_trans)
     

     
        base2link8_rot=quaternion_multiply(base2goal_rot,self.gripper2link8_rot)
        base2link8_trans=self.gripper2link8_trans+base2goal_trans




        self.tf_broadcaster.sendTransform(
            base2goal_trans,
            base2goal_rot,
            rospy.Time.now(),
            "base2goal",
            "panda_link0")        
        #发布目标抓取姿态在base坐标系的位置
        self.tf_broadcaster.sendTransform(
            goal_trans,
            base2link8_rot,
            rospy.Time.now(),
            "marker2goal",
            "ar_marker_6")   
        #发布目标抓取姿态在base坐标系的位置
        self.tf_broadcaster.sendTransform(
            base2link8_trans,
            base2link8_rot,
            rospy.Time.now(),
            "base2link8",
            "panda_link0")        
        #发布目标抓取姿态在base坐标系的位置
        self.tf_broadcaster.sendTransform(
            self.base2marker_trans,
            base2link8_rot,
            rospy.Time.now(),
            "base2marker",
            "panda_link0")        

        #标志回调函数处理完毕
        self.callback_done=False 


    def scale_trajectory_speed(self,traj,spd=0.1):
        new_traj = RobotTrajectory()
        new_traj = traj

        n_joints = len(traj.joint_trajectory.joint_names)
        n_points = len(traj.joint_trajectory.points)

        #spd = 3.0

        points = list(traj.joint_trajectory.points)

        for i in range(n_points):
            point = JointTrajectoryPoint()
            point.time_from_start = traj.joint_trajectory.points[i].time_from_start / spd
            point.velocities = list(traj.joint_trajectory.points[i].velocities)
            point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
            point.positions = traj.joint_trajectory.points[i].positions

            for j in range(n_joints):
                point.velocities[j] = point.velocities[j] * spd
                point.accelerations[j] = point.accelerations[j] * spd

            points[i] = point

        new_traj.joint_trajectory.points = points     
        return   new_traj

    def add_table(self):
        """为场景中添加抓取桌面，防止机械臂与桌子发生碰撞
        """
        #清除场景可能存在的遗留物体
        self.scene.remove_world_object('table') 
        #设置桌面尺寸      x  y   z
        table_size = [0.6, 1.2, 0.01]
        #设置桌子的位置姿态
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'panda_link0'
        table_pose.pose.position.x = 0.55
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = 0.025
        table_pose.pose.orientation.w = 1.0
        # 将table加入场景当中
        self.scene.add_box('table', table_pose, table_size)

    def set_gripper(self,hand_group,gripper_width):
        """设置panda 夹爪的开合大小
        gripper_width 最大0.08m
        """
        if gripper_width>0.08 or gripper_width<0.0:
             raise Exception
        oneside=gripper_width/2
        joint_goal = hand_group.get_current_joint_values()
        joint_goal[0] = oneside
        joint_goal[1] = oneside
        hand_group.go(joint_goal, wait=True)
        hand_group.stop()
        print("Gripper move done!")


    def __init__(self):
        """
        关于baxter无法通过moveit获取当前姿态的错误    https://github.com/ros-planning/moveit/issues/1187
        joint_state_topic = ['joint_states:=/robot/joint_states']
        初始化moveit的 API接口
        """
        moveit_commander.roscpp_initialize(sys.argv)
        #初始化ros节点 名为panda_grasp
        rospy.init_node('panda_grasp', anonymous=True)
        #构建一个tf发布器
        self.tf_broadcaster=tf.TransformBroadcaster()

        self.grasp_config=GraspConfig()

        #创建多用途的TF监听器
        self.tf_listener = tf.TransformListener()
        #变换关系正确读取的标志位
        get_transform=False
        #等待并获取正确的tf变换关系
        while not get_transform:
            try:
                #尝试查看机器人基座base与桌面标签之间的转换
                base2marker_trans, self.base2marker_rot = self.tf_listener.lookupTransform('/panda_link0', '/ar_marker_6', rospy.Time(0))
                euler = tf.transformations.euler_from_quaternion(self.base2marker_rot)
                self.base2marker = tf.transformations.compose_matrix(translate = base2marker_trans, angles = euler)
                #将trans转换成为ndarry
                self.base2marker_trans=np.array(base2marker_trans)
                #查看gripper到link8之间的变换
                gripper2link8_trans, self.gripper2link8_rot = self.tf_listener.lookupTransform( '/panda_EE', '/panda_link8',rospy.Time(0))
                euler = tf.transformations.euler_from_quaternion(self.gripper2link8_rot)
                self.gripper2link8 = tf.transformations.compose_matrix(translate = gripper2link8_trans, angles = euler)
                self.gripper2link8_trans=np.array(gripper2link8_trans)
                #查看base到panda_link8的变换，此时就是查询gripper的初始姿态
                trans, rot = self.tf_listener.lookupTransform( '/panda_link0', '/panda_link8',rospy.Time(0))
                euler = tf.transformations.euler_from_quaternion(rot)
                self.base2Initial_link8 = tf.transformations.compose_matrix(translate = trans, angles = euler)

                get_transform = True
                rospy.loginfo("got transform complete")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("got transform failed")
                rospy.sleep(0.5)
                continue


        # 初始化场景对象
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(2)
        # 创建机械臂规划组对象
        self.selected_arm = moveit_commander.MoveGroupCommander('panda_arm')
        #创建机械手规划对象
        self.hand_group=moveit_commander.MoveGroupCommander('hand')
        #通过此发布器发布规划的轨迹
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               DisplayTrajectory,
                                               queue_size=20)
        # 获取末端执行器名称
        self.end_effector_link = self.selected_arm.get_end_effector_link()
        print("检测到末端执行器{}".format(self.end_effector_link))
                        
        # 创建机械臂父坐标系名称字符
        reference_frame = 'panda_link0'
      
        # 设置父坐标系名称
        #self.selected_arm.set_pose_reference_frame(reference_frame)

        # 允许机械臂末位姿的错误余量
        self.selected_arm.set_goal_position_tolerance(0.01)
        self.selected_arm.set_goal_orientation_tolerance(0.05)

        #不允许规划失败重规划,规划时间只允许5秒钟,否则很浪费时间
        self.selected_arm.allow_replanning(False)
        self.selected_arm.set_planning_time(5)
        #为场景添加桌子，防止机械臂碰撞桌面
        self.add_table()
        
        # 设置panda的home姿态
        Home_positions = [0.04, -0.70, 0.18, -2.80,  0.19, 2.13, 0.92]#移动到工作位置，使用正运动学
        #self.selected_arm.remember_joint_values('resting', joint_positions)#存储当前状态为初始状态
        #self.start_state =self.selected_arm.get_current_pose()

        # 设置机械臂各个关节目标姿态为home角度
        self.selected_arm.set_joint_value_target(Home_positions)
        # Plan and execute the motion，运动到Home位置
        self.selected_arm.go()
        #执行完毕之后stop机械臂防止一些其他的余量？
        self.selected_arm.stop()
        
        #初始张开夹爪
        self.set_gripper(self.hand_group,0.08)

        ######################开始等待接收夹爪姿态#########################
        print("Waiting for gripper pose!")
        #等待gripper_pose这个话题的发布（目标抓取姿态，该姿态将会进行抓取）
        #rospy.wait_for_message('/detect_grasps/clustered_grasps', GraspConfigList) 
        #创建消息订阅器，订阅“gripper_pose”话题，这个gripper_pose，是以桌面标签为参考系的
        #接收到之后，调用回调函数，有两件事要做
        # 1.将gripper_pose
        # 计算后撤距离
        self.callback_done=False
        rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, self.Callback,queue_size=1)

        #######################执行抓取####################################
        while not rospy.is_shutdown():
            #等待回调函数处理完
            if self.callback_done:
                self.callback_done=False
            else:
                rospy.sleep(0.5)
                continue


            #以当前姿态作为规划起始点
            self.selected_arm.set_start_state_to_current_state()  
            # 对末端执行器姿态设定目标姿态
            #self.selected_arm.set_pose_target(target_pose, 'left_gripper')
            
            # 规划轨迹
            #traj = self.selected_arm.plan(target_pose.pose)
            
            # 执行轨迹，运行到预抓取位置
            #self.selected_arm.execute(traj)
            #print(self.end_effector_link)


            print('Moving to pre_grasp_pose')
            #self.selected_arm.pick("test",self.grasp_config,plan_only = True)
            #traj=self.selected_arm.plan(self.pre_grasp_pose)
            #self.selected_arm.set_pose_target(self.pre_grasp_pose,end_effector_link="panda_EE")
            #traj=self.selected_arm.plan()

            #continue

            #success=self.selected_arm.execute(traj)

            #print(target_pose.pose)
            #设置规划
            #self.selected_arm.set_planning_time(5)
            success=self.selected_arm.go(self.pre_grasp_pose,wait=True)
            self.selected_arm.stop()
            self.selected_arm.clear_pose_targets()

            
            
            if not success:
                print('Failed to move to pre_grasp_pose!')
                continue
            
            print('Move to pre_grasp_pose succeed')
            #等待机械臂稳定
            rospy.sleep(1)
            #再设置当前姿态为起始姿态
            self.selected_arm.set_start_state_to_current_state()  
            #
            waypoints = []
            wpose=self.selected_arm.get_current_pose().pose
            #print("#####wpose.position")
            #print(wpose.position)
            #print("#####self.grasp_pose2")
            #print(self.grasp_pose.position)
            wpose.position.x=  self.grasp_pose.position.x
            wpose.position.y=  self.grasp_pose.position.y
            wpose.position.z=  self.grasp_pose.position.z

            waypoints.append(copy.deepcopy(wpose))
            #wpose = self.selected_arm.get_current_pose().pose
            #wpose.position.z -= scale * 0.1

            #规划从当前位姿，保持姿态，转移到目标夹爪姿态的路径
            (plan, fraction) = self.selected_arm.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.01,        # eef_step
                0.0)         # jump_threshold
             ##显示轨迹
            display_trajectory = DisplayTrajectory()
            display_trajectory.trajectory_start = self.selected_arm.get_current_state()
            display_trajectory.trajectory.append(plan)
            # Publish
            display_trajectory_publisher.publish(display_trajectory)

            #执行,并等待这个轨迹执行成功
            new_plan=self.scale_trajectory_speed(plan,0.3)
            self.selected_arm.execute(new_plan,wait=True)
            #self.selected_arm.shift_pose_target(2,0.05,"panda_link8")
            #self.selected_arm.go()


            #执行抓取
            rospy.sleep(2)
            print("Grasping")
            joint_goal = self.hand_group.get_current_joint_values()
            joint_goal[0] = 0.015
            joint_goal[1] = 0.015
            #plan=self.hand_group.plan(joint_goal)
            #new_plan=self.scale_trajectory_speed(plan,0.3)
            self.hand_group.go(joint_goal,wait=True)
            self.hand_group.stop()

            ####################抓取完后撤####################
            waypoints = []
            wpose=self.selected_arm.get_current_pose().pose
            
            wpose.position.x=  self.pre_grasp_pose.position.x
            wpose.position.y=  self.pre_grasp_pose.position.y
            wpose.position.z=  self.pre_grasp_pose.position.z

            waypoints.append(copy.deepcopy(wpose))
            
            #规划从当前位姿，保持姿态，转移到目标夹爪姿态的路径
            (plan, fraction) = self.selected_arm.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.01,        # eef_step
                0.0)         # jump_threshold

            #执行,并等待后撤成功
            new_plan=self.scale_trajectory_speed(plan,0.6)
            self.selected_arm.execute(new_plan,wait=True)
            """
            display_trajectory = DisplayTrajectory()
            display_trajectory.trajectory_start = self.selected_arm.get_current_state()
            display_trajectory.trajectory.append(plan)
            # Publish
            display_trajectory_publisher.publish(display_trajectory)
            """

            ######################暂时设置直接回到Home############################

            #self.selected_arm.remember_joint_values('resting', joint_positions)#存储当前状态为初始状态
            #self.start_state =self.selected_arm.get_current_pose(self.end_effector_link)
            
            # Set the arm's goal configuration to the be the joint positions
            self.selected_arm.set_joint_value_target(Home_positions)
                    
            # Plan and execute the motion，运动到Home位置
            self.selected_arm.go()
            self.selected_arm.stop()

            joint_goal = self.hand_group.get_current_joint_values()
            joint_goal[0] = 0.04
            joint_goal[1] = 0.04
            self.hand_group.go(joint_goal, wait=True)
            self.hand_group.stop()

            print("Grasp done")

            rospy.sleep(5)


        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Arm tracker node terminated.")

    
    