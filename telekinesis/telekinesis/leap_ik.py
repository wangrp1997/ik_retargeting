#!/usr/bin/env python3
import pybullet as p
import numpy as np
import rclpy
import os
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
import sys
from ament_index_python.packages import get_package_share_directory
'''
This takes the glove data, and runs inverse kinematics and then publishes onto LEAP Hand.

Note how the fingertip positions are matching, but the joint angles between the two hands are not.  :) 

Inspired by Dexcap https://dex-cap.github.io/ by Wang et. al. and Robotic Telekinesis by Shaw et. al.
'''
# 关节名称与urdf中的索引硬编码映射
ordered_urdf_indices = [
    3, 1,       # FAJ1, FAJ3
    13, 12, 11, 10,  # FFJ1-4
    29, 28, 27, 26, 25,  # LFJ1-5
    18, 17, 16, 15,  # MFJ1-4
    23, 22, 21, 20,  # RFJ1-4
    8, 7, 6, 5       # THJ1-4
]

class BotyardPybulletIK(Node):
    def __init__(self):
        super().__init__('leap_pyb_ik')  
        # start pybullet
        #clid = p.connect(p.SHARED_MEMORY)
        #clid = p.connect(p.DIRECT)
        p.connect(p.GUI)
        # load right leap hand      
        path_src = os.path.abspath(__file__)
        path_src = os.path.dirname(path_src)
        self.is_left = self.declare_parameter('isLeft', True).get_parameter_value().bool_value
        self.glove_to_botyard_mapping_scale_x = 1.6
        self.glove_to_botyard_mapping_scale_y = 1
        self.glove_to_botyard_mapping_scale_z = 1
        self.botyardEndEffectorIndex = [13, 14, 18, 19, 23, 24, 28, 29, 8, 9]
        if self.is_left:
            path_src = os.path.join(path_src, "botyard_hand_left/botyard_hand_left_pybullet.urdf")
            ##You may have to set this path for your setup on ROS2
            self.botyardId = p.loadURDF(
                path_src,
                [-0.15, 0.02, 0.00],
                p.getQuaternionFromEuler([-1.57, 0, -1.57]),
                useFixedBase = True
            )
            num_joints = p.getNumJoints(self.botyardId)
            for i in range(num_joints):
                info = p.getJointInfo(self.botyardId, i)
                print(f"Joint index: {i}, name: {info[1].decode('utf-8')}")
            self.hand_pub = self.create_publisher(Float64MultiArray, 
                                                   '/dexhand_position_controller/commands', 10)
            # 初始化消息数据，大小为23，值为0.0
            self.command = Float64MultiArray()
            self.sub_skeleton = self.create_subscription(PoseArray, "/glove/l_short", self.get_glove_data, 10)
        else:
            path_src = os.path.join(path_src, "leap_hand_mesh_right/robot_pybullet.urdf")
            ##You may have to set this path for your setup on ROS2
            self.botyardId = p.loadURDF(
                path_src,
                [0.12, 0.052, 0],
                p.getQuaternionFromEuler([0, 0, 0]),
                useFixedBase = True
            )
            
            self.pub_hand = self.create_publisher(JointState, '/leaphand_node/cmd_allegro_right', 10)
            self.sub_skeleton = self.create_subscription(PoseArray, "/glove/r_short", self.get_glove_data, 10)

        self.numJoints = p.getNumJoints(self.botyardId)
        p.setGravity(0, 0, 0)
        useRealTimeSimulation = 0
        p.setRealTimeSimulation(useRealTimeSimulation)
        self.create_target_vis()
            
    def create_target_vis(self):
        # load balls
        small_ball_radius = 0.01
        small_ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=small_ball_radius)
        ball_radius = 0.01
        ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
        baseMass = 0.001
        basePosition = [0.25, 0.25, 0]
        
        self.ballMbt = []
        for i in range(0,5):
            self.ballMbt.append(p.createMultiBody(baseMass=baseMass, baseCollisionShapeIndex=ball_shape, basePosition=basePosition)) # for base and finger tip joints    
            no_collision_group = 0
            no_collision_mask = 0
            p.setCollisionFilterGroupMask(self.ballMbt[i], -1, no_collision_group, no_collision_mask)
        p.changeVisualShape(self.ballMbt[0], -1, rgbaColor=[1, 0, 0, 1]) 
        p.changeVisualShape(self.ballMbt[1], -1, rgbaColor=[0, 1, 0, 1]) 
        p.changeVisualShape(self.ballMbt[2], -1, rgbaColor=[0, 0, 1, 1])  
        p.changeVisualShape(self.ballMbt[3], -1, rgbaColor=[1, 1, 1, 1])
        p.changeVisualShape(self.ballMbt[4], -1, rgbaColor=[1, 0, 1, 1])

        
    def update_target_vis(self, hand_pos):
        _, current_orientation = p.getBasePositionAndOrientation( self.ballMbt[0])
        p.resetBasePositionAndOrientation(self.ballMbt[0], hand_pos[3], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[1])
        p.resetBasePositionAndOrientation(self.ballMbt[1], hand_pos[5], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[2])
        p.resetBasePositionAndOrientation(self.ballMbt[2], hand_pos[7], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[3])
        p.resetBasePositionAndOrientation(self.ballMbt[3], hand_pos[1], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[4])
        p.resetBasePositionAndOrientation(self.ballMbt[4], hand_pos[9], current_orientation)

    def get_glove_data(self, pose):
        #gets the data converts it and then computes IK and visualizes
        poses = pose.poses
        hand_pos = []  
        for i in range(0,10):
            hand_pos.append([poses[i].position.x * self.glove_to_botyard_mapping_scale_x,
                              poses[i].position.y * self.glove_to_botyard_mapping_scale_y,
                                poses[i].position.z * self.glove_to_botyard_mapping_scale_z])
        # hand_pos[9][0] = hand_pos[2][0] + 0.01
        # hand_pos[1][0] += 0.035
        hand_pos[1][1] += 0.03

        self.compute_IK(hand_pos)
        self.update_target_vis(hand_pos)
        
    def compute_IK(self, hand_pos):
        p.stepSimulation()     

        rightHandIndex_middle_pos = hand_pos[2]
        rightHandIndex_pos = hand_pos[3]
        
        rightHandMiddle_middle_pos = hand_pos[4]
        rightHandMiddle_pos = hand_pos[5]
        
        rightHandRing_middle_pos = hand_pos[6]
        rightHandRing_pos = hand_pos[7]

        rightHandPinky_middle_pos = hand_pos[8]
        rightHandPinky_pos = hand_pos[9]

        rightHandThumb_middle_pos = hand_pos[0]
        rightHandThumb_pos = hand_pos[1]
        
        botyardEndEffectorPos = [
            rightHandIndex_middle_pos,
            rightHandIndex_pos,
            rightHandMiddle_middle_pos,
            rightHandMiddle_pos,
            rightHandRing_middle_pos,
            rightHandRing_pos,
            rightHandPinky_middle_pos,
            rightHandPinky_pos,
            rightHandThumb_middle_pos,
            rightHandThumb_pos
        ]

        jointPoses = p.calculateInverseKinematics2(
            self.botyardId,
            self.botyardEndEffectorIndex,
            botyardEndEffectorPos,
            solver=p.IK_DLS,
            maxNumIterations=50,
            residualThreshold=0.0001,
        )
        # print(jointPoses)
        combined_jointPoses = ((0,0,0,0,0,) + jointPoses[0:4] +(0.0,) 
                               + jointPoses[4:8] +(0.0,) +jointPoses[8:12]+(0.0,)
                                 + jointPoses[12:16]+(0.0,) + (0.0,)+ jointPoses[16:20]+(0.0,0.0))
        combined_jointPoses = list(combined_jointPoses)

        # update the hand joints
        for i in range(len(combined_jointPoses)):
            p.setJointMotorControl2(
                bodyIndex=self.botyardId,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=combined_jointPoses[i],
                targetVelocity=0,
                force=500,
                positionGain=0.3,
                velocityGain=1,
            )

        # map results to real robot
        robot_joint_pose = [combined_jointPoses[i] for i in ordered_urdf_indices]
        self.command.data = robot_joint_pose  # 将获取的 robot_joint_pose 直接赋值给消息数据
        self.get_logger().info(f"Publishing message: {self.command.data}")
        self.hand_pub.publish(self.command)

        self.get_logger().info('Publishing hand position commands...')
        # self.pub_hand.publish(stater)

def main(args=None):
    rclpy.init(args=args)
    botyardpybulletik = BotyardPybulletIK()
    rclpy.spin(botyardpybulletik)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    botyardpybulletik.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()