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
        
        # 为不同手指和不同坐标系设置比例系数
        self.glove_to_botyard_mapping_scale = {
            'thumb': {
                'x': 1.6,  # 大拇指X轴比例
                'y': 1,  # 大拇指Y轴比例
                'z': 1.6   # 大拇指Z轴比例
            },
            'index': {
                'x': 1.6,  # 食指X轴比例
                'y': 1.0,  # 食指Y轴比例
                'z': 1.6  # 食指Z轴比例
            },
            'middle': {
                'x': 1.6,  # 中指X轴比例
                'y': 1.0,  # 中指Y轴比例
                'z': 1.6   # 中指Z轴比例
            },
            'ring': {
                'x': 1.6,  # 无名指X轴比例
                'y': 1.0,  # 无名指Y轴比例
                'z': 1.8   # 无名指Z轴比例
            },
            'pinky': {
                'x': 1.8,  # 小指X轴比例（通常更短，需要更大比例）
                'y': 1.0,  # 小指Y轴比例
                'z': 1.6   # 小指Z轴比例
            }
        }
        
        # 保持向后兼容的默认比例
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
        # gets the data converts it and then computes IK and visualizes
        poses = pose.poses
        
        hand_pos = []
        glove_pos = []
        
        # 定义手指索引映射
        finger_indices = {
            'thumb_middle': 0,   # 拇指中段
            'thumb_tip': 1,      # 拇指尖
            'index_middle': 2,   # 食指中段
            'index_tip': 3,      # 食指尖
            'middle_middle': 4,  # 中指中段
            'middle_tip': 5,     # 中指尖
            'ring_middle': 6,    # 无名指中段
            'ring_tip': 7,       # 无名指尖
            'pinky_middle': 8,   # 小指中段
            'pinky_tip': 9       # 小指尖
        }
        
        for i in range(0, 10):
            # 存储原始手套数据
            glove_pos.append([poses[i].position.x,
                              poses[i].position.y,
                                poses[i].position.z])
            
            # 根据索引确定手指类型
            if i == finger_indices['thumb_middle'] or i == finger_indices['thumb_tip']:
                finger_type = 'thumb'
            elif i == finger_indices['index_middle'] or i == finger_indices['index_tip']:
                finger_type = 'index'
            elif i == finger_indices['middle_middle'] or i == finger_indices['middle_tip']:
                finger_type = 'middle'
            elif i == finger_indices['ring_middle'] or i == finger_indices['ring_tip']:
                finger_type = 'ring'
            elif i == finger_indices['pinky_middle'] or i == finger_indices['pinky_tip']:
                finger_type = 'pinky'
            else:
                finger_type = 'index'  # 默认使用食指比例
            
            # 使用对应手指的比例系数
            scale = self.glove_to_botyard_mapping_scale[finger_type]
            hand_pos.append([
                poses[i].position.x * scale['x'],
                poses[i].position.y * scale['y'],
                poses[i].position.z * scale['z']
            ])
        # hand_pos[9][0] = hand_pos[2][0] + 0.01
        # hand_pos[1][0] += 0.035
        hand_pos[1][1] += 0.03
        
        # 计算人手大拇指和其他手指尖的距离
        thumb_pos = glove_pos[1]  # 大拇指位置（使用原始手套数据）
        
        # 根据hand_pos的索引，定义各手指位置
        # hand_pos[0] = 拇指中段, hand_pos[1] = 拇指尖
        # hand_pos[2] = 食指中段, hand_pos[3] = 食指尖
        # hand_pos[4] = 中指中段, hand_pos[5] = 中指尖
        # hand_pos[6] = 无名指中段, hand_pos[7] = 无名指尖
        # hand_pos[8] = 小指中段, hand_pos[9] = 小指尖
        
        fingertip_positions = {
            'index': glove_pos[3],    # 食指尖
            'middle': glove_pos[5],   # 中指尖
            'ring': glove_pos[7],     # 无名指尖
            'pinky': glove_pos[9]     # 小指尖
        }
        
        # 计算大拇指与每个手指尖的距离
        distances = {}
        pinch_detected = False
        pinch_finger = None
        
        # 为不同手指设置不同的捏合阈值
        pinch_thresholds = {
            'index': 0.03,   # 食指阈值
            'middle': 0.03,  # 中指阈值
            'ring': 0.04,    # 无名指阈值
            'pinky': 0.05    # 小指阈值
        }
        
        for finger_name, finger_pos in fingertip_positions.items():
            # 计算欧几里得距离
            distance = np.sqrt(
                (thumb_pos[0] - finger_pos[0])**2 + 
                (thumb_pos[1] - finger_pos[1])**2 + 
                (thumb_pos[2] - finger_pos[2])**2
            )
            distances[finger_name] = distance
            print(f"人手大拇指与{finger_name}指尖距离: {distance:.4f}")
            
            # 使用对应手指的阈值判断是否捏合
            threshold = pinch_thresholds[finger_name]
            if distance < threshold:
                print(f"检测到人手{finger_name}指尖捏合！(阈值: {threshold})")
                pinch_detected = True
                pinch_finger = finger_name
        
        # 找到距离大拇指最近的手指
        if distances:
            closest_finger = min(distances, key=distances.get)
            min_distance = distances[closest_finger]
            print(f"人手距离大拇指最近的手指: {closest_finger}, 距离: {min_distance:.4f}")
        
        # 将捏合信息传递给compute_IK函数
        self.compute_IK(hand_pos, pinch_detected, pinch_finger)
        self.update_target_vis(hand_pos)

        
    def compute_IK(self, hand_pos, pinch_detected, pinch_finger):
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

        # 第一次计算逆运动学
        jointPoses = p.calculateInverseKinematics2(
            self.botyardId,
            self.botyardEndEffectorIndex,
            botyardEndEffectorPos,
            solver=p.IK_DLS,
            maxNumIterations=50,
            residualThreshold=0.0001,
        )
        
        # 应用第一次的关节角度
        combined_jointPoses = ((0,0,0,0,0,) + jointPoses[0:4] +(0.0,) 
                               + jointPoses[4:8] +(0.0,) +jointPoses[8:12]+(0.0,)
                                 + jointPoses[12:16]+(0.0,) + (0.0,)+ jointPoses[16:20]+(0.0,0.0))
        combined_jointPoses = list(combined_jointPoses)

        # 应用第一次的关节角度到机器人
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
        
        # 如果检测到捏合，获取灵巧手对应手指的link位置并重新计算
        if pinch_detected and pinch_finger:
            print(f"检测到与{pinch_finger}捏合，获取灵巧手link位置并重新计算")
            
            # 定义各手指对应的link index
            finger_link_indices = {
                'index': 14,   # FFJ0  
                'middle': 19,  # MFJ0
                'ring': 24,    # RFJ0
                'pinky': 29    # LFJ0
            }
            
            try:
                # 获取被捏合手指的link位置
                pinch_link_index = finger_link_indices[pinch_finger]
                pinch_link_pos = p.getLinkState(self.botyardId, pinch_link_index)[0]
                print(f"灵巧手{pinch_finger}指尖位置: {pinch_link_pos}")
                
                # 将大拇指目标位置设置为灵巧手对应手指的link位置
                rightHandThumb_pos = pinch_link_pos
                
                # 大拇指中段也使用相同位置，不进行偏移
                rightHandThumb_middle_pos = pinch_link_pos
                
                # 更新目标位置数组
                botyardEndEffectorPos[8] = rightHandThumb_middle_pos  # 大拇指中段
                botyardEndEffectorPos[9] = rightHandThumb_pos         # 大拇指尖
                
                print(f"调整后大拇指目标位置: {rightHandThumb_pos}")
                
                # 第二次计算逆运动学（使用调整后的目标位置）
                jointPoses = p.calculateInverseKinematics2(
                    self.botyardId,
                    self.botyardEndEffectorIndex,
                    botyardEndEffectorPos,
                    solver=p.IK_DLS,
                    maxNumIterations=50,
                    residualThreshold=0.0001,
                )
                
                # 应用第二次的关节角度
                combined_jointPoses = ((0,0,0,0,0,) + jointPoses[0:4] +(0.0,) 
                                       + jointPoses[4:8] +(0.0,) +jointPoses[8:12]+(0.0,)
                                         + jointPoses[12:16]+(0.0,) + (0.0,)+ jointPoses[16:20]+(0.0,0.0))
                combined_jointPoses = list(combined_jointPoses)

                # 应用第二次的关节角度到机器人
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
                    
            except Exception as e:
                print(f"获取灵巧手{pinch_finger}指尖位置失败: {e}")

        # map results to real robot
        robot_joint_pose = [combined_jointPoses[i] for i in ordered_urdf_indices]
        self.command.data = robot_joint_pose  # 将获取的 robot_joint_pose 直接赋值给消息数据
        # self.get_logger().info(f"Publishing message: {self.command.data}")
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