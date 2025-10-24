from urdfpy import URDF
import numpy as np
import os
urdf_path = os.path.abspath("/home/beatrix/ros2_ws/src/leap_control/leap_hand_control/leap_hand_control/data/leap_right/robot.urdf")
robot = URDF.load(urdf_path)

joint_angles = np.ones(16)*0 #16 juntas
# fk_result = robot.link_fk(joint_angles)
# #fk = robot.link_fk()
# print(fk_result)

for link in robot.links:
    print(link.name)


for joint in robot.joints:
    print('{} '.format(
        joint.name
    ))

print(robot.base_link.name)

fk = robot.link_fk()
# print(fk[robot.links[4]])
# print(fk[robot.links[8]])
# print(fk[robot.links[12]])
# print(fk[robot.links[16]])



middle_pos = (np.array([1564, 2048, 3136, 2569]) / 4095) * 2 * np.pi
thumb_pos = (np.array([2550, 630, 2935, 3030]) / 4095) * 2 * np.pi
joint_angles[4:8] = np.array([middle_pos[1]-np.pi,middle_pos[0]-(np.pi/2), middle_pos[2]-np.pi,middle_pos[3]-np.pi])
joint_angles[12:16] = [0,0,0,0]

fk_result = robot.link_fk(joint_angles-np.pi)


middle_fingertip = fk_result[robot.links[8]][:3, 3]
thumb_fingertip = fk_result[robot.links[16]][:3, 3]
distance = np.linalg.norm(middle_fingertip- thumb_fingertip)
print(distance)

joint_angles = joint_angles
# joint_angles = np.ones(16)*0
# joint_angles[4:8] = [0,0,0,1.57]
print(joint_angles)
joint_cfg = {joint.name: angle for joint, angle in zip(robot.joints, joint_angles)}

robot.show(cfg=joint_cfg)