from urdfpy import URDF
import numpy as np
robot = URDF.load('robot.urdf')

joint_angles = np.ones(16)*np.pi  # Ajuste conforme as suas juntas
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
print(fk[robot.links[1]])

robot.show(cfg={
    '0': 0.0,
    '1':0.0

 })