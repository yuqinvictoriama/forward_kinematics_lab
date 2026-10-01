import numpy as np
import math as m 

def rotation_x(angle):
    return np.array([
        [1, 0, 0, 0],
        [0, np.cos(angle), -np.sin(angle), 0],
        [0, np.sin(angle), np.cos(angle), 0],
        [0, 0, 0, 1],
    ])


def rotation_y(angle):
    return np.array([
        [np.cos(angle), 0, np.sin(angle), 0],
        [0, 1, 0, 0],
        [-np.sin(angle), 0, np.cos(angle), 0],
        [0, 0, 0, 1],
    ])


def rotation_z(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle), 0, 0],
        [np.sin(angle), np.cos(angle), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])

def translation(x, y, z):
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1],
    ])
    
def fk_front_right(theta1, theta2, theta3, d12):

    # T_0_1 (base_link to leg_front_r_1)
    T_0_1 = translation(0.07500, -0.04450, 0.0) @ rotation_x(1.57080) @ rotation_z(theta1)

    # T_1_2 (leg_front_r_1 to leg_front_r_2)
    T_1_2 = translation(0.0, 0.0, d12) @ rotation_y(-1.57080) @ rotation_z(theta2)

    # T_2_3 (leg_front_r_2 to leg_front_r_3)
    T_2_3 = translation(0.0, -0.0494, 0.0685) @ rotation_y(1.57080) @ rotation_z(theta3)

    # T_3_ee (leg_front_r_3 to end-effector)
    T_3_ee = translation(0.06231, -0.06216, 0.018)

    # Compute the final transformation
    T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

    # Extract the end-effector position
    end_effector_position = T_0_ee @ (0, 0, 0, 1)

    return end_effector_position[:3]


# Testing EEF position with different errors 
print("0 deg, 0.039 m: ", fk_front_right(0.0, 0.0, 0.0, 0.039))
print("0 deg, 0.037 m: ", fk_front_right(0.0, 0.0, 0.0, 0.037))
print("0 deg, 0.035 m: ", fk_front_right(0.0, 0.0, 0.0, 0.035))
print("0 deg, 0.031 m: ", fk_front_right(0.0, 0.0, 0.0, 0.031))


print("45 deg, 0.039 m: ", fk_front_right(m.pi/4, m.pi/4, m.pi/4, 0.039))
print("45 deg, 0.037 m: ", fk_front_right(m.pi/4, m.pi/4, m.pi/4, 0.037))
print("45 deg, 0.035 m: ", fk_front_right(m.pi/4, m.pi/4, m.pi/4, 0.035))
print("45 deg, 0.031 m: ", fk_front_right(m.pi/4, m.pi/4, m.pi/4, 0.031))
