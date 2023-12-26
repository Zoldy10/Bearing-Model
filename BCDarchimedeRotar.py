#!/usr/bin/env python3
# todo 
# test for resistance
import pybullet as p
import pybullet_data
import time
import numpy as np
import math 
p.connect(p.GUI, options='--background_color_red=0.1 --background_color_green=0.9 --background_color_blue=0.9')
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0, 0, 0 ])
playback = 0.03 # this is the delay between simulator steps. 

def get_positive_euler_from_quaternion(quat):
    # Convert the quaternion to Euler angles
    euler = np.array(p.getEulerFromQuaternion(quat))
    # Convert negative angles to the range [0, 2π)
    euler[euler < 0] += 2 * np.pi
    return euler

def print_masses(bodyUniqueId):
    # Print the base mass
    base_mass = p.getDynamicsInfo(bodyUniqueId, -1)[0]  # The mass is the first element of the tuple
    print("Base mass:", base_mass)
    # Get the number of joints (which is also the number of links, since each joint connects two links)
    num_joints = p.getNumJoints(bodyUniqueId)
    # Print the mass of each link
    for i in range(num_joints):
        link_name = p.getJointInfo(bodyUniqueId, i)[12].decode('utf-8')  # The link name is the 13th element of the tuple
        link_mass = p.getDynamicsInfo(bodyUniqueId, i)[0]  # The mass is the first element of the tuple
        print(f"Link {i} (name: {link_name}) mass:", link_mass)

def disable_drag(body):
    num_joints = p.getNumJoints(body)
    # if contactStiffness and contactDamping are not defined, the function will not change any of the dynamics for a specific link
    for i in range(-1, (num_joints)):
        print(f"Dynamics info for link {i}: {p.getDynamicsInfo(body, i)}")
        # Set the drag-related dynamics to 0
        p.changeDynamics(body, i, lateralFriction=0, spinningFriction=0, rollingFriction=0, linearDamping=0, angularDamping=0, contactStiffness=100, contactDamping=.1, restitution=1)
        print(f"Dynamics info for link {i}: {p.getDynamicsInfo(body, i)}")

def disable_joint_friction(body):
    for joint_index in range(0, p.getNumJoints(body)-1):
        p.setJointMotorControl2(bodyUniqueId=body, jointIndex=joint_index, controlMode=p.VELOCITY_CONTROL, force=0)

def no_collision(object1_body_id, object2_body_id):
    # Disable collision detection between all links of the two objects
    for link_index1 in range(-1, p.getNumJoints(object1_body_id)):
        for link_index2 in range(-1, p.getNumJoints(object2_body_id)):
            p.setCollisionFilterPair(object1_body_id, object2_body_id, link_index1, link_index2, enableCollision=0)

def torque_until_rpm_within_angle(bodyUniqueId, jointIndex, targetRPM, torqueValue, min_angle, max_angle):
    max_velocity_radians = targetRPM * (2 * np.pi) / 60  # Convert RPM to rad/s
    # Apply the torque until the joint reaches the desired speed and the angle is within range
    while True:
        joint_state = p.getJointState(bodyUniqueId, jointIndex)
        angle = joint_state[0]  # Joint angle is the first value in the tuple
        speed = joint_state[1]  # Joint velocity is the second value in the tuple
        # Conditions to break the loop: reach target RPM and angle within specified range
        if abs(speed) >= max_velocity_radians and min_angle <= angle <= max_angle:
            # Disable motor control to allow the joint to rotate freely
            p.setJointMotorControl2(bodyUniqueId, jointIndex, p.VELOCITY_CONTROL, force=0)
            break
        # Apply torque to the joint
        p.setJointMotorControl2(bodyUniqueId, jointIndex, p.TORQUE_CONTROL, force=torqueValue)
        step_inc() 

def print_orientation_and_y_position(bodyUniqueId, linkIndex):
    link_state = p.getLinkState(bodyUniqueId, linkIndex)
    quat = link_state[1]  # The world orientation of the link frame is the second element of the tuple
    euler = get_positive_euler_from_quaternion(quat)
    print(f'####### simlation step : ', sim_step, 'time : ', (sim_step * (1/240)), '########')
    print(f'Z-axis rotation (in radians):', round(euler[2],6), ' in degrees ', round(math.degrees(euler[2]),6))  # Z-axis rotation is the third element of the tuple
    joint_state = p.getJointState(bodyUniqueId, 0)
    speed_rad_s = joint_state[1]
    speed_rpm = (speed_rad_s * 60) / (2 * np.pi)
    print(f'RPMs: ', speed_rpm) 
    pos, _ = p.getBasePositionAndOrientation(bodyUniqueId)
    # Print the y-coordinate
    print("Y-coordinate:", round(pos[1],6))


def disable_drag_on_point_mass(point_mass_id):
    p.changeDynamics(point_mass_id, -1, lateralFriction=0, spinningFriction=0, rollingFriction=0, linearDamping=0, angularDamping=0, contactStiffness=100, contactDamping=.1, restitution=1)
    print(f"Dynamics info for point mass: {p.getDynamicsInfo(point_mass_id, -1)}")




sim_step = 0 # should only be accessed in step_inc(). However, I need it to be presistant. So using it as gobal in step_inc() 
def step_inc():
    global sim_step
    p.stepSimulation()
    sim_step += 1
    time.sleep(playback)  # Sleep to match the PyBullet simulation timestep
    return sim_step, sim_step * (1 / 240)

# load the plane
planeId = p.loadURDF("plane.urdf", basePosition=[0,0,-1],)

# Create the rail
rail_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 100, 0.02])
rail_id = p.createMultiBody(baseMass = 10000,  baseCollisionShapeIndex=rail_shape,
                            baseVisualShapeIndex=-1,
                            basePosition=[0,0,0],
                            baseOrientation=[0,0,0,1])

# Create the poimt mass
point_mass_radius = 0.1  # Radius close to zero
point_mass_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=point_mass_radius)
point_mass_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=point_mass_shape,
                                  baseVisualShapeIndex=-1,
                                  basePosition=[5, 0, 0],
                                  baseOrientation=[0, 0, 0, 1])

# Disable drag on the tip
disable_drag_on_point_mass(point_mass_id)

# load the bearing
bearing_id = p.loadURDF("bearing.urdf", basePosition=[0,0,0], baseOrientation=[0,0,0,1], useFixedBase=False)

# Fix the rail to the platform. The platform is unmovable in the simulator.
create_fixed_base_joint = True
if create_fixed_base_joint:
    fixed_base_joint = p.createConstraint(
      parentBodyUniqueId=planeId,
      parentLinkIndex=-1,
      childBodyUniqueId=rail_id,
      childLinkIndex=-1,
      jointType=p.JOINT_FIXED,  # Use prismatic joint for sliding on Ypos.
      jointAxis=[0, 1, 0],
      parentFramePosition=[0, 0, 1],
      childFramePosition=[0, 0, 0]
    )
# Create the rail to bearing joint. This is a prismatic joint. The joints allows the bearing system to slide on the rail on Y's axis. But constrains all other motion.
create_sliding_joint = True 
if create_sliding_joint:
    sliding_joint = p.createConstraint(
      parentBodyUniqueId=rail_id,
      parentLinkIndex=-1,
      childBodyUniqueId=bearing_id,
      childLinkIndex=-1,
      jointType=p.JOINT_PRISMATIC,  # Use prismatic joint for sliding on Ypos.
      jointAxis=[0, 1, 0],
      parentFramePosition=[0, 0, 0],
      childFramePosition=[0, 0, 0]
    )
    # remove friction for pristmatic joint. It seems this is done by removing drag. Not sure but it seems the joint itself doesn't have resistance.
    disable_drag(rail_id)
    disable_drag(bearing_id)
# Fix the tip to the arm/bearing.
create_tip_joint = True 
if create_tip_joint:
    tip_joint = p.createConstraint(
      parentBodyUniqueId=bearing_id,
      parentLinkIndex=0,
      childBodyUniqueId=point_mass_id,
      childLinkIndex=-1,
      jointType=p.JOINT_FIXED,  # Use prismatic joint for sliding on Ypos.
      jointAxis=[0, 0, 1],
      parentFramePosition=[2.5,0,0],
      childFramePosition=[0, 0, 0]
    )
# Fix the bearing to the platform. This is temporary just until we get to the correct RPMs.
create_fixed_bearing_joint = True
if create_fixed_bearing_joint:
    fixed_bearing_joint = p.createConstraint(
      parentBodyUniqueId=planeId,
      parentLinkIndex=-1,
      childBodyUniqueId=bearing_id,
      childLinkIndex=-1,
      jointType=p.JOINT_FIXED,  # Use prismatic joint for sliding on Ypos.
      jointAxis=[0, 1, 0],
      parentFramePosition=[0, 0, 1],
      childFramePosition=[0, 0, 0]
    )
# Turn off collision between objects
no_collision(rail_id, bearing_id)
no_collision(rail_id, point_mass_id)
disable_joint_friction(bearing_id)
print("bearing masses: ")
print_masses(bearing_id)
print("tip mass: ")
print_masses(point_mass_id)

# simulation starts here.
# apply torque until we get up to the correct RPM and at the correct angle:
print_orientation_and_y_position(bearing_id, 0)
torque_until_rpm_within_angle(bodyUniqueId=bearing_id, jointIndex=0, targetRPM=25, torqueValue=400, min_angle=1.59, max_angle=1.61)
# remove the contraints fixing the bearing on the rail.
p.removeConstraint(fixed_bearing_joint)
print("Bearing released!!")
print_orientation_and_y_position(bearing_id, 0)
# main loop of the simulation after release of the bearing.
counter = 0
while True:
     step_inc()
     counter += 1
     if counter % 100 == 0:
         print_orientation_and_y_position(bearing_id, 0)
