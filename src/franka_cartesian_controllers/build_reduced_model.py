import pinocchio as pin
import numpy as np
import os

urdf_path = "/home/danningzhao/franka-impedance-ws/src/franka_interactive_controllers/panda.urdf"
FRAME_NAME = "panda_EE"
JOINTS_TO_LOCK = ["panda_finger_joint1", "panda_finger_joint2"]
DEFAULT_CONFIGURATION = np.array([0, -0.151628, 0,
                                  -2.160299,0.0063, 2.0304,
                                  0.8428, 0.01, 0.01])
DEFAULT_CONFIGURATION2 = np.array([0, -0.151628, 0,
                                  -2.160299,0.0063, 2.0304,
                                  0.8428])
# === CONFIGURATION ===
# Path to your URDF file
# Change this to your actual URDF path
# Joints you want to lock (by name)

# === LOAD MODEL ===
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()
frame_id = model.getFrameId(FRAME_NAME)

print("Original model:")
print(f"- Number of joints: {model.njoints}")
print(f"- Number of DOFs:   {model.nv}")
pin.forwardKinematics(model, data, DEFAULT_CONFIGURATION)
pin.updateFramePlacements(model, data) 
ee_pose = data.oMf[frame_id]
ee_pos = ee_pose.translation
ee_ort = pin.Quaternion(ee_pose.rotation)
print(ee_pose)
# === GET REFERENCE CONFIGURATION ===
# You can also use any valid q0, e.g., from an SRDF or manually created
#q0 = pin.neutral(model)  # Default neutral configuration

# === GET JOINT IDs TO LOCK ===
joint_ids_to_lock = [model.getJointId(name) for name in JOINTS_TO_LOCK]

# === BUILD REDUCED MODEL ===
#reduced_model = pin.buildReducedModel(model, joint_ids_to_lock, q0)
reduced_model = pin.buildReducedModel(model, joint_ids_to_lock, DEFAULT_CONFIGURATION)
reduced_data = reduced_model.createData()
frame_id = reduced_model.getFrameId(FRAME_NAME)
pin.forwardKinematics(reduced_model, reduced_data, DEFAULT_CONFIGURATION2)
pin.updateFramePlacements(reduced_model, reduced_data) 
ee_pose = reduced_data.oMf[frame_id]
ee_pos = ee_pose.translation
ee_ort = pin.Quaternion(ee_pose.rotation)

print("\nReduced model (after locking joints):")
print(f"- Number of joints: {reduced_model.njoints}")
print(f"- Number of DOFs:   {reduced_model.nv}")
print(ee_pose)
# === OPTIONAL: Show joint names ===
print("\nRemaining joint names:")
for i, name in enumerate(reduced_model.names):
    print(f"{i}: {name}")
