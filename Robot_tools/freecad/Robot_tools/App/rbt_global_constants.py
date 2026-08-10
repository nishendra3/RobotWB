"""Robot Constants.

Name: rbt_global_constants.py

Author: Carlo Dormeletti and Nishendra Singh
Copyright: 2026
Licence: LGPL 2.1
"""

# TODO: Check the case where user decides
# to rename the robot default names from
# below constants
ROBOT_FPO_NAME = "Robot_FPO"
ROBOT_ASSEMBLY_LABEL = "Robot_Assembly"
GROUNDED_JOINT_NAME = "GroundedJoint"   # REMOVE: Legacy Path
BASE_FRAME_NAME = "BaseFrame"


RBT_PREFS = "User parameter:BaseApp/Preferences/Mod/Robot_tools"

# stagger percentage for new parts added to asm (0-100)
# 0 = parts at their CAD file pose
# >0 = parts staggered away from prev added part
DEFAULT_INSERT_STAGGER_PCT = 0.0

# default robot joint speeds
DEFAULT_MAX_SPEEDS = {
    "revolute": 10,    # deg/sec at 100%
    "prismatic": 100,  # mm/sec at 100%
}

MM_PER_M = 1000.0
DEFAULT_SLIDER_TRAVEL_MM = 500

# trajectory feature
TRAJ_JSON_VERSION = 1        # Waypoints_json schema version
DEFAULT_TRAJ_SAMPLES = 24    # path preview samples per segment

DEFAULT_PTP_SPEED = 100.0    # % of full joint speed (ptp)
DEFAULT_LIN_SPEED_TCP = 100  # mm/s
DEFAULT_LIN_SPEED_ORI = 45   # deg/s (for reorientation of the tool)

LIN_IK_STEP_MM = 5  # solve at every 5 mm steps for linear path
LIN_IK_STEP_DEG = 5  # solve at every 5 deg tool reorientation

LIN_IK_STEPS_MAX = 100  # per-segment cap on the ik steps solved

LIN_JOINT_JUMP_DEG = 30  # max joint jump allowed to avoid config flip
LIN_JOINT_JUMP_MM = 30  # max linear motion allowed to avoid config flip

TRAJ_TICK_MS = 33  # update trajectroy frame every 33 millisec.

ap_clr = {
    "Black": [(0.0, 0.0, 0.0), "#000000"],
    "Grey75": [(0.25, 0.25, 0.25), "#404040"],
    "White": [(1.0, 1.0, 1.0), "#FFFFFF"],
    # Vibrant
    "V_Blue": [(0.0, 0.467, 0.733), "#0077BB"],
    "V_Cyan": [(0.20, 0.733, 0.933), "#33BBEE"],
    "V_Teal": [(0.0, 0.6, 0.533), "#009988"],
    "V_Orange": [(0.933, 0.467, 0.2), "#EE7733"],
    "V_Red": [(0.8, 0.2, 0.67), "#CC3311"],
    "V_Magenta": [(0.933, 0.2, 0.467), "#EE3377"],
    "V_Grey": [(0.733, 0.733, 0.733), "#BBBBBB"],
    # Bright
    "B_Blue": [(0.267, 0.467, 0.667), "#4477AA"],
    "B_Cyan": [(0.4, 0.8, 0.933), "#66CCEE"],
    "B_Green": [(0.133, 0.533, 0.200), "#228833"],
    "B_Yellow": [(0.8, 0.733, 0.267), "#CCBB44"],
    "B_Red": [(0.933, 0.4, 0.467), "#EE6677"],
    "B_Purple": [(0.667, 0.2, 0.467), "#AA3377"],
    # High Contrast
    "HC_Yellow": [(0.867, 0.667, 0.2), "#DDAA33"],
    "HC_Red": [(0.733, 0.333, 0.4), "#BB5566"],
    "HC_Blue": [(0.0, 0.267, 0.533), "#004488"],
    # Additional
    "Aqua": [(0.0, 1.0, 1.0), "#00FFFF"],
    "Orange": [(1.0, 0.647, 0.0), "#FFA500"],
}


DEFAULT_KIN_LIB = "numpy_dls"

PIP_HINTS = {
    "pinocchio": "pip install pin",
    "tesseract": "pip install tesseract-robotics",
    "ikpy": "pip install ikpy",
}
