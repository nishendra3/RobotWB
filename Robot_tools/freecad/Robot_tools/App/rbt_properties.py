ROBOT_SCHEMA = [

    # core properties
    ("RobotAssembly", "App::PropertyLinkGlobal",
     "Robot", "Robot assembly"),
    ("RobotJoints", "App::PropertyLinkListGlobal",
     "Robot", "Robot joints list"),
    ("RobotLinks", "App::PropertyPlacementList",
     "Robot", "Robot links list"),
    ("RobotJointsCfg", "App::PropertyString", "Robot",
     "per-joint config json: {joint: {dir, zero, home}}"),

    # tool handling properties
    ("Tools", "App::PropertyLinkListGlobal",
     "Tools", "Tool FPOs attached"),
    ("ActiveTool", "App::PropertyLinkGlobal",
     "Tools", "Currently active tool"),

    # kinematics properties
    ("KinematicsLib", "App::PropertyEnumeration",
     "Kinematics", "FK/IK solver"),
    ("PtpSpeedDefault", "App::PropertyFloatConstraint", "Kinematics",
     "default PTP speed for new waypoints (% of max)"),
    ("LinSpeedDefault", "App::PropertyFloatConstraint", "Kinematics",
     "default LIN TCP speed for new waypoints (mm/s)"),
    ("SpeedOverride", "App::PropertyFloatConstraint", "Kinematics",
     "global speed scaling 1-100 %"),

    # placement properties
    ("BasePlacement", "App::PropertyPlacement", "Placement",
     "World -> robot base frame"),
    ("BaseOffset", "App::PropertyPlacement", "Placement",
     "base frame in base-link coords "
     "(moves the frame label, not the robot)"),

    # trajectory properties
    # hidden scope for "Trajectories" below to prevent dependency loop
    ("Trajectories", "App::PropertyLinkListHidden", "Trajectory",
     "Trajectories attached to this robot"),
]

TRAJ_SCHEMA = [
    ("Robot", "App::PropertyLinkGlobal", "Trajectory",
     "Robot this trajectory drives"),

    ("WaypointsJson", "App::PropertyString", "Trajectory",
     "(hidden) serialised waypoints, edit via the trajectory panel"),

    ("WaypointCount", "App::PropertyInteger", "Trajectory",
     "(read only) number of waypoints"),

    ("PreviewSamples", "App::PropertyInteger", "Display",
     "Path preview samples per segment between two taught points"),
]

JNT_KINE_PROPS = ("MaxSpeed", "AngleMin", "AngleMax",
                  "LengthMin", "LengthMax")
SPEED_PROPS = ("PtpSpeedDefault", "LinSpeedDefault", "SpeedOverride")
KINE_PROPS = SPEED_PROPS + ("KinematicsLib",)

TRAJ_PROPS = {n for n, *_ in TRAJ_SCHEMA}


# old docs migration #
RENAMED_PROPS = {
    "Ptp_speed_default": "PtpSpeedDefault",
    "Lin_speed_default": "LinSpeedDefault",
    "Speed_override": "SpeedOverride",
    "Kinematics_lib": "KinematicsLib",
    "Base_placement": "BasePlacement",
    "Base_offset": "BaseOffset",
    "Robot_assembly": "RobotAssembly",
    "Robot_joints": "RobotJoints",
    "Robot_links": "RobotLinks",
    "Robot_joints_cfg": "RobotJointsCfg",
    "Active_tool": "ActiveTool",
}
