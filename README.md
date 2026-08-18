# Robot Tools

A [FreeCAD](https://www.freecad.org/) workbench for creating, kinematically
analyzing, and interacting with robots.

Full documentation: **[project wiki](https://github.com/nishendra3/RobotWB/wiki)**

## Features

- **Define Robot** : wraps an Assembly as a robot in a 4-step wizard; tracks joint angles & reachability
- **Animate Robot** : interactive joint posing with per-joint sliders, home and zero poses
- **Define Tool** : create and add tools to the robot & modify its TCP; drag the TCP for IK posing
- **Multi-Robot Control** : jog all robots in the document from one panel
- **Robot Trajectory** : teach waypoints and play them as a trajectory
- **Trajectory Player** : play trajectories of all robots on one clock

## Requirements

- FreeCAD (with the built-in Assembly workbench)
- Python 3.11 (bundled with FreeCAD)
- Kinematic libraries: none needed for the default backend; optional: pinocchio, tesseract-robotics, ikpy

## Installation

### Stable version

Use the latest release and download the zip file and install in your FreeCAD `Mod` folder.

### development version
Copy the `freecad/Robot_tools` directory into your FreeCAD `Mod` folder:
`[FreeCAD user dir]/Mod/Robot_tools/freecad/Robot_tools`

Restart FreeCAD. The **Robot Tools** toolbar appears in the GUI.

> Find your user dir via **Edit → Preferences → General**, or the Python
> console: `App.getUserAppDataDir()`.

## Usage

Use the toolbar buttons in order:

1. **Define Robot** — build the robot from CAD parts
2. **Animate Robot** — jog the joints
3. **Define Tool** — add a tool & TCP
4. **Robot Trajectory** — teach and play waypoints

See the [wiki](https://github.com/nishendra3/RobotWB/wiki) for the full guide.

## License

LGPL 2.1


