# Robot Tools

A [FreeCAD](https://www.freecad.org/) workbench for creating, kinematically
analyzing, and interacting with robots.

## Features

- **Create Robot Object** : wraps an Assembly as a robot that allows us to track its joints angles & reachability.
- **Animate Robot** :  interactive joint posing with per-joint slider
- **Add Tool** :  Create and add new tools to the robot & modify its TCP

## Requirements

- FreeCAD
- Python 3.11 (bundled with FreeCAD)
- Kinematic Libraries : ikpy or pinocchio

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

Open a robot **Assembly** document, then use the toolbar buttons in order:

1. **Create Robot Object**
2. **Animate Robot**
3. **Add a Tool & TCP**

## License

LGPL 2.1


