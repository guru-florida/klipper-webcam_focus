# Webcam Focus for Klipper

## Description

_Having AutoFocus issues with your webcam?_
This plugin can set manual focus using the printer's XY coordinates.

Also control other webcam settings such as focus, white balance, aperature speed and gain using gcode macros.

## Disclaimer
**This is work in progress and currently in "alpha" state.**

If you encounter any problems, feel free to open an issue.

If you want to help by testing or contributing, please contact me on Discord: guru-florida#4637


## Demonstration

You can see the plugin in action and performing calibration on my YouTube video:
[Klipper Webcan Focus using Toolhead position](https://www.youtube.com/watch?v=wuBYe9llHTk)


## Installation

The module can be installed into a existing Klipper installation with an install script. 

    cd ~
    git clone https://github.com/guru-florida/klipper-webcam_focus.git
    cd klipper-webcam_focus
    ./install-webcam_focus.sh

## Configuration

To enable the plugin add [webcam] section to your printer.cfg. You should also add these config settings:

    [webcam_focus]
    camera_position: 250, 0    (where your camera is located X, Y, Z)
    focal_axis: 0, 1           (what axis should contribute to distance calculation, here only Y since my camera is facing directly down the Y axis)
    focus_mappings: 60.0:220, 80.0:140, 100.0:110, 120.0:100, ... (more focal calibration points)

You should use the WEBCAM_FOCUS_CALIBRATE gcode macro with a (paper) printed calibration pattern to
automatically determine the focus_mappings parameter.


## GCode Commands

The plugin adds these gcode commands:

### WEBCAM_SETTINGS
Set settings on your webcam

### WEBCAM_FOCUS_CALIBRATE [Y_MIN=] [Y_MAX=] [Y_STEP=] [MOVE_SPEED=]
Calibrate focus automatically using a focus pattern on the toolhead. Ensure your printer area is clear as the toolhead will move during this calibration. (Beta) For now this only works along the Y axis.

### WEBCAM_FOCUS_CLEAR
Clear any remembered focal points.

### WEBCAM_FOCUS_SAVE
Add the current focus and position into the focus mapper.

### WEBCAM_FOCUS_MAPPER
Show the points in the focus mapper.


## Mentions

Thank you to Julian Schill as his code was a big help in learning how to write a Klipper plugin
[Klipper LED Effect](https://github.com/julianschill/klipper-led_effect)

