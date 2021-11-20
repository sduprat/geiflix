# Function 5 - Motor & Direction 

The function is build around a evalboard Nucleo-F103 from ST. Firmware code can be found in firmware directory.

RQ: Firmware has been converted to STMCubeIDE v7 since 09/2021. 

## WHAT IS AVAILABLE

This function retrieves data from the Raspi through the CAN bus. Two types of CAN frames can be used with this function (either one or the other) : 
*Control Motor Command (CMC)* frames, of ID 0x010
*Speed & Steering Commands (SSC)* frames, of ID 0x020
More information is available about the CAN and the content of these frames in the Git at geiflix/documentation/software/networks/CAN/Can Bus.md

* The CMC mode allows to command PWM directly.
* The SSC mode allows to command the propulsion and direction directly from a given speed and steering angle. A differential between left and right wheels has been implemented for more effective turns.

## HOW-TO FOR THE FIRST USE OF SSC MODE  

0. Use STM32CubeIDE. In main.c, define MODE to 0 (calibration). Build and go to Debug mode. Open calibrate.c and scroll down.
1. Run the program. Let the calibration program run for about 10 seconds.
2. Put the wheels in line to roll straight with the buttons from the dashboard. Once the wheels are properly aligned, press the blue button of the NucleoF103RB.
3. Put a breakpoint at the indicated location.
4. Observe the values of capt_when_straight and capt_when_right thanks to right-click then "Watch Add Expression".
5. Switch to Debug view to observe these values in the Expressions tab.
6. Report these values in the #define of the same name in steering.c.
7. In main.c, switch MODE to 2 to resume activity in normal SSC mode. 

CONGRATULATIONS, you have completed the calibration!
