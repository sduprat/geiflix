# Function 4 - GPS/IMU 

The function is build around a evalboard Nucleo-L473 from ST. Firmware code can be found in firmware directory

## Firmware

Firmware code is split between drivers parts (in Drivers/BSP directory) and application (in Core directory). Every driver is initialized in main function (I/O, Interruptions, memory); Dynamic part of the application can be found in scheduler.c. Scheduler is cooperative and launch periodic tasks, roughtly one for each sensor, one for GPS, one for AHRS and one for printing on virtual com port over USB.
Shared variables (like latest values from sensors) can be found in globvar.c

## Sensors
All sensors are accessed via I2C. Available sensors are 
- accelerometer (values given in milli g (mg))
- gyroscope (values given in milli degree per seconde (mdps))
- magnetometer (values given in milli Gauss (mG))
- Humidity sensor (values given in percent)
- Pressure (values given in hectoPascal (hPa))
- temperature (values given in Celsius degree (degC))

File iks01a2.c/h serve as top level API for accessing sensors

## GPS
GPS is accessed via uart1. Driver can be found in Drivers/BSP/teseo_liv3f. Driver is based on C++ code found on arduino website and as be cut down and converted to C. There is a huge number of function.
In fact, in this basic application, only initialisation, update and position gathering are used.
Raw frame from GPS give position in DM.mmm format (decimal degree minute), that is not very used. This format is given by function TESEO_GetCoords_DMM.
An other one is provided, TESEO_GetCoords_DD, which return coordinates in format DD.dddd (degree and sub degree), more likely usefull.
Both format can be used on google maps

## CAN
CAN driver (found in drivers/BSP/can) is currently not enabled (init is commented out in main function) because, whitout hardware transceiver, the interrupt handler was overwhelming CPU (spurious interrupt). Code is based on ST example and should work. Currently, no ID and no frame are defined.

## AHRS
AHRS (for Attitude and Heading Reference System) is a sensor fusion algorithm that aim to provide a compass out of accelerometer, gyroscope and magnetometer. It is based on AHRS from function 3, but with a code heavily reworked. Sensor access functions have been removed because sensors are differents from function 3 and 4 but also because it's not the place.
Currently, AHRS is not working, probably because values from sensors are not in correct format or maybe update function is called not often enough. I don't have time to investigate anymore, feel free to:
- found out bugs and correct them
- throw away code and use better algorithms