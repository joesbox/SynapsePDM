# SynapsePDM
### CAN Enabled Automotive Power Distribution Module

#### Features (WIP):
+ 14 High Side Driver output channels configurable as PWM (200Hz) or digital with soft-start/soft-stop capability.
+ Up to 17A output per channel. Channels can be paired to increase current output.
+ Configurable load current and channel diagnostics.
+ Colour LCD interface.
+ 8 digital inputs, active high, active low, one-time pulse or simple on/off.
+ 8 configurable 5V analogue/digital inputs with switchable pull-up or pull-down resistors.
+ CAN channel control and diagnostics.
+ SD card logging.
+ Configuration and monitoring via USB.
+ LiPO Battery-backup with charging and state-of-charge management IC.
+ 6-DOF on-board IMU.
+ SIM card slot for tracking and telemetry.

## Display Configuration
This project uses the TFT_eSPI library. The display configuration for which is in User_Setup.h in the src folder. Once you have built the project and pulled in the library, you need to overwrite the User_Setup.h file in libdeps\STM32\TFT_eSPI. I have opened an issue for this and will work out how to get the library to point at the custom User_Setup.h file without the need to manually copy it.

## License
This work is licensed under a GPL-3.0 license.
