# OBD-II Service/Mode 02, 03 and 04 emulation (STM32G431KB)
OBD2 diagnostic trouble codes (DTCs) and freeze frame data are emulated using STM32/HAL.

![OBD-II Service 02, 03 and 04 in action](/Assets/Images/obd2_service_03_in_action.jpg)

![OBD-II Service 03 in action with Surix](/Assets/Images/obd2_dtc_surix.jpg)

![OBD-II Service 02 in action with Surix](/Assets/Images/obd2_freeze_frame_surix.jpg)

![OBD-II Service 03 in action with Vgate](/Assets/Images/obd2_dtc_vgate.jpg)

![OBD-II Service 02 in action with Vgate](/Assets/Images/obd2_freeze_frame_vgate.jpg)

![OBD-II Service 03 in PCAN-View](/Assets/Images/pcan_obd2_dtc_all.JPG)

![OBD-II Service 03 in PCAN-View](/Assets/Images/pcan_obd2_dtc_current.JPG)

![OBD-II Service 03 in PCAN-View](/Assets/Images/pcan_obd2_freeze_frame.JPG)

Play with [Service 09](https://github.com/ufnalski/obd2_service_09_emulation_g431kb) to get familiar with CAN-TP (ISO-TP). Play with [Service 01](https://github.com/ufnalski/obd2_service_01_emulation_g431kb) to get familiar with vehicle parameter streaming.

# Missing files?
Don't worry :slightly_smiling_face: Just hit Alt-K to generate /Drivers/CMCIS/ and /Drivers/STM32G4xx_HAL_Driver/ based on the .ioc file. After a couple of seconds your project will be ready for building.

# OBD-II DTCs and freeze frames
* [OBD-II PIDs](https://en.wikipedia.org/wiki/OBD-II_PIDs) (Wikipedia)
* [Example 3: OBD2 Diagnostic Trouble Codes (DTCs)](https://www.csselectronics.com/pages/obd2-explained-simple-intro) (CSS Electronics)
* [OBD-II Trouble Codes](https://www.obd-codes.com/trouble_codes/)
* [Find your OBD-II Trouble Code](https://repairpal.com/obd-ii-code-chart) (RepairPal)
* [Getting Started with CAN Bus Sniffing / What Can I See via the OBD Port?](https://dauntlessdevices.com/getting-started-with-can-bus-sniffing-what-can-i-see-via-the-obd-port/) (Dauntless Devices)
* [Unlocking On-Board diagnostics: A Guide to OBD 2 Protocol](https://www.embien.com/automotive-insights/on-board-diagnostics-a-comprehensive-guide-to-obd-2-protocol) (Embien Technologies)
* [A handbook of OBD modes and services](https://www.embitel.com/wp-content/uploads/OBD-Handbook.pdf) (Embitel)
* [OBD Diagnostic Service (Mode) $02 - Request Powertrain Freeze Frame Data](https://x-engineer.org/obd-diagnostic-service-mode-02-request-powertrain-freeze-frame-data/)[^1] (x-engineer)
* [OBD Diagnostic Service (Mode) $03 - Request emission-related diagnostic trouble codes (DTCs)](https://x-engineer.org/obd-diagnostic-service-mode-03-request-emission-related-diagnostic-trouble-codes-dtcs/) (x-engineer)
* [OBD Diagnostic Service (Mode) $04 - Clear/reset emission-related diagnostic information](https://x-engineer.org/obd-diagnostic-service-mode-04-clear-reset-emission-related-diagnostic-information/) (x-engineer)

[^1]: Saved me a lot of time :upside_down_face:

# Call for action
Create your own [home laboratory/workshop/garage](http://ufnalski.edu.pl/control_engineering_for_hobbyists/2024_dzien_otwarty_we/Dzien_Otwarty_WE_2024_Control_Engineering_for_Hobbyists.pdf)! Get inspired by [ControllersTech](https://www.youtube.com/@ControllersTech), [DroneBot Workshop](https://www.youtube.com/@Dronebotworkshop), [Andreas Spiess](https://www.youtube.com/@AndreasSpiess), [GreatScott!](https://www.youtube.com/@greatscottlab), [ElectroBOOM](https://www.youtube.com/@ElectroBOOM), [Phil's Lab](https://www.youtube.com/@PhilsLab), [atomic14](https://www.youtube.com/@atomic14), [That Project](https://www.youtube.com/@ThatProject), [Paul McWhorter](https://www.youtube.com/@paulmcwhorter), [RS Elektronika](https://www.youtube.com/@RSElektronika), and many other professional hobbyists sharing their awesome projects and tutorials! Shout-out/kudos to all of them!

> [!WARNING]
> Control engineering - do try this at home :exclamation:

190+ challenges to start from: [Control Engineering for Hobbyists at the Warsaw University of Technology](http://ufnalski.edu.pl/control_engineering_for_hobbyists/Control_Engineering_for_Hobbyists_list_of_challenges.pdf).

Stay tuned!
