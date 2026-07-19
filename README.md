These are basic sketches for the Seeed Studio XIAO nRF52840 Sense (https://www.seeedstudio.com/Seeed-XIAO-BLE-Sense-nRF52840-p-5253.html).
Some of these will work with other nRF52840 boards as well as the Arduino Nano 33 BLE Sense Rev1 and Rev2, and others.  One or two #defines
near the top of the sketches may need to be enabled/disabled for specific board types.

Note that not all combinations of boards have been tested with these sketches.  Please contact me if there are problems.

I put these here because while it's almost certain that sketches like these have been out there for years, a Web search has real trouble
finding them, especially with AI. ;-)

1. Seeed_Studio_XIAO_nRF52840_Blink1: Blink example sketch modified to work with these boards.
2. Seeed_Studio_XIAO_nRF52840_Sense_RGB_Cycle_Colors1: Fancy colorful version of Blink for boards with the RGB LEDs. ;-)
3. Seeed_Studio_XIAO_nRF52840_Sense_Sensor_Test5: Exercise the sensors with data and visual displays.  Use the next one instead. ;-)
4. Seeed_Studio_XIAO_nRF52840_Sense_Sensor_External_RGB_Test5: Similar to above but adds options for external RGB LEDs on D0-D2.
5. Seeed_Studio_XIAO_nRF52840_Sense_RGB_LED_Color_via_Bluetooth1.
6. Seeed_Studio_XIAO_nRF52840_Sense_Bluetooth_Send_Test2.  Send Gyro data to iOS device via Bluetooth
7. Arduino_BLE_Central_LED_Control2 and Arduino_BLE_Peripheral_LED_Control2: Mating pair to show example of Bluetooth
   data transfer between boards.
8. Bluetooth_Send_Gyro_Data7.ino and Bluetooth_Receive_Gyro_Data9.ino: Mating pair to show perhaps
   semi-useful exmaple of Bluetooth gyroscope data transfer between boards.

Written/modified by Samuel M. Goldwasser.  Copyright® (if any) 1994-2025 and additional details in the sketch headers.
