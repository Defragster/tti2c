# tti2c
Teensy tested i2c devices

1.	Lidar Lite v3 (LIDARLITE_ADDR_DEFAULT 0x62): https://github.com/garmin/LIDARLite_Arduino_Library/ (Note Modifed lib included in repository)
2.	BNO080 (BNO080_DEFAULT_ADDRESS 0x4B): https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library
3.	BNO055 (BNO055_ADDRESS_A 0x29): https://github.com/adafruit/Adafruit_BNO055
3.1	Also #include <Adafruit_Sensor.h> : https://github.com/adafruit/Adafruit_Sensor
4.	MPU9250 (MPU9250 address will be 0x68): https://github.com/bolderflight/MPU9250
5.	SSD1306 ( Display address will be 0x3C): https://github.com/adafruit/Adafruit_SSD1306
6.	MB85RC256V (FRAM address will be 0x50): https://github.com/adafruit/Adafruit_FRAM_I2C [ and FRAM chip has a special ID feature which appears at 0x7C.
7.  T3.2 (slave at address 0x09): https://github.com/madsci1016/Arduino-EasyTransfer (EasyTransfer library)

## Extended Libraries
1.	SHT31: https://github.com/adafruit/Adafruit_SHT31
2.	SFun Qwiic Button : https://github.com/sparkfun/SparkFun_Qwiic_Button_Arduino_Library
3.	SFun Qwiic Keypad : https://github.com/sparkfun/SparkFun_Qwiic_Keypad_Arduino_Library
4.  Slave Rx https://github.com/madsci1016/Arduino-EasyTransfer for : https://github.com/madsci1016/Arduino-EasyTransfer/tree/master/EasyTransferI2C/examples/EasyTransfer_RX_Example


## Alternate Libraries

Arduino library for I2C FRAM - Fujitsu MB85RC & Cypress FM24, CY15B,  https: //github.com/mjs513/FRAM_MB85RC_I2C. This is a modification of the original library so you can select which wire port you want to use.
