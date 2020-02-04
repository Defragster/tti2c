
#if defined(_use_MB85)
#include <math.h>

#include <FRAM_MB85RC_I2C.h>
//define a struct of various data types
typedef struct {
  bool data_0;
  float data_1;
  long data_2;
  int data_3;
  byte data_4;
  char data_5[32];
} MYDATA_t;

//define a struct joining MYDATA_t to an array of bytes to be stored
typedef union {
  MYDATA_t datastruct;
  uint8_t I2CPacket[sizeof(MYDATA_t)];
} MYDATA4I2C_t;

MYDATA4I2C_t mydata; //data to be written in memory
MYDATA4I2C_t readdata; //data read from memory

//random address to write from
uint16_t writeaddress2 = (256 * 128) - 80; // calculated regarding density to hit more or less the end of memory map
byte resultw, resultr;

//Creating object for FRAM chip
FRAM_MB85RC_I2C mymemory(_MB85_port);
#endif

#if defined ( _use_ssd1306 )
#include <Adafruit_SSD1306.h>
//#include <splash.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, DisplayWire, OLED_RESET);

#endif

#if defined( _use_9250)
#include "MPU9250.h"
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(_MPU9250_port, 0x68);
int status;
#endif

#if defined( _use_BNO055)
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &_BNO055_port);
#endif

#if defined( _use_BNO080)
#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 myIMU;
#endif

#if defined( _use_lidar)
#include <LIDARLite.h>
LIDARLite myLidarLite;
#endif

#if defined(_use_QKEYPAD)  //0x4b
#include <SparkFun_Qwiic_Keypad_Arduino_Library.h>
KEYPAD qkeypad; //Create instance of this object
#endif

#if defined(_use_QBTN1) || defined(_use_QBTN2)
#include <SparkFun_Qwiic_Button.h>
#if defined(_use_QBTN1)
QwiicButton  qbtn1;
#endif
#if defined(_use_QBTN2)
QwiicButton  qbtn2;
#endif
#endif

#if defined(_use_SHT31)
#include "Adafruit_SHT31.h"
Adafruit_SHT31 sht31 = Adafruit_SHT31(&_SHT31_port);
#endif

#if defined(_use_T32)
//create object
  EasyTransferI2C ET; 
  
  //use for T3.2 slave
  struct SEND_DATA_STRUCTURE{
    //put your variable definitions here for the data you want to send
    //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
    int16_t blinks;
    int16_t pause;
  };
  
  //give a name to the group of data
  SEND_DATA_STRUCTURE slavedata;
  
  //define slave i2c address
  #define I2C_SLAVE_ADDRESS 9
#endif
