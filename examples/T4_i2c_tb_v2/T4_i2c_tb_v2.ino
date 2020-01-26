#include <Wire.h>
#define _use_9250
#define _use_BNO055
#define _use_BNO080
#define _use_lidar
//#define _use_MB85
#define _use_ssd1306
const TwoWire *DisplayWire = &Wire1;

#define _MPU9250_port Wire
#define _BNO055_port  Wire
#define _BNO080_port  Wire1
#define _LIDAR_port   Wire
#define _MB85_port    Wire

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

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

#if defined ( _use_ssd1306 )
  //DisplayWire->begin();
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }
  delay(500);
#if defined ( _use_ssd1306 )
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0, 0);            // Start at top-left corner
  display.println(F("Hello"));
  display.display();
#endif
#endif

#if defined(_use_9250)
  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU 9250 initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
#if defined ( _use_ssd1306 )
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("9250 Initialized"));
  display.display();
#endif
#endif

#if defined(_use_BNO055)
  /* Initialise the sensor */  
    //max
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
#if defined ( _use_ssd1306 )
  //display.clearDisplay();
  display.setTextSize(1);
  display.println(F("055 Initialized"));
  //display.setCursor(0, 0);
  display.display();
#endif
#endif

#if defined(_use_BNO080)
  if (myIMU.begin(BNO080_DEFAULT_ADDRESS, _BNO080_port) == false)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz   //max

  myIMU.enableRotationVector(50); //Send data update every 50ms

  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Output in form i, j, k, real, accuracy"));

#if defined ( _use_ssd1306 )
  display.setTextSize(1);
  //display.setCursor(0, 0);
  display.println(F("080 Initialized"));
  display.display();
#endif
#endif

#if defined( _use_lidar)
  //myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  _LIDAR_port.begin();
  Wire.setClock(400000UL);  //max

  myLidarLite.configure(0); // Change this number to try out alternate configurations
#if defined ( _use_ssd1306 )
  display.setTextSize(1);
  //display.setCursor(0, 0);
  display.println(F("LL3 Configured"));
  display.display();
#endif
#endif
}

void loop()
{
#if defined( _use_9250)
  loop9250();
  Serial.println();
#endif

#if defined(_use_BNO055)
  loop055();
  Serial.println();
#endif

#if defined(_use_BNO080)
  loop080();
  Serial.println();
#endif

#if defined(_use_lidar)
  looplidar();
  Serial.println();
#endif
}

#if defined(_use_9250)
void loop9250() {
  // read the sensor
  IMU.readSensor();

  // display the data
  Serial.print("MPU9250: ");
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print(", ");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print(", ");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print(", ");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print(", ");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print(", ");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print(", ");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print(", ");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print(", ");
  Serial.println(IMU.getMagZ_uT(),6);
  //Serial.print(", ");
  //Serial.println(IMU.getTemperature_C(),6);
  delay(20);
}
#endif

#if defined(_use_BNO055)
void loop055(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  Serial.print("BNO055: ");
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\n");

#if defined ( _use_ssd1306 )
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("BNO055: ");
  display.print(euler.x());
  display.print(",");
  display.print(euler.y());
  display.print(", ");
  display.print(euler.z());
  
  display.display();
#endif

  /*
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  */

  /* Display calibration status for each sensor. */
  /*uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
  */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
#endif

#if defined(_use_BNO080)
void loop080()
{
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

    Serial.print("BNO080: ");
    Serial.print(quatI, 2);
    Serial.print(F(","));
    Serial.print(quatJ, 2);
    Serial.print(F(","));
    Serial.print(quatK, 2);
    Serial.print(F(","));
    Serial.print(quatReal, 2);
    Serial.print(F(","));
    Serial.print(quatRadianAccuracy, 2);
    Serial.print(F(","));

    Serial.println();
  }
}
#endif

#if defined(_use_lidar)
void looplidar()
{
  /*
    distance(bool biasCorrection, char lidarliteAddress)

    Take a distance measurement and read the result.

    Parameters
    ----------------------------------------------------------------------------
    biasCorrection: Default true. Take aquisition with receiver bias
      correction. If set to false measurements will be faster. Receiver bias
      correction must be performed periodically. (e.g. 1 out of every 100
      readings).
    lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
  */

  // Take a measurement with receiver bias correction and print to serial terminal
  Serial.print("LidarLiteV3: "); 
  Serial.println(myLidarLite.distance());
  Serial.print("LidarLiteV3: "); 
  // Take 99 measurements without receiver bias correction and print to serial terminal
  for (int i = 0; i < 99; i++)
  {
    int lld = myLidarLite.distance(false);
    static int lldL = 0;
    if ( lld > lldL + 2 || lld < lldL - 2 ) {
      Serial.print(lld);
      if ( !(i % 20 ) )
        Serial.print("\n");
      else
        Serial.print(", ");
      lldL = lld;
    }
  }
  Serial.print("\n");
}
#endif
