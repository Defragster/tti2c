#include <Wire.h>
#if defined(__IMXRT1062__)
#include "Watchdog_t4.h"
WDT_T4<WDT2> wdt;
#endif
#include "Wire_Scanner_all.ino.h"

#define _use_9250
#define _use_BNO055
#define _use_BNO080
#define _use_lidar
#define _use_MB85
//#define _write_init_MB85
uint16_t writeaddress = 0x025;

#define _use_ssd1306
TwoWire *DisplayWire = &Wire1;

#define _MPU9250_port Wire
#define _BNO055_port  Wire
#define _BNO080_port  Wire
#define _LIDAR_port   Wire
#define _MB85_port    Wire

int idisp9250 = 0;
int idisp055 = 1;
int idisp080 = 0;
void myCallback() {
  Serial.println("FEED THE DOG SOON, OR RESET!");
  idisp9250 = 1;
  idisp055 = 0;
  idisp080 = 0;
}

#include "configDevices.h"

void ToggleClock0( int iCmd ) {
  Serial.print("TC #");
  Serial.print(iCmd);
  if ( !iCmd ) return; // Pass in ZERO to have exit with no action
  pinMode( 18, INPUT );
  pinMode( 19, OUTPUT );
#define ICNT 32
  Serial.print("\tTC SDA read: ");
  for ( int ii = 0; ii < ICNT; ii++ ) {
    digitalWriteFast(19, ii % 2 );
    Serial.print( digitalReadFast(18) );
    Serial.print( " " );
    delayMicroseconds(3);
  }
  Serial.print("\n");
}

void printSSD( int xx, int yy, const char * szOut, int tSize ) {
#if defined ( _use_ssd1306 )
  if ( xx == 0 && yy == 0 ) {
    display.clearDisplay();
    Serial.println();
  }
  Serial.print(szOut);
  display.setTextSize(tSize);
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  if ( xx >= 0 && yy >= 0 ) display.setCursor(xx, yy);
  display.print(szOut);
  display.display();
#endif
}

/*
   =========================================
   Setup Devices now
   ========================================
*/
void setup() {
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("\n" __FILE__ " " __DATE__ " " __TIME__);
  Scansetup(); // setup() :: Wire_Scanner_all.ino.h
  Scanloop(); // one loop() :: Wire_Scanner_all.ino.h
#if defined(__IMXRT1062__)
  WDT_timings_t config;
  config.trigger = 10; /* in seconds, 0->128 */
  config.timeout = 20; /* in seconds, 0->128 */
  config.callback = myCallback;
  wdt.begin(config);
#endif

  ToggleClock0( 1 );
#if defined( _use_MB85)
  Serial.println("Starting MB85...");

  mymemory.begin();
  //_MB85_port.setClock(1000000);
  //mymemory.eraseDevice();

#if defined( _write_init_MB85)
  //---------init data - load array
  byte arraySize = sizeof(MYDATA_t);
  mydata.datastruct.data_0 = true;
  Serial.print("Data_0: ");
  if (mydata.datastruct.data_0) Serial.println("true");
  if (!mydata.datastruct.data_0) Serial.println("false");
  mydata.datastruct.data_1 = 1.3575;
  Serial.print("Data_1: ");
  Serial.println(mydata.datastruct.data_1, DEC);
  mydata.datastruct.data_2 = 314159L;
  Serial.print("Data_2: ");
  Serial.println(mydata.datastruct.data_2, DEC);
  mydata.datastruct.data_3 = 142;
  Serial.print("Data_3: ");
  Serial.println(mydata.datastruct.data_3, DEC);
  mydata.datastruct.data_4 = 0x50;
  Serial.print("Data_4: 0x");
  Serial.println(mydata.datastruct.data_4, HEX);

  //string test
  String string_test = "The Quick Brown Fox Jumped";
  char cbuff[string_test.length() + 1];
  string_test.toCharArray(cbuff, string_test.length() + 1);
  for (uint8_t j = 0; j < string_test.length() + 1; j++) {
    mydata.datastruct.data_5[j] = cbuff[j];
  }
  Serial.println(string_test);

  Serial.println("...... ...... ......");
  Serial.println("Init Done - array loaded");
  Serial.println("...... ...... ......");

  //----------write to FRAM chip
  byte result = mymemory.writeArray(writeaddress, arraySize, mydata.I2CPacket);
  if (result == 0) Serial.println("Write Done - array loaded in FRAM chip");
  if (result != 0) Serial.println("Write failed");
  Serial.println("...... ...... ......");

#endif //write_init_data
#endif

#if defined ( _use_ssd1306 )
  //DisplayWire->begin();
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }
  delay(500);
  printSSD( 0, 0, ("Hello\n"), 2 );

#endif

#if defined(_use_9250)
  // start communication with IMU
  printSSD( 0, 0, ("9250 Init"), 1 );
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU 9250 initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  // setting the accelerometer full scale range to +/-8G
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
  printSSD( -1, -1, ("ialized\n"), 1 );
#endif

#if defined(_use_BNO055)
  /* Initialise the sensor */
  printSSD( -1, -1, ("055 Init"), 1 );
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  printSSD( -1, -1, ("ialized\n"), 1 );

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
#endif

#if defined(_use_BNO080)
  printSSD( -1, -1, ("080 Init"), 1 );
  delay(200);
  _BNO080_port.begin();
  if (myIMU.begin(0x4B, _BNO080_port) == false)
  {
    Serial.println("\tBNO080 not detected at default I2C address. Check your jumpers and the hookup guide. TOGGLE...");
    if ( 1 ) {
      printSSD( -1, -1, ("... TOGGLE\n"), 1 );
      ToggleClock0( 1 );
      if (!bno.begin())
      {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("\tOoops, no BNO080 detected ... Check your wiring or I2C ADDR!");
        while (1);
      }
      printSSD( -1, -1, ("080 Init"), 1 );
      _BNO080_port.begin();
      if (myIMU.begin(0x4B, _BNO080_port) == false)
      {
        Serial.println("\tBNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
        printSSD( -1, -1, ("... FAIL"), 1 );
        while (1);
      }
      else {
        printSSD( -1, -1, ("\nRecovered\n"), 1 );
        Serial.print("\nRecovered << ============================================================= \n");
        Serial.print("\nRecovered << ============================================================= \n");
        delay(5000);
      }
    }
    else
      while (1);
  }

  _BNO080_port.setClock(400000); //Increase I2C data rate to 400kHz   //max

  myIMU.enableRotationVector(50); //Send data update every 50ms

  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Output in form i, j, k, real, accuracy"));

  printSSD( -1, -1, ("ialized\n"), 1 );
#endif

#if defined( _use_lidar)
  printSSD( -1, -1, ("LL3 Init"), 1 );
  //myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  _LIDAR_port.begin();
  Wire.setClock(400000UL);  //max

  myLidarLite.configure(0); // Change this number to try out alternate configurations
  printSSD( -1, -1, ("ialized C+\n"), 1 );
  delay(1000);
#endif
}

void loop()
{
  static uint32_t feed = millis();
  if ( millis() - feed > 21000 ) {
    feed = millis();
#if defined(__IMXRT1062__)
    wdt.feed(); /* feed the dog every 11 seconds, to exceed 10second timeout period to refresh callback and gpio state for repeat */
#endif
    idisp055 = 1;
    idisp9250 = 0;
    idisp080 = 0;
  } else if (millis() - feed > 16000) {
    idisp055 = 0;
    idisp9250 = 0;
    idisp080 = 1;
  }


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

#if defined( _use_MB85)
  loopMB85();
  Serial.println();
#endif

}
