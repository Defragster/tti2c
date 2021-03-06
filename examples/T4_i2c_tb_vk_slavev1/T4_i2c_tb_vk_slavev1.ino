#include <Wire.h>
#if defined(__IMXRT1062__)
#include "Watchdog_t4.h"
WDT_T4<WDT2> wdt;
#endif
#include <EasyTransferI2C.h>
#include "Wire_Scanner_all.ino.h"

#define _use_9250
#define _use_BNO055
#define _use_BNO080
#define _use_lidar
#define _use_MB85
//#define _write_init_MB85
uint16_t writeaddress = 0x025;
#define _use_QKEYPAD  0x4b
#define _use_QBTN1    0x6f
//#define _use_QBTN2  0x6f
#define _use_SHT31    0x44
#define _use_T32

#define _use_ssd1306
TwoWire *DisplayWire = &Wire1;

#define _MPU9250_port Wire
#define _BNO055_port  Wire
#define _BNO080_port  Wire
#define _LIDAR_port   Wire
#define _MB85_port    Wire
#define _QKEY_port    Wire1
#define _QBTN1_port   Wire1
#define _QBTN2_port   Wire1
#define _SHT31_port   Wire
#define _T32_port     Wire

int loopCount = 0;
uint8_t printTest = 1;

//int idisp9250 = 0;
//int idisp055 = 1;
//int idisp080 = 0;
typedef enum _disp_data_indexes {
#if defined(_use_9250)
  DISPLAY_9250,
#endif
#if defined(_use_BNO055)
  DISPLAY_BNO055,
#endif
#if defined(_use_BNO080)
  DISPLAY_BNO080,
#endif
#if defined(_use_lidar)
  DISPLAY_lidar,
#endif
#if defined(_use_MB85)
  DISPLAY_MB85,
#endif
#if defined(_use_SHT31)
  DISPLAY_SHT31,
#endif
#if defined(_use_T32)
  DISPLAY_T32,
#endif
  DISPLAY_FIELD_COUNT
} DISP_DATA_INDEX_t;

int disp_data_index = (DISP_DATA_INDEX_t)0;
elapsedMillis disp_data_elapsed = 0;
#define FIELD_DISPLAY_TIME 2500
bool hold_display_field = false;

//=======================
void myCallback() {
  Serial.println("FEED THE DOG SOON, OR RESET!");
  disp_data_index = (DISP_DATA_INDEX_t)0;
}

#include "configDevices.h"

void ToggleClock0( int iCmd ) {
  return;
  if ( !iCmd ) return;
  if ( 2 <= iCmd ) {
    // TBD
  }
  Serial.print("TC #");
  Serial.println(iCmd);

  pinMode( 18, INPUT );
  pinMode( 19, OUTPUT );
  for ( int ii = 0; ii < 32; ii++ ) {
    digitalWriteFast(19, ii % 2 );
    // digitalReadFast(18); // could check for 0 from slave ?
    delayMicroseconds(3);
  }
  if ( 2 <= iCmd ) {
    // TBD
  }
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

void printActiveDevs()
{
  Serial.println("============================");
  Serial.println("I2C Test Board Configuration");
  Serial.println("============================");
  Serial.printf("MPU9250\tBNO055\tBNO080\tMB85\tLIDAR\tSHT31\tT32\n");
  #if defined(_use_9250)
    Serial.printf("Yes\t");
  #else 
    Serial.printf("No\t"); 
  #endif
  #if defined(_use_BNO055)
    Serial.printf("Yes\t");
  #else 
    Serial.printf("No\t"); 
  #endif
  #if defined(_use_BNO080)
    Serial.printf("Yes\t");
  #else 
    Serial.printf("No\t"); 
  #endif
  #if defined(_use_MB85)    
    Serial.printf("Yes\t");
  #else 
    Serial.printf("No\t"); 
  #endif
  #if defined(_use_lidar)
    Serial.printf("Yes\t");
  #else 
    Serial.printf("No\t"); 
  #endif
  #if defined(_use_SHT31)
    Serial.printf("Yes\t");
  #else 
    Serial.printf("No\t"); 
  #endif
  #if defined(_use_T32)
    Serial.printf("Yes\t");
  #else 
    Serial.printf("No\t"); 
  #endif
  Serial.println("\n");
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
  Scansetup(); // setup() :: Wire_Scanner_all.ino.h
  Scanloop(); // one loop() :: Wire_Scanner_all.ino.h

  printActiveDevs();

  ToggleClock0( 1 );
  Serial.println("==============================");
  Serial.println("INITIALIZING CONNECTED DEVICES");
  Serial.println("==============================");
#if defined( _use_MB85)
  Serial.println("---> Starting MB85...");

  mymemory.begin();
  //_MB85_port.setClock(1000000);
  //mymemory.eraseDevice();

#if defined( _write_init_MB85)
  //---------init data - load array
  Serial.println("\tWriting data to MB85:");
  byte arraySize = sizeof(MYDATA_t);
  mydata.datastruct.data_0 = true;
  Serial.print("\tData_0: ");
  if (mydata.datastruct.data_0) Serial.println("\ttrue");
  if (!mydata.datastruct.data_0) Serial.println("\tfalse");
  mydata.datastruct.data_1 = 1.3575;
  Serial.print("\tData_1: ");
  Serial.println(mydata.datastruct.data_1, DEC);
  mydata.datastruct.data_2 = 314159L;
  Serial.print("\tData_2: ");
  Serial.println(mydata.datastruct.data_2, DEC);
  mydata.datastruct.data_3 = 142;
  Serial.print("\tData_3: ");
  Serial.println(mydata.datastruct.data_3, DEC);
  mydata.datastruct.data_4 = 0x50;
  Serial.print("\tData_4: 0x");
  Serial.println(mydata.datastruct.data_4, HEX);

  //string test
  String string_test = "The Quick Brown Fox Jumped";
  char cbuff[string_test.length() + 1];
  string_test.toCharArray(cbuff, string_test.length() + 1);
  for (uint8_t j = 0; j < string_test.length() + 1; j++) {
    mydata.datastruct.data_5[j] = cbuff[j];
  }
  Serial.print("\t");
  Serial.println(string_test);

  Serial.println("\t...... ...... ......");
  Serial.println("\tInit Done - array loaded");
  Serial.println("\t...... ...... ......");

  //----------write to FRAM chip
  byte result = mymemory.writeArray(writeaddress, arraySize, mydata.I2CPacket);
  if (result == 0) Serial.println("\tWrite Done - array loaded in FRAM chip");
  if (result != 0) Serial.println("\tWrite failed");
  Serial.println("\t...... ...... ......");

#endif //write_init_data
#endif

#if defined ( _use_ssd1306 )
  //DisplayWire->begin();
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }
  delay(500);
  printSSD( 0, 0, ("---> Hello SSD1306\n"), 2 );

#endif

#if defined(_use_9250)
  // start communication with IMU
  printSSD( 0, 0, ("---> 9250 Init"), 1 );
  status = IMU.begin();
  if (status < 0) {
    Serial.println("\tIMU 9250 initialization unsuccessful");
    Serial.println("\tCheck IMU wiring or try cycling power");
    Serial.print("\tStatus: ");
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
  printSSD( -1, -1, ("---> 055 Init"), 1 );
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  printSSD( -1, -1, ("ialized\n"), 1 );

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("\tCurrent Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
#endif

#if defined(_use_BNO080)
  printSSD( -1, -1, ("---> 080 Init"), 1 );
  delay(200);
  _BNO080_port.begin();
  myIMU.enableDebugging();
  /*
  if (myIMU.begin(0x4B, _BNO080_port) == false)
  {
    Serial.println("\tBNO080 not detected at default I2C address. Check your jumpers and the hookup guide. TOGGLE...");
    if ( 1 ) {
      printSSD( -1, -1, ("... TOGGLE\n"), 1 );
      ToggleClock0( 1 );
      if (!myIMU.begin())
      {
        // There was a problem detecting the BNO055 ... check your connections
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
        delay(5000);
      }
    }
    else
      while (1);
  }
  */
  _BNO080_port.begin();//lib doesnt set up wire.begin()
  status = myIMU.begin(0x4B, _BNO080_port);
  _BNO080_port.setClock(400000); //Increase I2C data rate to 400kHz   //max

  myIMU.enableRotationVector(50); //Send data update every 50ms

  Serial.println(F("\tRotation vector enabled"));
  //Serial.println(F("\tOutput in form i, j, k, real, accuracy"));

  printSSD( -1, -1, ("ialized\n"), 1 );
#endif

#if defined( _use_lidar)
  printSSD( -1, -1, ("---> LL3 Init"), 1 );
  //myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  _LIDAR_port.begin();
  //Wire.setClock(400000UL);  //max

  myLidarLite.configure(0); // Change this number to try out alternate configurations
  printSSD( -1, -1, ("ialized C+\n"), 1 );
#endif

#if defined(_use_QKEYPAD) //  0x4b
  printSSD( -1, -1, ("---> QKey Init"), 1 );
  if (qkeypad.begin(_QKEY_port, _use_QKEYPAD) == false) {  // Note, using begin() like this will use default I2C address, 0x4B.
    printSSD( -1, -1, ("... FAIL"), 1 );
  } else {
    printSSD( -1, -1, ("ialized C+\n"), 1 );
    Serial.println(qkeypad.getVersion());
  }
#endif
#if defined(_use_QBTN1)
  printSSD( -1, -1, ("---> QBtn1 Init"), 1 );
  if (qbtn1.begin( _use_QBTN1, _QBTN1_port) == false) {  // Note, using begin() like this will use default I2C address, 0x4B.
    printSSD( -1, -1, ("... FAIL"), 1 );
  } else {
    printSSD( -1, -1, ("ialized C+\n"), 1 );
    qbtn1.clearEventBits();
    qbtn1.LEDoff();
  }
#endif
#if defined(_use_QBTN2)
  printSSD( -1, -1, ("---> QBtn2 Init"), 1 );
  if (qbtn2.begin(_use_QBTN2, _QBTN2_port) == false) {  // Note, using begin() like this will use default I2C address, 0x4B.
    printSSD( -1, -1, ("... FAIL"), 1 );
  } else {
    printSSD( -1, -1, ("ialized C+\n"), 1 );
    qbtn2.clearEventBits();
    qbtn2.LEDoff();
  }
#endif

#if defined(_use_SHT31)
  printSSD( -1, -1, ("---> SHT31 Init"), 1 );
  if (sht31.begin(_use_SHT31) == false) {  // Note, using begin() like this will use default I2C address, 0x4B.
    printSSD( -1, -1, ("... FAIL"), 1 );
  } else {
    printSSD( -1, -1, ("ialized C+\n"), 1 );
  }
#endif

#if defined(_use_T32)
  Serial.println("---> T3.2 Connected");
  ET.begin(details(slavedata), &_T32_port);
  pinMode(13, OUTPUT);
  randomSeed(analogRead(0));
#endif

#if defined(__IMXRT1062__)
  WDT_timings_t config;
  config.trigger = 10; /* in seconds, 0->128 */
  config.timeout = 20; /* in seconds, 0->128 */
  config.callback = myCallback;
  wdt.begin(config);
#endif

  Wire.setClock(400000);
  Wire1.setClock(400000);
  delay(1000);
}

void loop()
{
  static uint32_t feed = millis();
  if ( millis() - feed > 21000 ) {
    feed = millis();
#if defined(__IMXRT1062__)
    wdt.feed(); /* feed the dog every 11 seconds, to exceed 10second timeout period to refresh callback and gpio state for repeat */
#endif
    //idisp055 = 1;
    //idisp9250 = 0;
    //idisp080 = 0;
  }

  // QBTN1 on says hold which field is displayed
  if (!hold_display_field && (disp_data_elapsed >= FIELD_DISPLAY_TIME)) {
    disp_data_elapsed = 0;
    disp_data_index++;
    if (disp_data_index >= DISPLAY_FIELD_COUNT)
      disp_data_index = 0;
  }

  if(loopCount % 20 == 0){
    Serial.println("=======================================================");
    Serial.print("==================  LOOP ");
    Serial.print(loopCount);
    Serial.println("  ==========================");
    Serial.println("=======================================================");
    printTest = 1;
  } else {
    printTest = 0;
  }
    loopCount = loopCount + 1;
    
#if defined( _use_9250)
  loop9250();
  if(printTest == 1) Serial.println();
#endif

#if defined(_use_BNO055)
  loop055();
  if(printTest == 1) Serial.println();
#endif

#if defined(_use_BNO080)
  loop080();
  if(printTest == 1) Serial.println();
#endif

#if defined(_use_lidar)
  looplidar();
  if(printTest == 1) Serial.println();
#endif

#if defined( _use_MB85)
  loopMB85();
  if(printTest == 1) Serial.println();
#endif

#if defined(_use_QKEYPAD) //  0x4b
  loopQPad();
#endif
#if defined(_use_QBTN1)
  loopQBtn1();
#endif
#if defined(_use_QBTN2)
  loopQBtn12();
#endif

#if defined(_use_SHT31)
  loopSHT31();
  if(printTest == 1) Serial.println();
#endif

#if defined(_use_T32)
  loopT32();
  if(printTest == 1) Serial.println();
#endif

}
