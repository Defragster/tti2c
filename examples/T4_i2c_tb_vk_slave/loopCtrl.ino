
#if defined(_use_9250)
void loop9250() {
  // read the sensor
  IMU.readSensor();

  // display the data
  Serial.print("MPU9250: ");
  Serial.print(IMU.getAccelX_mss(), 6);
  Serial.print(", ");
  Serial.print(IMU.getAccelY_mss(), 6);
  Serial.print(", ");
  Serial.print(IMU.getAccelZ_mss(), 6);
  Serial.print(", ");
  Serial.print(IMU.getGyroX_rads(), 6);
  Serial.print(", ");
  Serial.print(IMU.getGyroY_rads(), 6);
  Serial.print(", ");
  Serial.print(IMU.getGyroZ_rads(), 6);
  Serial.print(", ");
  Serial.print(IMU.getMagX_uT(), 6);
  Serial.print(", ");
  Serial.print(IMU.getMagY_uT(), 6);
  Serial.print(", ");
  Serial.println(IMU.getMagZ_uT(), 6);

  //Serial.print(", ");
  //Serial.println(IMU.getTemperature_C(),6);
#if defined ( _use_ssd1306 )
  if (disp_data_index == DISPLAY_9250) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("9250: ");
    display.print(IMU.getAccelX_mss(), 2);
    display.print(",");
    display.print(IMU.getAccelY_mss(), 2);
    display.print(", ");
    display.print(IMU.getAccelZ_mss(), 2);
    display.display();
  }
#endif
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

#if defined ( _use_ssd1306)
  if (disp_data_index == DISPLAY_BNO055) {
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
  }
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

#if defined ( _use_ssd1306)
    if (disp_data_index == DISPLAY_BNO080) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println("BNO0080: ");
      display.print(quatI, 2);
      display.print(",");
      display.print(quatJ, 2);
      display.print(", ");
      display.print(quatK, 2);
      display.print(", ");
      display.print(quatReal, 2);
      display.display();
    }
#endif
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
  Serial.print(myLidarLite.distance());
  // Take 99 measurements without receiver bias correction and print to serial terminal
  for (int i = 0; i < 99; i++)
  {
    int lld = myLidarLite.distance(false);
#if defined ( _use_ssd1306 )
    if (disp_data_index == DISPLAY_lidar) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println("LLv3: ");
      display.print(lld);
      if (lld < 10)
        display.print(" cm Too Close");
      display.display();
      if ( 0 == i )   Scanloop(); // one loop() :: Wire_Scanner_all.ino.h
    }
#endif
    static int lldL = 0, jj = 1;
    if ( lld > lldL + 2 || lld < lldL - 2 ) {
      jj++;
      if ( !(jj % 20 ) )
        Serial.print("\nLidarLiteV3: ");
      else
        Serial.print(", ");
      Serial.print(lld);
      lldL = lld;
    }
  }
  Serial.print("\n");
}
#endif

#if defined( _use_MB85)
void loopMB85()
{
  //---------read data from memory chip
  byte arraySize = sizeof(MYDATA_t);
  byte result = mymemory.readArray(writeaddress, arraySize, readdata.I2CPacket);
  if (result == 0) Serial.println("MB85 Read Done - array loaded with read data");
  if (result != 0) Serial.println("MB85 Read failed");
  Serial.println("...... ...... ......");

  //---------Send data to serial
  Serial.print("Data: ");
  if (readdata.datastruct.data_0) Serial.print("true");
  if (!readdata.datastruct.data_0) Serial.print("false");
  Serial.print(", ");
  Serial.print(readdata.datastruct.data_1, DEC);
  Serial.print(",  ");
  Serial.print(readdata.datastruct.data_2, DEC);
  Serial.print(", ");
  Serial.print(readdata.datastruct.data_3, DEC);
  Serial.print(", 0x");
  Serial.println(readdata.datastruct.data_4, HEX);
  Serial.print("Data_5: ");
  for (uint8_t j = 0; j < 19 + 1; j++) {
    Serial.print(readdata.datastruct.data_5[j]);
  }
  Serial.println();
  /*
    Serial.println("...... ...... ......");
    Serial.println("Read Write test done - check data if successfull");
    Serial.println("...... ...... ......");
  */
  // dump the entire 32K of memory!
  /*
    uint8_t value;
    for (uint16_t a = 0; a < 0x100; a++) {
    resultr = mymemory.readByte(a, &value);

    if ((a % 32) == 0) {
      Serial.print("\n 0x"); Serial.print(a, HEX); Serial.print(": ");
    }
    Serial.print("0x");
    if (value < 0x1)
      Serial.print('0');
    Serial.print(value, HEX); Serial.print(" ");
    }
  */
}
#endif

#if defined(_use_QBTN1)
void loopQBtn1()
{
  if (qbtn1.hasBeenClicked()) {
    qbtn1.clearEventBits();
    Serial.println("QBtn1 Has been clicked");
    // see if I can toggle
    if (qbtn1.readSingleRegister(LED_BRIGHTNESS)) {
      qbtn1.LEDoff();
      hold_display_field = false;
    } else {
      qbtn1.LEDon(100);
      hold_display_field = true;
    }
  }
}
#endif
#if defined(_use_QBTN2)
void loopQBtn12()
{
  if (qbtn2.hasBeenClicked()) {
    Serial.println("QBtn2 Has been clicked");
    qbtn2.clearEventBits();
    // see if I can toggle
    if (qbtn2.readSingleRegister(LED_BRIGHTNESS)) {
      qbtn2.LEDoff();
    } else {
      qbtn2.LEDon(100);
    }
  }
}
#endif

#if defined(_use_QKEYPAD) //  0x4b
void loopQPad()
{
  qkeypad.updateFIFO();  // necessary for keypad to pull button from stack to readable register
  char button = qkeypad.getButton();

  if (button == -1)
  {
    Serial.println("No keypad detected");
    delay(1000);
  }
  else if (button != 0)
  {
    if (button == '#') Serial.println();
    else if (button == '*') Serial.print(" ");
    else {
      Serial.print("BUTTON PUSHED: "); Serial.println(button);
      Serial.println();
      disp_data_elapsed = 0;
      disp_data_index = (button - '0');
      if (disp_data_index >= DISPLAY_FIELD_COUNT)
        disp_data_index = 0;
    }
  }
}
#endif

#if defined(_use_SHT31)
void loopSHT31()
{
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();

  if (! isnan(t)) {  // check if 'is not a number'
    Serial.print("Temp *C = "); Serial.println(t);
  } else {
    Serial.println("Failed to read temperature");
  }

  if (! isnan(h)) {  // check if 'is not a number'
    Serial.print("Hum. % = "); Serial.println(h);
  } else {
    Serial.println("Failed to read humidity");
  }
#if defined ( _use_ssd1306)
  if (disp_data_index == DISPLAY_SHT31) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("SHT31: ");
    display.print(t, 2);
    display.print("C, ");
    display.print(t * 1.8 + 32.0, 2);
    display.print("F,");
    display.print(h, 2);
    display.print("H%");
    display.display();
  }
#endif
}
#endif

#if defined(_use_T32)
void loopT32()
{
  //this is how you access the variables. [name of the group].[variable name]
  slavedata.blinks = random(5);
  slavedata.pause = random(5);
  //send the data
  ET.sendData(I2C_SLAVE_ADDRESS);

  //Just for fun, we will blink it out too
  for (int i = slavedata.blinks; i > 0; i--) {
    digitalWrite(13, HIGH);
    delay(slavedata.pause * 100);
    digitalWrite(13, LOW);
    delay(slavedata.pause * 100);
  }

  Serial.printf("Sending Data to T3.2 ====> \n", slavedata.blinks);
}
#endif
