/**************************************************************************/
/*!
    @file     FRAM_I2C_store_anything.ino
    @author   SOSAndroid (E. Ha.)
    @license  BSD (see license.txt)

    Example sketch that stores and retreive any kind of data.

    @section  HISTORY

    v1.0 - First release
	v1.0.1 - fix constructor call error
*/
/**************************************************************************/

#include <Wire.h>
#include <math.h>

#include <FRAM_MB85RC_I2C.h>


//define a struct of various data types
typedef struct MYDATA_t {
	bool data_0;
	float data_1; 
	long data_2; 
	int data_3;
	byte data_4;
  char data_5[32];
};

//define a struct joining MYDATA_t to an array of bytes to be stored
typedef union MYDATA4I2C_t {
 MYDATA_t datastruct;
 uint8_t I2CPacket[sizeof(MYDATA_t)];
};

MYDATA4I2C_t mydata; //data to be written in memory
MYDATA4I2C_t readdata; //data read from memory

//random address to write from
uint16_t writeaddress = 0x025;
uint16_t writeaddress2 = (256 * 128) - 80; // calculated regarding density to hit more or less the end of memory map
byte resultw, resultr; 

//Creating object for FRAM chip
FRAM_MB85RC_I2C mymemory;

void setup() {

	Serial.begin(9600);
	while (!Serial) ; //wait until Serial ready
	//Wire.begin();
  
	
	byte arraySize = sizeof(MYDATA_t);
	
    Serial.println("Starting...");
		
	mymemory.begin();
 Wire.setClock(1000000);
  //mymemory.eraseDevice();
	
//---------init data - load array
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
  char cbuff[string_test.length()+1];
  string_test.toCharArray(cbuff, string_test.length()+1);
  for(uint8_t j=0; j <string_test.length()+1; j++){
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
	
	
//---------read data from memory chip
	result = mymemory.readArray(writeaddress, arraySize, readdata.I2CPacket);
    if (result == 0) Serial.println("Read Done - array loaded with read data");
    if (result != 0) Serial.println("Read failed");
	Serial.println("...... ...... ......");
	
//---------Send data to serial
	Serial.print("Data_0: ");
	if (readdata.datastruct.data_0) Serial.println("true");
	if (!readdata.datastruct.data_0) Serial.println("false");
	Serial.print("Data_1: ");
	Serial.println(readdata.datastruct.data_1, DEC);
	Serial.print("Data_2: ");
	Serial.println(readdata.datastruct.data_2, DEC);
	Serial.print("Data_3: ");
	Serial.println(readdata.datastruct.data_3, DEC);	
	Serial.print("Data_4: 0x");
	Serial.println(readdata.datastruct.data_4, HEX);
  Serial.print("Data_5: ");
  for(uint8_t j = 0; j < string_test.length()+1; j++){
    Serial.print(readdata.datastruct.data_5[j]);
  }
  Serial.println();

  
	Serial.println("...... ...... ......");
	Serial.println("Read Write test done - check data if successfull");
	Serial.println("...... ...... ......");	

  
  // dump the entire 32K of memory!
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

}

void loop() {
	// nothing to do
}
