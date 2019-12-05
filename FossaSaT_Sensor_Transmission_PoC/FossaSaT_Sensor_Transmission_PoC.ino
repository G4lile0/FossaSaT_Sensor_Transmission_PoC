/*
   RadioLib SX127x Receive with Interrupts Example

   This example listens for LoRa transmissions and tries to
   receive them. Once a packet is received, an interrupt is
   triggered. To successfully receive data, the following
   settings have to be the same on both transmitter
   and receiver:
    - carrier frequency
    - bandwidth
    - spreading factor
    - coding rate
    - sync word

   Other modules from SX127x/RFM9x family can also be used.

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>
#include "Comms.h"


/* 
 *  
Topic configuration 
*/

char key[3] = { 'T', '0' , '@'};     // Key  to identify the packets as a sensor packet 
char topic[2] = { 0x01 , 0x01} ;      // identification of the topic, we have 65536 

// now you can build your message as your wish, but please keep it short, for this example we will send two floats, first the temperature, second the preasure.
char float1[4] = { 0x00 , 0x01, 0x02 , 0x01 };
char float2[4];





/* 
 *  
 Sensor configuration for this PoC we will use a BMP180
*/

#include <BMP180I2C.h>
#define I2C_ADDRESS 0x77
//create an BMP180 object using the I2C interface
BMP180I2C bmp180(I2C_ADDRESS);




// end of sensor configuration






// SX1278 has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// DIO1 pin:  3
SX1278 lora = new Module(18, 26, 12);
#define LORA_CARRIER_FREQUENCY                          436.7f  // MHz
#define LORA_BANDWIDTH                                  125.0f  // kHz dual sideband
#define LORA_SPREADING_FACTOR                           11
#define LORA_SPREADING_FACTOR_ALT                       10
#define LORA_CODING_RATE                                8       // 4/8, Extended Hamming
#define LORA_OUTPUT_POWER                               21      // dBm
#define LORA_CURRENT_LIMIT                              120     // mA


// satellite callsign
char callsign[] = "FOSSASAT-1";

void setup() {
  Serial.begin(115200);
  // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));
  // carrier frequency:           436.7 MHz
  // bandwidth:                   125.0 kHz
  // spreading factor:            11
  // coding rate:                 8
  // sync word:                   0xff
  // output power:                17 dBm
  // current limit:               100 mA
  // preamble length:             8 symbols
  // amplifier gain:              0 (automatic gain control)
//  int state = lora.begin();
 
     int state = lora.begin(LORA_CARRIER_FREQUENCY,
                            LORA_BANDWIDTH,
                            LORA_SPREADING_FACTOR,
                            LORA_CODING_RATE,
                            0xff,
                            17,
                            (uint8_t)LORA_CURRENT_LIMIT);
                            Serial.println(LORA_CARRIER_FREQUENCY);
                            Serial.println(LORA_SPREADING_FACTOR);
                            Serial.println(LORA_CODING_RATE);
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // set the function that will be called
  // when new packet is received
  lora.setDio0Action(setFlag);

  // start listening for LoRa packets
  Serial.print(F("[SX1278] Starting to listen ... "));
  state = lora.startReceive();
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);


  }
  
/* 
 *  
 Sensor Setup for this PoC we will use a BMP180
*/
  
   Wire.begin (4, 15);

  
    //begin() initializes the interface, checks the sensor ID and reads the calibration parameters.  
  if (!bmp180.begin())
  {
    Serial.println("begin() failed. check your BMP180 Interface and I2C Address.");
    while (1);
  }

  //reset sensor to default parameters.
  bmp180.resetToDefaults();

  //enable ultra high resolution mode for pressure measurements
  bmp180.setSamplingMode(BMP180MI::MODE_UHR);

  
  
  

  // if needed, 'listen' mode can be disabled by calling
  // any of the following methods:
  //
  // lora.standby()
  // lora.sleep()
  // lora.transmit();
  // lora.receive();
  // lora.readData();
  // lora.scanChannel();
}

// flag to indicate that a packet was received
volatile bool receivedFlag = false;
volatile bool fossa_good_packet = false;


// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // we got a packet, set the flag
  receivedFlag = true;
}


void loop() {

   if (receivedFlag)  process_packet();
   if (fossa_good_packet) Retransmit();




/* 
 *  
 Sensor loop for this PoC we will use a BMP180
*/


  delay(1000);


  //start a temperature measurement
  if (!bmp180.measureTemperature())
  {
    Serial.println("could not start temperature measurement, is a measurement already running?");
    return;
  }

  //wait for the measurement to finish. proceed as soon as hasValue() returned true. 
  do
  {
    delay(100);
  } while (!bmp180.hasValue());

  float temperature = bmp180.getTemperature();
  Serial.print("Temperature: "); 
  Serial.print(temperature); 
  Serial.println(" degC");

  float2Bytes(temperature,&float1[0]);
  



  //start a pressure measurement. pressure measurements depend on temperature measurement, you should only start a pressure 
  //measurement immediately after a temperature measurement. 
  if (!bmp180.measurePressure())
  {
    Serial.println("could not start perssure measurement, is a measurement already running?");
    return;
  }

  //wait for the measurement to finish. proceed as soon as hasValue() returned true. 
  do
  {
    delay(100);
  } while (!bmp180.hasValue());


  float preasure=bmp180.getPressure();
  Serial.print("Pressure: "); 
  Serial.print(preasure);
  Serial.println(" Pa");

  
  float2Bytes(temperature,&float2[0]);  



}






void process_packet ( ) {
  // check if the flag is set
    // disable the interrupt service routine while
    // processing the data
    enableInterrupt = false;

    // reset flag
    receivedFlag = false;


    // read received data
    size_t respLen = lora.getPacketLength();
    uint8_t* respFrame = new uint8_t[respLen];
    int state = lora.readData(respFrame, respLen);

    // get function ID
      uint8_t functionId = FCP_Get_FunctionID(callsign, respFrame, respLen);
      Serial.print(F("Function ID: 0x"));
      Serial.println(functionId, HEX);

      // check optional data
      uint8_t* respOptData = nullptr;
      uint8_t respOptDataLen = FCP_Get_OptData_Length(callsign, respFrame, respLen);
      Serial.print(F("Optional data ("));
      Serial.print(respOptDataLen);
      Serial.println(F(" bytes):"));
      if(respOptDataLen > 0) {
        // read optional data
        respOptData = new uint8_t[respOptDataLen];
        FCP_Get_OptData(callsign, respFrame, respLen, respOptData);
        PRINT_BUFF(respFrame, respLen);
      }

      // process received frame
      switch(functionId) {
        case RESP_PONG:
          Serial.println(F("Pong!"));
          fossa_good_packet = true;
          break;

        case RESP_SYSTEM_INFO:
          Serial.println(F("System info:"));

          Serial.print(F("batteryChargingVoltage = "));
          Serial.println(FCP_Get_Battery_Charging_Voltage(respOptData));

          Serial.print(F("batteryChargingCurrent = "));
          Serial.println(FCP_Get_Battery_Charging_Current(respOptData), 4);

          Serial.print(F("batteryVoltage = "));
          Serial.println(FCP_Get_Battery_Voltage(respOptData));

          Serial.print(F("solarCellAVoltage = "));
          Serial.println(FCP_Get_Solar_Cell_Voltage(0, respOptData));

          Serial.print(F("solarCellBVoltage = "));
          Serial.println(FCP_Get_Solar_Cell_Voltage(1, respOptData));

          Serial.print(F("solarCellCVoltage = "));
          Serial.println(FCP_Get_Solar_Cell_Voltage(2, respOptData));

          Serial.print(F("batteryTemperature = "));
          Serial.println(FCP_Get_Battery_Temperature(respOptData));

          Serial.print(F("boardTemperature = "));
          Serial.println(FCP_Get_Board_Temperature(respOptData));

          Serial.print(F("mcuTemperature = "));
          Serial.println(FCP_Get_MCU_Temperature(respOptData));

          Serial.print(F("resetCounter = "));
          Serial.println(FCP_Get_Reset_Counter(respOptData));

          Serial.print(F("powerConfig = 0b"));
          Serial.println(FCP_Get_Power_Configuration(respOptData), BIN);
          fossa_good_packet = true;
          break;

        case RESP_LAST_PACKET_INFO:
          Serial.println(F("Last packet info:"));

          Serial.print(F("SNR = "));
          Serial.print(respOptData[0] / 4.0);
          Serial.println(F(" dB"));

          Serial.print(F("RSSI = "));
          Serial.print(respOptData[1] / -2.0);
          Serial.println(F(" dBm"));
          fossa_good_packet = true;
          break;

        case RESP_REPEATED_MESSAGE:
          Serial.println(F("Got repeated message:"));
          Serial.println((char*)respOptData);
          fossa_good_packet = true;
          break;

        default:
          Serial.println(F("Unknown function ID!"));
          fossa_good_packet = false;
          break;
      }


      
    // you can also read received data as byte array
    /*
      byte byteArr[8];
      int state = lora.receive(byteArr, 8);
    */

    // you can read received data as an Arduino String
    //    String str;
    //    int state = lora.readData(str);

    if (state == ERR_NONE) {
      // packet was successfully received
      Serial.println(F("[SX1278] Received packet!"));

      // print data of the packet
      Serial.print(F("[SX1278] Data:\t\t"));
//      Serial.println(str);

      // print RSSI (Received Signal Strength Indicator)
      Serial.print(F("[SX1278] RSSI:\t\t"));
      Serial.print(lora.getRSSI());
      Serial.println(F(" dBm"));

      // print SNR (Signal-to-Noise Ratio)
      Serial.print(F("[SX1278] SNR:\t\t"));
      Serial.print(lora.getSNR());
      Serial.println(F(" dB"));

      // print frequency error
      Serial.print(F("[SX1278] Frequency error:\t"));
      Serial.print(lora.getFrequencyError());
      Serial.println(F(" Hz"));

    } else if (state == ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("[SX1278] CRC error!"));

    } else {
      // some other error occurred
      Serial.print(F("[SX1278] Failed, code "));
      Serial.println(state);
    }

    // put module back to listen mode
    lora.startReceive();

    // we're ready to receive more packets,
    // enable interrupt service routine
    enableInterrupt = true;
}












void Retransmit() {

  
   // disable reception interrupt
    enableInterrupt = false;
   // detachInterrupt(digitalPinToInterrupt(DIO1));

//  Serial.println(F("Enter message to be sent:"));
//  Serial.println(F("(max 32 characters, end with LF or CR+LF)"));

  // get data to be retransmited
  char optData[32];

/*
 * 
  uint8_t bufferPos = 0;
  while(bufferPos < 32) {
    while(!Serial.available());
    char c = Serial.read();
    Serial.print(c);
    if((c != '\r') && (c != '\n')) {
      optData[bufferPos] = c;
      bufferPos++;
    } else {
      break;
    }
  }

 
  // wait for a bit to receive any trailing characters
  delay(100);

  // dump the serial buffer
  while(Serial.available()) {
    Serial.read();
  }


 */


  
  optData[0] =  key[0];
  optData[1] =  key[1];
  optData[2] =  key[2];
  
  optData[3] =  topic[0];
  optData[4] =  topic[1];

  // float1
  optData[5] = float1[0];
  optData[6] = float1[1];
  optData[7] = float1[2];
  optData[8] = float1[3];
  
    // float2
  optData[9] = float2[0];
  optData[10] = float2[1];
  optData[11] = float2[2];
  optData[12] = float2[3];

 //...

  optData[13] = '\0';

 
  Serial.println();
  Serial.print(F("Requesting retransmission ... "));

  // data to transmit
  uint8_t functionId = CMD_RETRANSMIT;
 // optData[bufferPos] = '\0';
  //  uint8_t optDataLen = strlen(optData);
  uint8_t optDataLen = 13;
  
  Serial.println("longitud");
  Serial.println(optDataLen);

  // build frame
  uint8_t len = FCP_Get_Frame_Length(callsign, optDataLen);
  uint8_t* frame = new uint8_t[len];
  FCP_Encode(frame, callsign, functionId, optDataLen, (uint8_t*)optData);

  // send data
  int state = lora.transmit(frame, len);
  delete[] frame;

  // check transmission success
  if (state == ERR_NONE) {
    Serial.println(F("sent successfully!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
  
   fossa_good_packet = false;
   lora.startReceive();
   enableInterrupt = true;
 
}



// https://stackoverflow.com/questions/24420246/c-function-to-convert-float-to-byte-array
// slightly modified

void float2Bytes(float val,char* char_array){
  // Create union of shared memory space
  union {
    float float_variable;
    char temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(char_array, u.temp_array, 4);
}
