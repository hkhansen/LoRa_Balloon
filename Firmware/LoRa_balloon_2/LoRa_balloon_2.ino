//READ THIS
//It is very important that the modified TinyGPS++.cpp file is
//used together with the ublox GPS receiver in order for the
//parser to function properly. There seems to be an error where
//the compiler only reads the TinyGPS library files from the
//library directory and not the directory of this .ino file.
//If the current GPS position is not transmitted correctly,
//that is most likely the error. Just take care to make sure 
//that the correct files are used for the compilation.
//
//Remember to set the calibrated center frequency for the module
//being programmed. All corrected frequencies can be found in
//the document "Frequency calibration of LoRa modules". If this 
//can not be determined, use the default value of 0xD90000.

#include "SX1272.h"
#include <SPI.h>
#include "TinyGPS++.h"

const unsigned char MAX_PACKET_LENGTH = 255;
unsigned long int timer = 0;
const unsigned int delayTime = 30000;
char messageWaiting = 0;

int e;
char my_packet[MAX_PACKET_LENGTH];

uint8_t ADDRESS = 2;
uint8_t ADDRESS_RECEIVER_BALLOON = 1;
uint8_t ADDRESS_RECEIVER_GROUND = 8;

//-------GPS----------
uint8_t set_flight_mode_cmd[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00};//, 0x16, 0xDC};
uint8_t set_flight_mode_chksm[2] = {0x00, 0x00};
uint8_t set_flight_mode_ack = 0;
//const PROGMEM char GPS_Request_string[] = "$PUBX,00*33\r\n";
byte gps_set_sucess = 0 ;

//Pin assignments
uint8_t const sx1272_reset_pin = 2;

#define debug 0 //1 - get packet info, 2 get program debug info

TinyGPSPlus gps;

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  #if (debug > 1)
    // Print a start message
    Serial.println(F("SX1272 module and Arduino: receive packets without ACK"));
  #endif

  //Pin setup
  pinMode(sx1272_reset_pin, OUTPUT);
  

   // Power ON the module
  sx1272.ON();
  
  // Set transmission mode and print the result
  e = sx1272.setMode(1);
  #if (debug > 1)
    Serial.print(F("Setting mode: state "));
    Serial.println(e, DEC);
  #endif
  
  // Select frequency channel
  e = sx1272.setChannel(0xD901D7); //Frequency corrected value
  #if (debug > 1)
    Serial.print(F("Setting Channel: state "));
    Serial.println(e, DEC);
  #endif

  //Set coding rate (error correction code)
  e = sx1272.setCR(CR_8); //Maximum error correction
  #if (debug > 1)
    Serial.print(F("Setting CR: state "));
    Serial.println(e, DEC);
  #endif
  
  // Select output power (Max, High or Low)
  e = sx1272.setPower('P');
  #if (debug > 1)
    Serial.print(F("Setting Power: state "));
    Serial.println(e);
  #endif

//  // Set value for over current protection (0x1B if 20dBm output power - 0x0B is default!)
  e = sx1272.setMaxCurrent(0x1B); // Only set this if 20dBm power is used
  #if (debug > 1)
    Serial.print(F("Setting Over Current Protection: state "));
    Serial.println(e);
  #endif

  //Select signal output pin  
  e = sx1272.setRfOutPin(1);

  #if (debug > 1)
    Serial.print(F("Setting output pin: state "));
    Serial.println(e);
  #endif
  
  // Set the node address and print the result
  e = sx1272.setNodeAddress(ADDRESS);
  #if (debug > 1)
    Serial.print(F("Setting node address: state "));
    Serial.println(e, DEC);
  #endif
  
  // Print a success message
  #if (debug > 1)
    Serial.println(F("SX1272 successfully configured "));
  #endif
  
  // Set external interrupt for received packages
  setInterruptPin(); //used with Lora Com Module
  initExtInt(); //used with Lora Com Module

  // Initiate the receive mode
  e = sx1272.receive();
  #if (debug > 1)
    Serial.print("Receive setup: ");
    Serial.println(e, DEC);
  #endif

  //Initiate GPS
  while(!gps_set_sucess) {
    sendUBX_calc_chksm(set_flight_mode_cmd, sizeof(set_flight_mode_cmd)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(set_flight_mode_cmd);
  }
}

void loop(void) {

  //Search for GPS info within the time of the argument
  smartDelay(1000);

  //Store the GPS information found in the parser
  double latitude = gps.location.lat();
  double langtitude = gps.location.lng();
  float height = gps.altitude.meters();
  int sats = gps.satellites.value();

  //Make a string containing the position information - shall be transmitted later
  String string = "B2: " + String(latitude,6) + "," + String(langtitude,6) + "," + String(height,1) + "," + String(sats);

  //Debugging
  #if (debug > 1)
    Serial.println(string);
  #endif

  //When delayTime has passed, transmit the current position
  if (millis() > timer + delayTime) {
    
    //Convert the string containing the position into a char array and store it in my_packet
    string.toCharArray(my_packet, MAX_PACKET_LENGTH);
    
    //Send the packet containing the position to ground
    e = sx1272.sendPacketMAXTimeout(ADDRESS_RECEIVER_GROUND, my_packet);
    //Debugging
    #if (debug > 1)
      Serial.print(F("Packet sent, state "));
      Serial.println(e, DEC);
    #endif
    
    // Initiate the receive mode. This should be done after the transmit mode has been used
    e = sx1272.receive();
    //Debugging
    #if (debug > 1)
      Serial.print("Receive setup: ");
      Serial.println(e, DEC);
    #endif

    //Set timer to the current millis value
    timer = millis();
  }
    
  //If the messageWaiting flag has been set, do:
  if (messageWaiting) {

    //Get the current packet from the SX1272 memory
    e = sx1272.getPacket();
    //Debugging
    #if (debug > 1)
      Serial.print(F("Packet status: "));
      Serial.println(e, DEC);
    #endif

    if (sx1272.packet_received.dst == ADDRESS) {

      //Get the packet RSSI. Should be included in the following transmission
      sx1272.getRSSIpacket();
  
      //Copy the received message to the array my_packet for retransmission
      for (unsigned int i = 0; i < sx1272.packet_received.length; i++)
      {
        my_packet[i] = (char)sx1272.packet_received.data[i];
      }
  
      //Convert source and RSSI to strings. Insert these into their respective char arrays
      //String SRC_string = String(sx1272.packet_received.src);
      //char SRC_char[] = {',',0,0,0]; //No null termination - will this work as intended?
      //SRC_string.toCharArray(SRC_char,3); //Let the comma be. Only insert up to three characters
  
      String RSSI_string = String(sx1272._RSSIpacket);
      char RSSI_char[] = {' ',' ',' ',' ',' '}; //Five long as the termination char is included
      RSSI_string.toCharArray(RSSI_char,5);
  
      if (sx1272.packet_received.length < 200) {
        //Insert the received packet RSSI into the packet to be transmitted
        for (int i = sx1272.packet_received.length; i < sx1272.packet_received.length + 4; i++) {
          my_packet[i - 5] = RSSI_char[i - sx1272.packet_received.length];
        }
        //^^ 'i-7' ??? That is because the packet length includes other information (source, etc.)! 
  
        //Insert the source of the received packet into the packet to be transmitted
        //for (int i = sx1272.packet_received.length + 4; i < sx1272.packet_received.length + 4 + 4; i++) {
        //  my_packet[i - 4] = SRC_char[i - sx1272.packet_received.length - 4];
        //}
        //^^ 'i-4' ??? same applies here?
        
        my_packet[sx1272.packet_received.length] = 0; //Null termination of the char array //Removed '-1' in my_packet[]
        for (int i = 0; i < sx1272.packet_received.length; i++) {
          //DEBUGGING
          //Serial.print(my_packet[i]);
        }
        //Serial.println();
      }
  
      /* 
       *  Send the received packet back again but the receiver depends on
       *  who transmitted the received packet. If the received packet came
       *  from ground, relay the message to the other balloon. If the
       *  received message came from the other balloon, relay it to ground.
       */
      int receiver_address = 0;
      if (sx1272.packet_received.src == ADDRESS_RECEIVER_GROUND) {
        receiver_address = ADDRESS_RECEIVER_BALLOON;
      }
      else {
         /*Make this an else just to relay any information that might be received 
         * from other channels to ground as well. Might give an indication if 
         * something is wrong.
         */
        receiver_address = ADDRESS_RECEIVER_GROUND;
      }
      e = sx1272.sendPacketMAXTimeout(receiver_address, my_packet);
      #if (debug > 1)
        Serial.print(F("Packet sent, state "));
        Serial.println(e, DEC);
      #endif
  
      #if (debug > 1)
        Serial.print(F("Message: "));
        Serial.println(my_packet);
        Serial.print(F("\", \"packnum\": "));
        Serial.println(sx1272.packet_received.packnum);
        sx1272.getRSSI();
        Serial.print(F(", \"RSSI\": "));
        Serial.println(sx1272._RSSI, DEC);
        sx1272.getRSSIpacket();
        Serial.print(F(", \"RSSIPacket\": "));
        Serial.println(sx1272._RSSIpacket, DEC);
        sx1272.getSNR();
        Serial.print(F(", \"SNR\": "));
        Serial.println(sx1272._SNR, DEC);
      #endif
  
      // Initiate the receive mode
      e = sx1272.receive();
      #if (debug > 1)
        Serial.print("Receive setup: ");
        Serial.println(e, DEC);
      #endif
    }

    //Reset the messageWaiting flag
    messageWaiting = 0;
  }
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial.available())
      gps.encode(Serial.read());
  } while (millis() - start < ms);
}

void setInterruptPin() {
  //Set pin PC5 as input
  DDRC &= ~(1 << DDC5);
}

void initExtInt() {
  //Disable global interrupts
  cli();
  
  //Enable external interrupts on pins PCINT[8:14] pins (datasheet p. 70). Can be read from the PCIF1 register
  PCICR |= (1 << PCIE1); //(p. 73)

  //Only enable pin change interrupt on pin PC5 (A5 on Arduino) 
  PCMSK1 |= (1 << PCINT13);

  //Enable global interrupts
  sei();
}

ISR (PCINT1_vect)
{
  if (PINC & (1 << PINC5)) {
    messageWaiting = 1;
  }
}

//-------GPS---------
// calc chksm and Send a byte array of UBX protocol to the GPS
void sendUBX_calc_chksm(uint8_t *MSG, uint8_t len) {
  uint8_t  CK_A = 0;
  uint8_t  CK_B = 0;
  uint8_t i = 0;
  for(i = 0; i<len; i++){
    if(i>=2){
      CK_A = CK_A + MSG[i];
      CK_B = CK_B + CK_A;
    }
  }
  //# ensure unsigned byte range
  CK_A = (CK_A & 0xFF);
  CK_B = (CK_B & 0xFF);


  for(int i = 0; i<len; i++){
  Serial.write(MSG[i]);
  }
  Serial.write(CK_A);
  Serial.write(CK_B);
  Serial.println();
  //Chip_UART_SendBlocking(LPC_UART3, U3_tx_String, i+2);
}

boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
 //                                    mySerial.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
   //                                                mySerial.println(" (SUCCESS!)");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
   //                                                      mySerial.println(" (FAILED!)");
      return false;
    }
 
    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
     //                                                      mySerial.print(b, HEX);
      } 
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }
 
    }
  }
}
