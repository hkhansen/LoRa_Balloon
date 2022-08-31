//READ THIS
//Remember to set the calibrated center frequency for the module
//being programmed. All corrected frequencies can be found in
//the document "Frequency calibration of LoRa modules". If this 
//can not be determined, use the default value of 0xD90000
//
//Remember to set the correct define just below this text 
//depending on the module type. There are still some AlbaTracker
//modules in use, although they are to be discountinued.
//Do also take care to correct the SS pin definition in the
//SX1272.h file accordingly.

#define LoraComModule
//#define AlbaTracker
//REMEMBER to change SS pin in SX1272.h file accordingly

#include "SX1272.h"
#include <SPI.h>

// SD Card
#include <SD.h>
const int SD_cs = 10;


const unsigned char MAX_PACKET_LENGTH = 255;
unsigned long int timer = 0;
const unsigned int delayTime = 20000;
char messageWaiting = 0;

int e;
char my_packet[MAX_PACKET_LENGTH];

uint8_t ADDRESS = 8;
uint8_t ADDRESS_BALLOON_1 = 1;
uint8_t ADDRESS_BALLOON_2 = 2;
uint8_t receiver_address = ADDRESS_BALLOON_1;

//Pin assignments
uint8_t const sx1272_reset_pin = 2;

#define debug 2 //1 - get packet info, 2 get program debug info

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  // Print a start message
  Serial.println(F("SX1272 module and Arduino: receive packets without ACK"));

  // Set reset pin as output
  pinMode(sx1272_reset_pin, OUTPUT);
  
   // Power ON the module
  sx1272.ON();
  
  // Set transmission mode and print the result
  e = sx1272.setMode(1);
  if (debug > 1) {
    Serial.print(F("Setting mode: state "));
    Serial.println(e, DEC);
  }
  
  // Select frequency channel
  e = sx1272.setChannel(0xD90000); //Frequency corrected value - defaul 0xD90000 (868MHz)
  if (debug > 1) {
    Serial.print(F("Setting Channel: state "));
    Serial.println(e, DEC);
  }

  //Adjust for any frequency offset in the module
  e = sx1272.setChannel(0xD9032A);//(0xD8EE66);
  if (debug > 1) {
    Serial.print(F("Setting frequency: state "));
    Serial.println(e, DEC);
  }

  //Set spreading factor
  e = sx1272.setSF(12); //Make the signal orthogonal to the position signals
  if (debug > 1) {
    Serial.print(F("Setting SF: state "));
    Serial.println(e, DEC);
  }

  //Set coding rate (error correction code)
  e = sx1272.setCR(CR_8); //Maximum error correction
  if (debug > 1) {
    Serial.print(F("Setting CR: state "));
    Serial.println(e, DEC);
  }
  
  // Select output power (Max, High or Low)
  e = sx1272.setPower('P');
    if (debug > 1) {
      Serial.print(F("Setting Power: state "));
      Serial.println(e);
    }


//  // Set value for over current protection (0x1B if 20dBm output power - 0x0B is default!)
  e = sx1272.setMaxCurrent(0x1B); // Only set this if 20dBm power is used
  if (debug > 1) {
    Serial.print(F("Setting Over Current Protection: state "));
    Serial.println(e);
  }

  //Select signal output pin
  e = sx1272.setRfOutPin(1);

  if (debug > 1) {
    Serial.print(F("Setting output pin: state "));
    Serial.println(e);
  }
  
  // Set the node address and print the result
  e = sx1272.setNodeAddress(ADDRESS);
  if (debug > 1) {
    Serial.print(F("Setting node address: state "));
    Serial.println(e, DEC);
  }
  
  // Print a success message
  if (debug > 1) {
    Serial.println(F("SX1272 successfully configured "));
  }
  
  // Initiate SD
  if (!SD.begin(SD_cs)) {
    Serial.println("SD not configured");
  }
  else {
    Serial.println("SD ready");
    }

  
  // Set external interrupt for received packages
  setInterruptPin(); //used with Lora Com Module
  initExtInt(); //used with Lora Com Module

  // Initiate the receive mode
  e = sx1272.receive();
  if (debug > 1) {
    Serial.print("Receive setup: ");
    Serial.println(e, DEC);
  }

  //Reset array
  for (int i = 0; i < MAX_PACKET_LENGTH; i++) {
    my_packet[i] = 0;
  }
}

void loop(void) {

  int k = 0;
  while (Serial.available()) {
    my_packet[k++] = Serial.read();
    Serial.print(my_packet[k-1]);
    delay(10); 
  }

  //See if the received serial data is the command to change receiver address
  if (my_packet[0] == 'A') {
    switch (my_packet[1]) {
      case '1':
        receiver_address = ADDRESS_BALLOON_1;
        Serial.print("Receiver address: 1");
        my_packet[0] = 0; //Don't transmit the command afterwards
        break;
      case '2':
        receiver_address = ADDRESS_BALLOON_2;
        Serial.print("Receiver address: 2");
        my_packet[0] = 0; //Don't transmit the command afterwards
        break;
      default:
        break;
    }
  }

  if (my_packet[0] > 0) {
    //Send packet
    e = sx1272.sendPacketMAXTimeout(receiver_address, my_packet);
    #if (debug > 1)
      Serial.print(F("Packet sent, state "));
      Serial.println(e, DEC);
    #endif
    
    // Initiate the receive mode
    e = sx1272.receive();
    #if (debug > 1)
      Serial.print("Receive setup: ");
      Serial.println(e, DEC);
    #endif

    //Reset array
    for (int i = 0; i < MAX_PACKET_LENGTH; i++) {
      my_packet[i] = 0;
    }
  }    
  
  //If the messageWaiting flag has been set and the receiver address in the 
  // messeage coresponds to the address in this moduke, do:
  if (messageWaiting) {
    e = sx1272.getPacket();
    #if (debug > 1)
      Serial.print(F("Packet status: "));
      Serial.print(e, DEC);
      Serial.print(", Dst: ");
      Serial.print(sx1272.packet_received.dst);
    #endif
    /*
     * If if was received correctly and dst addsress
     * correspondst to the address of the module
     */
    if ( e == 0 && sx1272.packet_received.dst == ADDRESS)
    {
      #if (debug > 1)
        Serial.print(F(", Receive packet, state "));
        Serial.print(e, DEC);
      #endif
  
      for (unsigned int i = 0; i < sx1272.packet_received.length; i++)
      {
        my_packet[i] = (char)sx1272.packet_received.data[i];
      }

      #if (debug > 1)
        Serial.print(F(", Message: "));
        Serial.print(my_packet);
        // Print to SD
        if (SD.begin(SD_cs)){ 
          File data = SD.open("data.txt", FILE_WRITE);
          data.println(my_packet);
          data.close();
        }
        Serial.print(F("\", \"packnum\": "));
        Serial.print(sx1272.packet_received.packnum);
        sx1272.getRSSI();
        Serial.print(F(", \"RSSI\": "));
        Serial.print(sx1272._RSSI, DEC);
        sx1272.getRSSIpacket();
        Serial.print(F(", \"RSSIPacket\": "));
        Serial.print(sx1272._RSSIpacket, DEC);
        sx1272.getSNR();
        Serial.print(F(", \"SNR\": "));
        Serial.print(sx1272._SNR, DEC);
      #endif
    }
    Serial.println("");
    //Reset array
    for (int i = 0; i < MAX_PACKET_LENGTH; i++) {
      my_packet[i] = 0;
    }
    
    messageWaiting = 0;
  }
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

ISR (INT0_vect)
{ //Executed on a rising edge. No need to check if the pin is high.
  messageWaiting = 1;
}
