#include "modbus.h"

uint8_t registers[160]; //this will hold the data up to and inclusive parameter 40 as per SDM630 manual. (4-bytes per parameter)

uint16_t calc_crc(uint8_t* data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < length; pos++) {
    crc ^= data[pos];
    for (int i = 0; i < 8; i++) {
      if ((crc & 1) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void handleInverter(SoftwareSerial& SoftSerial, PubSubClient& client, bool DEBUG_RS485) 
{
  if (SoftSerial.available()>7)                                      //handle incoming requests
  {
    static uint8_t recbuffer[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    unsigned char firstByte = SoftSerial.read();

    if(firstByte==0x01)
    {
      recbuffer[0]=0x01;
      for(int i=1;i<8;i++)
      {
        recbuffer[i] = SoftSerial.read();
      }
   
      if(DEBUG_RS485)
      {
        Serial.print(millis());
        Serial.print(" ");
        for (int i = 0; i < 8; i++) 
        {
          Serial.print(" ");
          if(recbuffer[i]<16)
            Serial.print("0");
          Serial.print(recbuffer[i], HEX);
        }
        Serial.println();
      }
  
      if (calc_crc(recbuffer, 8) != 0)                              //if CRC doesn't match, return
      {
        Serial.println("malformed packet");
        return;
      }
    }
    else
    {
      static uint8_t recbuffer2[13]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
      recbuffer2[0]=firstByte;
      while(SoftSerial.available()<12) {}
      for(int i=1;i<13;i++)
      {
        recbuffer2[i] = SoftSerial.read();
      }
   
      if(DEBUG_RS485)
      {
        Serial.print(millis());
        Serial.print("This weird one ");
        for (int i = 0; i < 13; i++) 
        {
          Serial.print(" ");
          if(recbuffer2[i]<16)
            Serial.print("0");
          Serial.print(recbuffer2[i], HEX);
        }
        Serial.println();
      }
      return;
    }
        
    uint8_t slaveadr = recbuffer[0];
    uint8_t functioncode = recbuffer[1];
    uint16_t address = (recbuffer[2] << 8) | recbuffer[3];
    uint16_t numRegisters = (recbuffer[4] << 8) | recbuffer[5];
 
    if(DEBUG_RS485)
    {
      Serial.print(" process register read ");
      Serial.print(slaveadr);
      Serial.print(" ");
      Serial.print(functioncode);
      Serial.print(" ");
      Serial.print(address);
      Serial.print(" ");
      Serial.println(numRegisters);
    }
    
    //build up the array for returning values
    uint8_t response[(numRegisters*2)+3+2]; //make space for the header and CRC - the registers are 16-bit values so they take up double space!
    for(int i=0;i<sizeof(response);i++)
      response[i]=0;

    //build up the header
    response[0]=0x01;                                               //set slave address
    response[1]=0x04;                                               //response type
    response[2]=numRegisters*2;                                       //remember the registers are 16-bit values

    //fill in the response from the local registers
    uint16_t responseP=0;
    for(int i=(address*2);i<((address*2)+(numRegisters*2));i++)                       //start at the address requested (multiply by 2 to get 16-bit values)
    {
      response[responseP+3]=registers[i];                           //copy to the response array - dont overwrite the header!
      responseP++;
    }

    uint16_t crc = calc_crc(response, (sizeof(response)-2));          //calculate CRC (don't calculate on the empty CRC fields)
    
    response[sizeof(response)-2] = crc & 0xFF;                      //insert CRC
    response[sizeof(response)-1] = (crc >> 8) & 0xFF;  
    
    if(DEBUG_RS485)
    {
      Serial.print("response");
      for (int i = 0; i < sizeof(response); i++) 
      {
        Serial.print(" ");
        if(response[i]<16)
          Serial.print("0");
        Serial.print(response[i], HEX);
      }
      Serial.println();
    }
    
    SoftSerial.write(response, sizeof(response));                   //Write the response!
    memset(recbuffer, 0, sizeof(recbuffer));                        //reset buffer for incoming message
  }
}
