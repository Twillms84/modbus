#ifndef MODBUS_H
#define MODBUS_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <PubSubClient.h>

void handleInverter(SoftwareSerial& SoftSerial, PubSubClient& client, bool DEBUG_RS485);
uint16_t calc_crc(uint8_t* data, uint8_t length);

#endif