#include <Arduino.h>
#include <PS4Controller.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

#include "esp_netif.h"

#include "BLEDevice.h"

unsigned long lastTimeStamp = 0;

uint8_t new_mac[] = {0x7c, 0x9e, 0xbd, 0xfa, 0x0b, 0xac};

// original code https://github.com/un0038998/PS4Controller_ESP32/blob/main/Remove_Paired_Devices/Remove_Paired_Devices.ino
void removePairedDevices(){
  uint8_t pairedDeviceBtAddr[20][6];  
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  for(int i = 0; i < count; i++) 
  {
    uint8_t* address = pairedDeviceBtAddr[i];
    Serial.printf("drop device - %02x:%02x:%02x:%02x:%02x:%02x\n", address[0],address[1],address[2],address[3],address[4],address[5]);
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }
}

void notify()
{
  char messageString[200];
  sprintf(messageString, "%4d,%4d,%4d,%4d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%4d,%4d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d",
  PS4.LStickX(),
  PS4.LStickY(),
  PS4.RStickX(),
  PS4.RStickY(),
  PS4.Left(),
  PS4.Down(),
  PS4.Right(),
  PS4.Up(),
  PS4.Square(),
  PS4.Cross(),
  PS4.Circle(),
  PS4.Triangle(),
  PS4.L1(),
  PS4.R1(),
  PS4.L2Value(),
  PS4.R2Value(),  
  PS4.Share(),
  PS4.Options(),
  PS4.PSButton(),
  PS4.Touchpad(),
  PS4.Charging(),
  PS4.Audio(),
  PS4.Mic(),
  PS4.Battery());

  //Only needed to print the message properly on serial monitor. Else we dont need it.
  if (millis() - lastTimeStamp > 50)
  {
    Serial.println(messageString);
    lastTimeStamp = millis();
  }
}

void onConnect()
{
  Serial.println("Connected!.");
}

void onDisConnect()
{
  Serial.println("Disconnected!.");    
}

void BLEChangeMAC()
{
  if (BLEDevice::getInitialized()) BLEDevice::deinit(true);
  esp_base_mac_addr_set(new_mac);
  BLEDevice::init("ESP32");
}

void setup() {
  Serial.begin(115200);

  BLEChangeMAC();
  
  PS4.begin();

  removePairedDevices(); // to avoid re-pairing issues

  const uint8_t* address = esp_bt_dev_get_address();
  char str[100];
  sprintf(str, "ESP32's Bluetooth MAC address is - %02x:%02x:%02x:%02x:%02x:%02x\n", address[0],address[1],address[2],address[3],address[4],address[5]);
  Serial.println(str);

  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);

  Serial.println("Ready.");
}

void loop() 
{
  //  if (PS4.isConnected()) {
  //     notify();

  //     delay(1000);
  //  }
}