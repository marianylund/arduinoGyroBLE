#include <Arduino.h>
#include <ArduinoBLE.h>
#include "sensorfusion.h"

const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* initCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
const char* initBoolCharacteristicUuid = "19b10002-e8f2-537e-4f6c-d104768a1214";
const char* quatCharacteristicUuid = "19b10004-e8f2-537e-4f6c-d104768a1214";
const int quatBufferSizeBytes = 41;

BLEService imuService(deviceServiceUuid); 
BLECharacteristic initCharacteristic(initCharacteristicUuid, BLERead | BLENotify | BLEWrite, quatBufferSizeBytes);
BLECharacteristic quatCharacteristic(quatCharacteristicUuid, BLERead | BLENotify, quatBufferSizeBytes);
BLEBoolCharacteristic initBoolCharacteristic(initBoolCharacteristicUuid, BLERead | BLENotify | BLEWrite);

long previousMillis = 0;  // last timechecked, in ms
unsigned long micros_per_reading_sensor, micros_per_reading_ble, micros_previous_ble, micros_previous_sensor, lastUpdate;
float att[4], quat[4];

void setup()
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected


  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  // Setup bluetooth
  BLE.setLocalName("Arduino");
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(initCharacteristic);
  imuService.addCharacteristic(quatCharacteristic);
  imuService.addCharacteristic(initBoolCharacteristic);
  BLE.addService(imuService);

  char buffer[quatBufferSizeBytes];
  sprintf(buffer, "0,0,0,1;");
  initCharacteristic.writeValue(buffer);
  initBoolCharacteristic.writeValue(1);

  Serial.print("Quaternion characteristic established with buffer size: "); Serial.println(quatBufferSizeBytes);
  
  // start advertising
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");

  setupSensors();

  micros_per_reading_sensor = 1000000 / getSensorRate(); // around 119 per second
  micros_per_reading_ble = 1000000 / 30; // around 30 per second
  micros_previous_ble = micros();
  micros_previous_sensor = micros();
}

// from: https://github.com/arduino/Arduino/blob/a2e7413d229812ff123cb8864747558b270498f1/hardware/arduino/sam/cores/arduino/avr/dtostrf.c
  char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
    char fmt[20];
    sprintf(fmt, "%%%d.%df", width, prec);
    sprintf(sout, fmt, val);
    return sout;
}

bool readInitRotData(){
  if(initBoolCharacteristic.value()){
    char readBuffer[quatBufferSizeBytes];
    initCharacteristic.readValue(readBuffer, quatBufferSizeBytes);
    char readData[quatBufferSizeBytes];
    strncpy(readData, readBuffer, quatBufferSizeBytes);

    char *ptr = strtok(readData, ",");
    int i = 0;
    while (ptr != NULL)
    {
      quat[i] = atof(ptr);
      i++;
      ptr = strtok(NULL, ",");
    }

    char test[quatBufferSizeBytes];
    sprintf(test, "%.2f, %.2f, %.2f, %.2f", quat[0], quat[1], quat[2], quat[3]);
    Serial.println(test);

    initBoolCharacteristic.writeValue(0);
    return 1;
  }
  return 0;  
}

void sendSensorData(){
  unsigned long micros_now;
  micros_now = micros();
  bool timeToUpdateBleData = false;
  if(micros_now - micros_previous_ble >= micros_per_reading_ble){
    timeToUpdateBleData = true;
    micros_previous_ble = micros_previous_ble + micros_per_reading_ble;
  }

  if(timeToUpdateBleData && readInitRotData()){
    // Overwrite the orientation data gotten from Vuforia
    att[0] = quat[0];
    att[1] = quat[1];
    att[2] = quat[2];
    att[3] = quat[3];
  }
  // Get new updated rotation based on the previous orientation
  getRotation(att, &lastUpdate);

  if(timeToUpdateBleData){
    // Convert floats to strings
    char q0[10], q1[10], q2[10], q3[10];
    dtostrf(att[0], 5, 2, q0);
    dtostrf(att[1], 5, 2, q1);
    dtostrf(att[2], 5, 2, q2);
    dtostrf(att[3], 5, 2, q3);
    // Merge string values into one with ',' as a separator 
    // important to have it at the end too - to ignore the extra data after the floats
    char buffer[quatBufferSizeBytes];
    sprintf(buffer, "%s,%s,%s,%s;", q0, q1, q2, q3);
  
    quatCharacteristic.writeValue(buffer);
  }
}

void loop()
{ 
  // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a BLE central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // while the central is connected:
    while (central.connected()) {
      unsigned long micros_now;
      micros_now = micros();

      if (micros_now - micros_previous_sensor >= micros_per_reading_sensor) {
          sendSensorData();
          micros_previous_sensor = micros_previous_sensor + micros_per_reading_sensor;
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}



