// #include <Arduino.h>
// #include "sensorfusion.h"


// long previousMillis = 0;  // last timechecked, in ms
// unsigned long micros_per_reading, micros_previous, lastUpdate;
// float att[4];
// void setup()
// {
//     Serial.begin(115200);

//     setupSensors();
//     micros_per_reading = 1000;//1000000 / sensorRate;
//     micros_previous = micros();
//     att[0] = 0;att[1] = 0;att[2] = 0;att[3] = 1;
// }

// void loop()
// {
//     unsigned long micros_now;
//     micros_now = micros();
//     if (micros_now - micros_previous >= micros_per_reading) {
//         getRotation(att, &lastUpdate);
//     } 
// }