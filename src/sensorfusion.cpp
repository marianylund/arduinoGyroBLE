#include <Arduino.h>
#include <Arduino_LSM9DS1.h>  // Extended 2.0 LSM9DS1 library written by Femme Verbeek, https://github.com/FemmeVerbeek/Arduino_LSM9DS1
#include <math.h> // M_PI, sqrtf, atan2f, asinf

void setupSensors(){
    if (!IMU.begin())
    { Serial.println("Failed to initialize IMU!");
        while (1);
    } 

    IMU.gyroUnit= RADIANSPERSECOND;   
    // Accelerometer code
    IMU.setAccelFS(0); // ±2g
    IMU.setAccelODR(3); // 119 Hz
    IMU.setAccelOffset(0.006332, -0.009766, -0.014550);
    IMU.setAccelSlope (0.997426, 0.992153, 1.006052);

    // Gyroscope code
    IMU.setGyroFS(0); // ±245 °/s
    IMU.setGyroODR(3); // 119 Hz
    IMU.setGyroOffset (0.345055, 0.619925, -0.242540);
    IMU.setGyroSlope (1.167765, 1.231580, 1.166394);

    // start the filter to run at the sample rate:
    float sensorRate = IMU.getGyroODR(); // The slowest ODR determines the sensor rate, Accel and Gyro share their ODR
    Serial.print("Sensor rate: "); Serial.println(sensorRate);

    delay(5000);

    Serial.println("Gyro settting ");  
    Serial.print("Gyroscope FS= ");   Serial.print(IMU.getGyroFS());
    Serial.print("Gyroscope ODR=");   Serial.println(IMU.getGyroODR());
    Serial.print("Gyro unit=");       Serial.println(IMU.gyroUnit);
}

float getSensorRate(){
    return IMU.getGyroODR();
}

void eulerToQuat(float x, float y, float z, float* quat)
{
    // source: https://stackoverflow.com/questions/12088610/conversion-between-euler-quaternion-like-in-unity3d-engine
    // Expects RADIANS
    float yaw = x;
    float pitch = y;
    float roll = z;

    double yawOver2 = yaw * 0.5f;
    float cosYawOver2 = (float)cos(yawOver2);
    float sinYawOver2 = (float)sin(yawOver2);
    double pitchOver2 = pitch * 0.5f;
    float cosPitchOver2 = (float)cos(pitchOver2);
    float sinPitchOver2 = (float)sin(pitchOver2);
    double rollOver2 = roll * 0.5f;
    float cosRollOver2 = (float)cos(rollOver2);
    float sinRollOver2 = (float)sin(rollOver2);    
    
    quat[0] = sinYawOver2 * cosPitchOver2 * cosRollOver2 + cosYawOver2 * sinPitchOver2 * sinRollOver2; // x
    quat[1] = cosYawOver2 * sinPitchOver2 * cosRollOver2 - sinYawOver2 * cosPitchOver2 * sinRollOver2; // y
    quat[2] = cosYawOver2 * cosPitchOver2 * sinRollOver2 - sinYawOver2 * sinPitchOver2 * cosRollOver2; // z
    quat[3] = cosYawOver2 * cosPitchOver2 * cosRollOver2 + sinYawOver2 * sinPitchOver2 * sinRollOver2; // w
}

// Simple only using the gyroscope
void getRotation(float* att, unsigned long *lastUpdate){
    if (!IMU.accelAvailable() || !IMU.gyroAvailable()){
        return;
    }
    static int count=0;

    // values for rotation, expects rad/sec
    float gx, gy, gz;
    IMU.readGyro(gx, gy, gz);

    // Converted from IMU to unity coordinate system
    //          Unity   IMU
    // forward  z       x
    // up       y       z
    // right    x       y
    float u_gx, u_gy, u_gz;
    u_gx = -gy; u_gy = -gz; u_gz = -gx;

    long now = micros();
    float l = *lastUpdate;
    float deltaT = ((now - l)/1000000.0f);
    *lastUpdate = now;

    float deltaG[4];
    eulerToQuat(u_gx * deltaT, u_gy * deltaT, u_gz * deltaT, deltaG);

    float q0, q1, q2, q3;
    q0 = att[3] * deltaG[0] + att[0] * deltaG[3] + att[1] * deltaG[2] - att[2] * deltaG[1];  // x
    q1 = att[3] * deltaG[1] - att[0] * deltaG[2] + att[1] * deltaG[3] + att[2] * deltaG[0];  // y
    q2 = att[3] * deltaG[2] + att[0] * deltaG[1] - att[1] * deltaG[0] + att[2] * deltaG[3];  // z
    q3 = att[3] * deltaG[3] - att[0] * deltaG[0] - att[1] * deltaG[1] - att[2] * deltaG[2];  // w

    att[0] = q0;
    att[1] = q1;
    att[2] = q2;
    att[3] = q3;

    // count++;
    // if (count > 20) // The optimum is probably something close to the refresh rate of your monitor.
    // {  
    //     count = 0;
    //     Serial.print(q0);
    //     Serial.print('\t');
    //     Serial.print(q1);
    //     Serial.print('\t');
    //     Serial.print(q2);
    //     Serial.print('\t');
    //     Serial.print(q3);
    //     Serial.print('\t');
    //     Serial.println(*lastUpdate);

    // }
}



// Expects radians per second, I think
void getRotationComplementaryFilter(float* att, unsigned long *lastUpdate){
    if (!IMU.accelAvailable() || !IMU.gyroAvailable()){
        return;
    }

    float q0 = att[0];
    float q1 = att[1];
    float q2 = att[2];
    float q3 = att[3];

    float yawAtt   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f / PI;   
    float pitchAtt = -asin(2.0f * (q1 * q3 - q0 * q2)) * 180.0f / PI;
    float rollAtt  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 180.0f / PI;

    // values for acceleration & rotation:
    float ax, ay, az;
    float gx, gy, gz;
    static int count=0;  

    // read all 9 DOF of the IMU:
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyro(gx, gy, gz);

    long now = micros();
    float alpha = 0.98; // default complementary filter coefficient
    float l = *lastUpdate;
    float deltaT = ((now - l)/1000000.0f); // set integration time by time elapsed since last filter update
    *lastUpdate = now;

    // The complementary filter code is from NexgenAHRS. ref: https://bitbucket.org/David_Such/nexgen_ahrs/src/main/

    //  Roll (Theta) and Pitch (Phi) from accelerometer
    float rollAcc = atan2(ay, az);
    float pitchAcc = atan2(-ax, sqrt(pow(ay, 2) + pow(az, 2)));

    // Auxiliary variables to avoid repeated arithmetic
    float _halfdT = deltaT * 0.5f;
    float _cosTheta = cos(rollAcc);
    float _cosPhi = cos(pitchAcc);
    float _sinTheta = sin(rollAcc);
    float _sinPhi = sin(pitchAcc);
    float _halfTheta = rollAcc * 0.5f;
    float _halfPhi = pitchAcc * 0.5f;
    float _cosHalfTheta = cos(_halfTheta);
    float _cosHalfPhi = cos(_halfPhi);
    float _sinHalfTheta = sin(_halfTheta);
    float _sinHalfPhi = sin(_halfPhi);
    
    //  Calculate Attitude Quaternion
    //  ref: https://ahrs.readthedocs.io/en/latest/filters/complementary.html
    att[0] = att[0] - _halfdT * gx * att[1] - _halfdT * gy * att[2] - _halfdT * gz * att[3];
    att[1] = att[1] + _halfdT * gx * att[0] - _halfdT * gy * att[3] + _halfdT * gz * att[2];
    att[2] = att[2] + _halfdT * gx * att[3] + _halfdT * gy * att[0] - _halfdT * gz * att[1];
    att[3] = att[3] - _halfdT * gx * att[2] + _halfdT * gy * att[1] + _halfdT * gz * att[0];

    float yaw = 0;
    // More auxiliary variables to avoid repeated arithmetic
    float _halfPsi = yaw * 0.5f;
    float _cosHalfPsi = cos(_halfPsi);
    float _sinHalfPsi = sin(_halfPsi);

    // We transform the roll-pitch-yaw angles to a quaternion representation
    float qam[4];
    qam[0] = _cosHalfPhi * _cosHalfTheta * _cosHalfPsi + _sinHalfPhi * _sinHalfTheta * _sinHalfPsi;
    qam[1] = _sinHalfPhi * _cosHalfTheta * _cosHalfPsi - _cosHalfPhi * _sinHalfTheta * _sinHalfPsi;
    qam[2] = _cosHalfPhi * _sinHalfTheta * _cosHalfPsi + _sinHalfPhi * _cosHalfTheta * _sinHalfPsi;
    qam[3] = _cosHalfPhi * _cosHalfTheta * _sinHalfPsi - _sinHalfPsi * _sinHalfTheta * _cosHalfPsi;

    //  Fuse attitude quaternion (att) with qam using complementary filter
    q0 = alpha * att[0] + (1 - alpha) * qam[0];
    q1 = alpha * att[1] + (1 - alpha) * qam[1];
    q2 = alpha * att[2] + (1 - alpha) * qam[2];
    q3 = alpha * att[3] + (1 - alpha) * qam[3];

    att[0] = q0;
    att[1] = q1;
    att[2] = q2;
    att[3] = q3;

    count++;
    if (count > 20) // The optimum is probably something close to the refresh rate of your monitor.
    {  
        count = 0;  

                // Convert to Euler angles:
        float yawRadians   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);   
        float pitchRadians = -asin(2.0f * (q1 * q3 - q0 * q2));
        float rollRadians  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);

        float pitch = pitchRadians * 180.0f / PI;
        float eulerYaw   = yawRadians * 180.0f / PI; 
        float roll  = rollRadians * 180.0f / PI;

        Serial.print(rollAtt);
        Serial.print('\t');
        Serial.print(pitchAtt);
        Serial.print('\t');
        Serial.print(yawAtt);
        Serial.print('\t');
        Serial.print('[');
        Serial.print(*lastUpdate);
        Serial.print(']');
        Serial.print('\t');
        Serial.print('>'); 

        Serial.print(roll);
        Serial.print('\t');
        Serial.print(pitch);
        Serial.print('\t');
        Serial.println(eulerYaw);
        // Serial.print("Gyro: ");
        // Serial.print(xGyro);
        // Serial.print('\t');
        // Serial.print(yGyro);
        // Serial.print('\t');
        // Serial.println(zGyro);

        // Serial.print("Acc: ");
        // Serial.print(xAcc);
        // Serial.print('\t');
        // Serial.print(yAcc);
        // Serial.print('\t');
        // Serial.println(zAcc);

        // Serial.print(_q0);
        // Serial.print('\t');
        // Serial.print(_q1);
        // Serial.print('\t');
        // Serial.print(_q2);
        // Serial.print('\t');
        // Serial.println(_q3);
    }
}



