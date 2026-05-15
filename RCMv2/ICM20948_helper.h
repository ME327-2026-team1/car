/**
from https://github.com/RCMgames/useful-code/tree/main/sensors/IMUs/ICM20948
*/

#ifndef ICM20948_HELPER_H
#define ICM20948_HELPER_H

#include "ICM_20948.h"
#include <Wire.h>

ICM_20948_I2C IMU;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float accX;
    float accY;
    float accZ;
    float gyrX;
    float gyrY;
    float gyrZ;
    float magX;
    float magY;
    float magZ;
    float q0;
    float q1;
    float q2;
    float q3;
    uint16_t accuracy;
    float temperature;
    unsigned long DMPTimestampMillis;
} IMUData;
IMUData imu;

/**
 * @brief sets up the ICM20948 sensor including the DMP
 * @param  wire: Wire or Wire1
 * @param  AD0_val: 0 if the AD0 pin is low, 1 if the AD0 pin is high, 0 if the address is 68, 1 if the address is 69
 * @retval true if successful
 */
boolean setupICM20948(TwoWire& wire, boolean AD0_val)
{
    bool success = true;
    wire.begin();
    wire.setClock(400000);
    IMU.begin(wire, AD0_val);
    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (IMU.initializeDMP() == ICM_20948_Stat_Ok);
    // Enable the DMP orientation sensor
    success &= (IMU.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
    success &= (IMU.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum rate
    // Enable the FIFO
    success &= (IMU.enableFIFO() == ICM_20948_Stat_Ok);
    // Enable the DMP
    success &= (IMU.enableDMP() == ICM_20948_Stat_Ok);
    // Reset DMP
    success &= (IMU.resetDMP() == ICM_20948_Stat_Ok);
    // Reset FIFO
    success &= (IMU.resetFIFO() == ICM_20948_Stat_Ok);
    return success;
}

/**
 * @brief  sets up the ICM20948 with the defaults for IMU built into the RCM Nibble
 */
boolean nibbleSetupImu()
{
    return setupICM20948(Wire, 1);
}

/**
 * @brief  call this continuously to update the IMUData stored in the imu variable.
 * @retval None
 */
void runIMU()
{
    IMU.getAGMT(); // The values are only updated when you call 'getAGMT'
    imu.accX = IMU.accX() / 1000.0;
    imu.accY = IMU.accY() / 1000.0;
    imu.accZ = IMU.accZ() / 1000.0;
    imu.gyrX = IMU.gyrX();
    imu.gyrY = IMU.gyrY();
    imu.gyrZ = IMU.gyrZ();
    imu.magX = IMU.magX();
    imu.magY = IMU.magY();
    imu.magZ = IMU.magZ();
    imu.temperature = IMU.temp();

    icm_20948_DMP_data_t data;
    IMU.readDMPdataFromFIFO(&data);

    if ((IMU.status == ICM_20948_Stat_Ok) || (IMU.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
    {
        if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
        {
            double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
            double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
            double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
            double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
            imu.q0 = q0;
            imu.q1 = q1;
            imu.q2 = q2;
            imu.q3 = q3;
            imu.accuracy = data.Quat9.Data.Accuracy;

            double q2sqr = q2 * q2;

            // roll (x-axis rotation)
            double t0 = +2.0 * (q0 * q1 + q2 * q3);
            double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
            double roll = atan2(t0, t1) * 180.0 / PI;

            // pitch (y-axis rotation)
            double t2 = +2.0 * (q0 * q2 - q3 * q1);
            t2 = t2 > 1.0 ? 1.0 : t2;
            t2 = t2 < -1.0 ? -1.0 : t2;
            double pitch = asin(t2) * 180.0 / PI;

            // yaw (z-axis rotation)
            double t3 = +2.0 * (q0 * q3 + q1 * q2);
            double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
            double yaw = atan2(t3, t4) * 180.0 / PI;

            imu.DMPTimestampMillis = millis();
            imu.roll = roll;
            imu.pitch = pitch;
            imu.yaw = yaw;
        }
    }
}

#endif // ICM20948_HELPER_H
