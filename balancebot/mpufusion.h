#ifndef MPUFUSION_H
#define MPUFUSION_H
#include <Arduino.h>
#include <Wire.h>
#include <Kalman.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

typedef struct {
    double ax, ay, az, gx, gy, gz, temp, mx, my, mz;
    double gax, gay, cax, cay, kax, kay;
} FusionDataType;

class MPUFusion {
    public:
        MPUFusion();
        void printMPUData();
        void printMagData();
        void printTempData();
        void printGyroAngles();
        void printComplementaryAngles();
        void printKalmanAngles();
        FusionDataType *getFusionData();

    private:
        uint32_t timer;
        FusionDataType fd;
        Kalman kalmanX;
        Kalman kalmanY;
        void getMPUData();
        void getMagData();
        void filterData();
        void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
        void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);
};

#endif