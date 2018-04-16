#include "mpufusion.h"

MPUFusion::MPUFusion(){
    Wire.begin();
    I2CwriteByte(MPU9250_ADDRESS,29,0x06);
    I2CwriteByte(MPU9250_ADDRESS,26,0x06);

    I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
    I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
    I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
    I2CwriteByte(MAG_ADDRESS,0x0A,0x16);

    pinMode(13, OUTPUT);

    // initialize data
    getMPUData();
    double roll  = atan(fd.ay / sqrt(fd.ax * fd.ax + fd.az * fd.az)) * RAD_TO_DEG;
    double pitch = atan2(-fd.ax, fd.az) * RAD_TO_DEG;
    kalmanX.setAngle(roll); // Set starting angle
    kalmanY.setAngle(pitch);
    fd.gax = roll;
    fd.gay = pitch;
    fd.cax = roll;
    fd.cay= pitch;
    timer = micros();
}

void MPUFusion::I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void MPUFusion::I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void MPUFusion::getMPUData(){
    uint8_t Buf[14];
    I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
    fd.ax=-(Buf[0]<<8 | Buf[1]);
    fd.ay=-(Buf[2]<<8 | Buf[3]);
    fd.az=Buf[4]<<8 | Buf[5];
    fd.temp=Buf[6]<<8 | Buf[7];
    fd.gx=-(Buf[8]<<8 | Buf[9]);
    fd.gy=-(Buf[10]<<8 | Buf[11]);
    fd.gz=Buf[12]<<8 | Buf[13];
}

void MPUFusion::getMagData(){
    uint8_t ST1;
    do
    {
        I2Cread(MAG_ADDRESS,0x02,1,&ST1);
    }
    while (!(ST1&0x01));

    uint8_t Mag[7];
    I2Cread(MAG_ADDRESS,0x03,7,Mag);

    fd.mx=-(Mag[3]<<8 | Mag[2]);
    fd.my=-(Mag[1]<<8 | Mag[0]);
    fd.mz=-(Mag[5]<<8 | Mag[4]);
}

void MPUFusion::filterData(){
    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    double roll  = atan(fd.ay / sqrt(fd.ax * fd.ax + fd.az * fd.az)) * RAD_TO_DEG;
    double pitch = atan2(-fd.ax, fd.az) * RAD_TO_DEG;
    double gyroXrate = fd.gx / 131.0; // Convert to deg/s
    double gyroYrate = fd.gy / 131.0; // Convert to deg/s
    if ((pitch < -90 && fd.kay > 90) || (pitch > 90 && fd.kay < -90)) {
        kalmanY.setAngle(pitch);
        fd.cay = pitch;
        fd.kay = pitch;
        fd.gay = pitch;
    } else {
        fd.kay = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
    }
    if (abs(fd.kay) > 90){
        gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    }
    fd.kax = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

//    gax += gyroXrate * dt; // Calculate gyro angle without any filter
//    gay += gyroYrate * dt;
    fd.gax += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    fd.gay += kalmanY.getRate() * dt;

    fd.cax = 0.93 * (fd.cax + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    fd.cay = 0.93 * (fd.cay + gyroYrate * dt) + 0.07 * pitch;

    // Reset the gyro angle when it has drifted too much
    if (fd.gax  < -180 || fd.gax > 180){
        fd.gax = fd.kax ;
    }
    if (fd.gay  < -180 || fd.gay > 180){
        fd.gay = fd.kay;
    }
}

FusionDataType *MPUFusion::getFusionData(){
    getMPUData();
    getMagData();
    filterData();
    return &fd;
}

void MPUFusion::printMPUData(){
    Serial.print(fd.ax); Serial.print("\t");
    Serial.print(fd.ay); Serial.print("\t");
    Serial.print(fd.az); Serial.print("\t");
    Serial.print(fd.gx); Serial.print("\t");
    Serial.print(fd.gy); Serial.print("\t");
    Serial.print(fd.gz); Serial.print("\t");
}

void MPUFusion::printMagData(){
    Serial.print(fd.mx); Serial.print("\t");
    Serial.print(fd.my); Serial.print("\t");
    Serial.print(fd.mz); Serial.print("\t");
}

void MPUFusion::printTempData(){
    Serial.print(fd.temp); Serial.print("\t");
}

void MPUFusion::printGyroAngles(){
    Serial.print(fd.gax); Serial.print("\t");
    Serial.print(fd.gay); Serial.print("\t");
}

void MPUFusion::printComplementaryAngles(){
    Serial.print(fd.cax); Serial.print("\t");
    Serial.print(fd.cay); Serial.print("\t");
}

void MPUFusion::printKalmanAngles(){
    Serial.print(fd.kax); Serial.print("\t");
    Serial.print(fd.kay); Serial.print("\t");
}