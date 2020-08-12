#ifndef _ICM20948_H
#define _ICM20948_H

#include "SpiDevice.h"
#include "ICM20948Def.h"

class ICM20948 {
  public:
    enum class GyroSampleRate : uint8_t { 
      LPF_12100HZ = GYRO_LPF_12100HZ,
      LPF_360HZ   = GYRO_LPF_360HZ,
      LPF_200HZ   = GYRO_LPF_200HZ,
      LPF_150HZ   = GYRO_LPF_150HZ,
      LPF_120HZ   = GYRO_LPF_120HZ,
      LPF_51HZ    = GYRO_LPF_51HZ,
      LPF_24HZ    = GYRO_LPF_24HZ,
      LPF_12HZ    = GYRO_LPF_12HZ,
      LPF_6HZ     = GYRO_LPF_6HZ
    };
    enum class GyroScale : uint8_t {
      MAX_250DPS = GYRO_FULLSCALE_250DPS ,
      MAX_500DPS = GYRO_FULLSCALE_500DPS ,
      MAX_1000DPS= GYRO_FULLSCALE_1000DPS,
      MAX_2000DPS= GYRO_FULLSCALE_2000DPS
    };
    enum class AccSampleRate : uint8_t {
      LPF_1210HZ=ACCEL_LPF_1210HZ,
      LPF_470HZ =ACCEL_LPF_470HZ ,
      LPF_246HZ =ACCEL_LPF_246HZ ,
      LPF_111HZ =ACCEL_LPF_111HZ ,
      LPF_50HZ  =ACCEL_LPF_50HZ  ,
      LPF_24HZ  =ACCEL_LPF_24HZ  ,
      LPF_12HZ  =ACCEL_LPF_12HZ  ,
      LPF_6HZ   =ACCEL_LPF_6HZ
    };
    enum class AccScale : uint8_t {
      MAX_2G = ACCEL_FULLSCALE_2G,
      MAX_4G = ACCEL_FULLSCALE_4G,
      MAX_8G = ACCEL_FULLSCALE_8G,
      MAX_16G= ACCEL_FULLSCALE_16G
    };
    enum class MagSampleRate : uint8_t {
      Off = 0,
      Single = 0x01,
      Mode10Hz = 0x02,
      Mode20Hz = 0x04,
      Mode50Hz = 0x06,
      Mode100Hz = 0x08
    };
  private:
    u_int8_t m_lastBank;
    SpiDevice m_spi;

    double m_gyroScale;
    double m_accScale;

    bool NeedsBankSwitch(uint16_t addr,uint8_t& bank,uint8_t& regAddr);
    bool WriteReg( uint16_t addr, uint8_t data );
    bool WriteAK09916Reg( uint8_t addr, uint8_t const data );
    bool ReadReg( uint16_t addr, uint8_t& retData );
    bool ReadRegs( uint16_t addr, uint8_t *retData, uint8_t length );
    bool ReadAK09916Reg( uint8_t addr, uint8_t& retData );

  public:
    ICM20948();

    bool Init(unsigned char spiBus,unsigned char spiSelect);
    void Close();

    bool DisableGyro();
    bool EnableGyro(GyroSampleRate rate,GyroScale scale);
    double GetGyroSensitivity(){return m_gyroScale;}
    bool ReadGyro(double& gx,double& gy,double& gz);
    bool DisableAcc();
    bool EnableAcc(AccSampleRate rate,AccScale scale);
    double GetAccSensitivity(){return m_accScale;}
    bool ReadAcc(double& ax,double& ay,double& az);
    bool ReadAccGyr(double& ax,double& ay,double& az,double& gx,double& gy,double& gz);
    bool ReadAccGyrRaw(int16_t* accRaw,int16_t* gyrRaw);
    bool DisableMag();
    bool EnableMag(MagSampleRate rate);
    bool ReadMag(double& mx,double& my,double& mz);
    uint8_t whoami();
    uint8_t whoamiMag();
};

#endif