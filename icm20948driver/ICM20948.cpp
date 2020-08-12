#include "ICM20948Def.h"
#include "ICM20948.h"
#include <unistd.h>
#include <stdio.h>

#define TRYorRETURN(retBoolAction,msg)  if (!(retBoolAction)) { printf("FAILED:");printf(msg);printf("\n"); return false; }
#define sleepMs(ms)                     usleep(1000*ms)

//-----------------------------------------------------------------------------------------------

ICM20948::ICM20948() : m_lastBank(-1),m_spi(),m_gyroScale(0.0),m_accScale(0.0)
{
}

/*-----------------------------------------------------------------------------------------------
                                    REGISTER READ & WRITE
usage: use these methods to read and write ICM20948 registers over SPI
-----------------------------------------------------------------------------------------------*/
bool ICM20948::NeedsBankSwitch(uint16_t addr,uint8_t& bank,uint8_t& regAddr)
{
    bank = addr >> 7;
    regAddr = addr & 0x7F;
    return bank != m_lastBank;
}

bool ICM20948::WriteReg( uint16_t addr, uint8_t data )
{
    uint8_t bank;
    uint8_t regAddr;
    if (NeedsBankSwitch(addr,bank,regAddr)){
        TRYorRETURN(m_spi.Write(REG_BANK_SEL,bank<<4),"Switch Bank");
    }

    TRYorRETURN(m_spi.Write(regAddr,data),"Writing data");
    
    return true;
}

bool ICM20948::ReadReg( uint16_t addr, uint8_t& retData )
{
    uint8_t bank;
    uint8_t regAddr;
    if (NeedsBankSwitch(addr,bank,regAddr)){
        TRYorRETURN(m_spi.Write(REG_BANK_SEL,bank<<4),"Switch Bank");
    }

    TRYorRETURN(m_spi.Read(regAddr,retData),"reading data");
    
    return true;
}

bool ICM20948::ReadRegs( uint16_t addr, uint8_t* retData, uint8_t nbBytes )
{
    uint8_t bank;
    uint8_t regAddr;
    if (NeedsBankSwitch(addr,bank,regAddr)){
        TRYorRETURN(m_spi.Write(REG_BANK_SEL,bank<<4),"Switch Bank");
    }

    TRYorRETURN(m_spi.Read(regAddr,nbBytes,retData),"reading data");
    
    return true;
}

bool ICM20948::ReadAK09916Reg( uint8_t addr, uint8_t& retData )
{
    uint8_t oldCtrl;
    uint8_t oldAddr;
    uint8_t oldReg;

    TRYorRETURN(ReadReg(REG_I2C_SLV0_CTRL,oldCtrl) && oldCtrl == 0,"Check if SLV0_CTRL is idle");
    TRYorRETURN(ReadReg(REG_I2C_SLV0_ADDR,oldAddr) && ReadReg(REG_I2C_SLV0_REG,oldReg),"Set aside current config of SLV0");

    const uint8_t nbBytes = 1;
    TRYorRETURN(
        WriteReg(REG_I2C_SLV0_ADDR, AK09916_I2C_SLAVE_ADDRESS | INV_MPU_BIT_I2C_READ)
        && WriteReg(REG_I2C_SLV0_REG, addr)
        && WriteReg(REG_I2C_SLV0_CTRL, INV_MPU_BIT_SLV_EN | nbBytes)
        ,"Send out read to SLV0");
    sleepMs(20);//wait for the slave data to be read
    TRYorRETURN(
        WriteReg(REG_I2C_SLV0_CTRL, oldCtrl)
        && WriteReg(REG_I2C_SLV0_ADDR, oldAddr)
        && WriteReg(REG_I2C_SLV0_REG, oldReg)
        && ReadReg(REG_EXT_SLV_SENS_DATA_00, retData)
        , "Restore state and read received i2c data");

    return true;
}

bool ICM20948::WriteAK09916Reg( uint8_t addr, uint8_t const data )
{
    uint8_t oldCtrl;
    uint8_t oldAddr;
    uint8_t oldReg;
    uint8_t oldDataOut;

    TRYorRETURN(ReadReg(REG_I2C_SLV1_CTRL,oldCtrl) && oldCtrl == 0,"Check if SLV1_CTRL is idle");
    TRYorRETURN(ReadReg(REG_I2C_SLV1_ADDR,oldAddr)
                && ReadReg(REG_I2C_SLV1_REG,oldReg)
                && ReadReg(REG_I2C_SLV1_DO,oldDataOut)
                ,"Set aside current config of SLV1");

    const uint8_t nbBytes = 1;
    TRYorRETURN(
        WriteReg(REG_I2C_SLV1_ADDR, AK09916_I2C_SLAVE_ADDRESS)
        && WriteReg(REG_I2C_SLV1_REG, addr)
        && WriteReg(REG_I2C_SLV1_CTRL, INV_MPU_BIT_SLV_EN | nbBytes)
        && WriteReg(REG_I2C_SLV1_DO, data)
        ,"Send out read to SLV1");
    sleepMs(20);//wait for the slave data to be sent
    TRYorRETURN(
        WriteReg(REG_I2C_SLV1_CTRL, oldCtrl)
        && WriteReg(REG_I2C_SLV1_DO, oldDataOut)
        && WriteReg(REG_I2C_SLV1_ADDR, oldAddr)
        && WriteReg(REG_I2C_SLV1_REG, oldReg)
        , "Restore state");

    return true;
}

bool ICM20948::Init(unsigned char spiBus,unsigned char spiSelect)
{
    TRYorRETURN(m_spi.Open(spiBus,spiSelect),"Opening SPI-connection");

    TRYorRETURN(WriteReg(REG_I2C_SLV0_CTRL,  0), "disable SLV0");
    TRYorRETURN(WriteReg(REG_I2C_SLV1_CTRL,  0), "disable SLV1");
    TRYorRETURN(WriteReg(REG_I2C_SLV2_CTRL,  0), "disable SLV2");
    TRYorRETURN(WriteReg(REG_I2C_SLV3_CTRL,  0), "disable SLV3");
    TRYorRETURN(WriteReg(REG_I2C_SLV4_CTRL,  0), "disable SLV4");
    TRYorRETURN(WriteReg(REG_USER_CTRL,      0), "disable fifo and possible active I2C functions");
    
    //wait for eventual shutdown of i2c
    sleepMs(50);

    //perform reboot
    TRYorRETURN(WriteReg(REG_PWR_MGMT_1,  BIT_H_RESET), "initiate ICM20948 reset");
    //wait for reset to happen
    sleepMs(100);
    TRYorRETURN(WriteReg(REG_PWR_MGMT_1,  BIT_CLK_PLL), "select clock");
    sleepMs(30);
    TRYorRETURN(WriteReg(REG_USER_CTRL,  BIT_I2C_MST_EN), "Enable i2c master mode");
    TRYorRETURN(WriteReg(REG_I2C_MST_CTRL,  I2C_MST_FREQ_345_6_MAX400HZ), "Failed to set REG_PWR_MGMT_2");

    TRYorRETURN(whoami() == ICM20948_DEVICE_ID,"Unexpected WhoAmI for ACC/GYR");
    TRYorRETURN(whoamiMag() == AK09916_WIA2_ID,"Unexpected WhoAmI for MAG:AK09916");
    return true;
}

void ICM20948::Close()
{
    m_spi.Close();
}

bool ICM20948::DisableGyro()
{
    uint8_t curVal;
    TRYorRETURN(
        ReadReg(REG_PWR_MGMT_2, curVal)
        && WriteReg(REG_PWR_MGMT_2, curVal | BIT_PWR_GYRO_STBY)
        ,"Disable gyro");
    return true;
}

bool ICM20948::EnableGyro(GyroSampleRate rate,GyroScale scale)
{
    DisableGyro();
    uint8_t curVal;
    TRYorRETURN(WriteReg(REG_GYRO_CONFIG_1, (uint8_t)rate | (uint8_t)scale)
        ,"Set gyro config");
    switch (scale)
    {
        case ICM20948::GyroScale::MAX_250DPS : m_gyroScale = 1.0/131 ; break;
        case ICM20948::GyroScale::MAX_500DPS : m_gyroScale = 1.0/65.5; break;
        case ICM20948::GyroScale::MAX_1000DPS: m_gyroScale = 1.0/32.8; break;
        case ICM20948::GyroScale::MAX_2000DPS: m_gyroScale = 1.0/16.4; break;
        default:m_gyroScale=0; break;
    }
    printf("m_gyroscale: %f\n",m_gyroScale);
    TRYorRETURN(
        ReadReg(REG_PWR_MGMT_2, curVal)
        && WriteReg(REG_PWR_MGMT_2, curVal & ~BIT_PWR_GYRO_STBY)
        ,"Enable gyro");
    return true;
}

bool ICM20948::DisableAcc()
{
    uint8_t curVal;
    TRYorRETURN(
        ReadReg(REG_PWR_MGMT_2, curVal)
        && WriteReg(REG_PWR_MGMT_2, curVal | BIT_PWR_ACCEL_STBY)
        ,"Disable gyro");
    return true;
}

bool ICM20948::EnableAcc(AccSampleRate rate,AccScale scale)
{
    DisableAcc();
    uint8_t curVal;
    TRYorRETURN(WriteReg(REG_GYRO_CONFIG_1, (uint8_t)rate | (uint8_t)scale)
        ,"Set acc config");
    switch (scale)
    {
        case ICM20948::AccScale::MAX_2G : m_accScale = 1.0/16384 ; break;
        case ICM20948::AccScale::MAX_4G : m_accScale = 1.0/8192; break;
        case ICM20948::AccScale::MAX_8G : m_accScale = 1.0/4096; break;
        case ICM20948::AccScale::MAX_16G: m_accScale = 1.0/2048; break;
        default:m_accScale=0; break;
    }
    printf("m_accScale: %f\n",m_accScale);
    TRYorRETURN(
        ReadReg(REG_PWR_MGMT_2, curVal)
        && WriteReg(REG_PWR_MGMT_2, curVal & ~BIT_PWR_ACCEL_STBY)
        ,"Enable acc");
    return true;
}

bool ICM20948::ReadGyro(double& gx,double& gy,double& gz)
{
    const uint8_t nbBytes = 6;
    uint8_t data[nbBytes];
    TRYorRETURN(ReadRegs(REG_GYRO_XOUT_H_SH,data,nbBytes)
        ,"Reading gyro data");
//    printf("raw: 0x%2X%2X 0x%2X%2X 0x%2X%2X\n",data[0],data[1],data[2],data[3],data[4],data[5]);
    gx = ((double)(int16_t)((data[0] << 8) | data[1])) * m_gyroScale;
    gy = ((double)(int16_t)((data[2] << 8) | data[3])) * m_gyroScale;
    gz = ((double)(int16_t)((data[4] << 8) | data[5])) * m_gyroScale;
    return true;
}

bool ICM20948::ReadAcc(double& ax,double& ay,double& az)
{
    const uint8_t nbBytes = 6;
    uint8_t data[nbBytes];
    TRYorRETURN(ReadRegs(REG_ACCEL_XOUT_H_SH,data,nbBytes)
        ,"Reading acc data");
//    printf("raw: 0x%2X%2X 0x%2X%2X 0x%2X%2X\n",data[0],data[1],data[2],data[3],data[4],data[5]);
    ax = ((double)(int16_t)((data[0] << 8) | data[1])) * m_accScale;
    ay = ((double)(int16_t)((data[2] << 8) | data[3])) * m_accScale;
    az = ((double)(int16_t)((data[4] << 8) | data[5])) * m_accScale;
    return true;
}

bool ICM20948::ReadAccGyr(double& ax,double& ay,double& az,double& gx,double& gy,double& gz)
{
    int16_t acc[3],gyr[3];
    ReadAccGyrRaw(acc,gyr);
//    printf("raw: 0x%2X%2X 0x%2X%2X 0x%2X%2X\n",data[0],data[1],data[2],data[3],data[4],data[5]);
    ax = ((double)acc[0]) * m_accScale;
    ay = ((double)acc[1]) * m_accScale;
    az = ((double)acc[2]) * m_accScale;
    gx = ((double)gyr[0]) * m_gyroScale;
    gy = ((double)gyr[1]) * m_gyroScale;
    gz = ((double)gyr[2]) * m_gyroScale;
    return true;
}

bool ICM20948::ReadAccGyrRaw(int16_t* accRaw,int16_t* gyrRaw)
{
    const uint8_t nbBytes = 12;
    uint8_t data[nbBytes];
    TRYorRETURN(ReadRegs(REG_ACCEL_XOUT_H_SH,data,nbBytes)
        ,"Reading acc data");

    accRaw[0] = (data[0]<<8)|data[1];
    accRaw[1] = (data[2]<<8)|data[3];
    accRaw[2] = (data[4]<<8)|data[5];
    gyrRaw[0] = (data[6]<<8)|data[7];
    gyrRaw[1] = (data[8]<<8)|data[9];
    gyrRaw[2] = (data[12]<<8)|data[11];
    // memcpy(accRaw,data,3*sizeof(int16_t));
    // memcpy(gyrRaw,data+6,3*sizeof(int16_t));
}

bool ICM20948::DisableMag()
{
    TRYorRETURN(
        WriteReg(REG_I2C_SLV0_CTRL, 0)
        && WriteReg(REG_I2C_SLV0_ADDR, 0)
        && WriteReg(REG_I2C_SLV0_REG, 0)
        ,"disable slave read to SLV0");

    TRYorRETURN(
        WriteAK09916Reg(REG_AK09916_CNTL2, (uint8_t)MagSampleRate::Off)
        ,"Disable magnetometer");
    sleepMs(1); //needed according to datasheet
    return true;
}

bool ICM20948::EnableMag(MagSampleRate rate)
{
    DisableMag();

    //configure slave i2c reading
    uint8_t oldCtrl;
    TRYorRETURN(ReadReg(REG_I2C_SLV0_CTRL,oldCtrl) && oldCtrl == 0,"Check if SLV0_CTRL is idle");

    const uint8_t nbBytes = 9;//read 9 bytes from AK09916
    TRYorRETURN(
        WriteReg(REG_I2C_SLV0_ADDR, AK09916_I2C_SLAVE_ADDRESS | INV_MPU_BIT_I2C_READ)
        && WriteReg(REG_I2C_SLV0_REG, REG_AK09916_STATUS1)
        && WriteReg(REG_I2C_SLV0_CTRL, INV_MPU_BIT_SLV_EN | nbBytes)
        ,"Send out read to SLV0");

    //enable AK09916
    TRYorRETURN(WriteAK09916Reg(REG_AK09916_CNTL2, (uint8_t)rate)
        ,"Set magnetometer mode");
    return true;
}

bool ICM20948::ReadMag(double& mx,double& my,double& mz)
{
    const double magScale = 4912.0 / 32752.0;
    const uint8_t nbBytes = 9;
    uint8_t data[nbBytes];
    TRYorRETURN(ReadRegs(REG_EXT_SLV_SENS_DATA_00,data,nbBytes)
        ,"Reading external i2c data");
    //printf("ST1: 0x%2X data: 0x%2X%2X 0x%2X%2X 0x%2X%2X ST2:0x%2X\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[8]);
    mx = ((double)(int16_t)((data[2] << 8) | data[1])) * magScale;
    my = ((double)-(int16_t)((data[4] << 8) | data[3])) * magScale;
    mz = ((double)-(int16_t)((data[6] << 8) | data[5])) * magScale;
    return true;
}

uint8_t ICM20948::whoami()
{
    uint8_t regValue;
    if (!ReadReg(REG_WHO_AM_I, regValue))
        return 0;
    return regValue;
}

uint8_t ICM20948::whoamiMag()
{
    uint8_t regValue;
    if (!ReadAK09916Reg(REG_AK09916_WIA2, regValue))
        return 0;
    return regValue;
}
