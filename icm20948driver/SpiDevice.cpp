#include "SpiDevice.h"

#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdlib.h>
#include <linux/spi/spidev.h>

#define SPI_ERROR(fmt) printf(fmt)
#define SPI_ERROR1(fmt,a1) printf(fmt,a1)
#define SPI_ERROR2(fmt,a1,a2) printf(fmt,a1,a2)
#define SPI_ERROR3(fmt,a1,a2,a3) printf(fmt,a1,a2,a3)


SpiDevice::SpiDevice() : m_fd(-1) {};

SpiDevice::~SpiDevice() {};

bool SpiDevice::Open(unsigned char spiBus,unsigned char spiSelect)
{
    char buf[32];
    sprintf(buf, "/dev/spidev%d.%d", spiBus, spiSelect);
    unsigned char SPIMode = SPI_MODE_0;
    unsigned char SPIBits = 8;
    uint32_t SPISpeed = 1000000;

    m_fd = open(buf, O_RDWR);
    if (m_fd < 0) {
        SPI_ERROR2("Failed to open SPI bus %d, select %d\n", spiBus, spiSelect);
        m_fd = -1;
        return false;
    }

    if (ioctl(m_fd, SPI_IOC_WR_MODE, &SPIMode) < 0) {
        SPI_ERROR1("Failed to set WR SPI_MODE0 on bus %d", spiBus);
        close(m_fd);
        return false;
    }

    if (ioctl(m_fd, SPI_IOC_RD_MODE, &SPIMode) < 0) {
        SPI_ERROR1("Failed to set RD SPI_MODE0 on bus %d", spiBus);
        close(m_fd);
        return false;
    }

    if (ioctl(m_fd, SPI_IOC_WR_BITS_PER_WORD, &SPIBits) < 0) {
        SPI_ERROR1("Failed to set WR 8 bit mode on bus %d", spiBus);
        close(m_fd);
        return false;
    }

    if (ioctl(m_fd, SPI_IOC_RD_BITS_PER_WORD, &SPIBits) < 0) {
        SPI_ERROR1("Failed to set RD 8 bit mode on bus %d", spiBus);
        close(m_fd);
        return false;
    }

    if (ioctl(m_fd, SPI_IOC_WR_MAX_SPEED_HZ, &SPISpeed) < 0) {
            SPI_ERROR2("Failed to set WR %dHz on bus %d", SPISpeed, spiBus);
            close(m_fd);
            return false;
    }

    if (ioctl(m_fd, SPI_IOC_RD_MAX_SPEED_HZ, &SPISpeed) < 0) {
            SPI_ERROR2("Failed to set RD %dHz on bus %d", SPISpeed, spiBus);
            close(m_fd);
            return false;
    }
    return true;
}

void SpiDevice::Close()
{
    if (m_fd >= 0) {
        close(m_fd);
        m_fd = -1;
    }
}

bool spiWrite(int fd, uint8_t *data, uint8_t length){
    struct spi_ioc_transfer wrIOC;

    memset(&wrIOC, 0, sizeof(wrIOC));
    wrIOC.tx_buf = (unsigned long) data;
    wrIOC.rx_buf = 0;
    wrIOC.len = length;
    return ioctl(fd, SPI_IOC_MESSAGE(1), &wrIOC);
}

bool SpiDevice::Write(uint8_t regAddr, uint8_t const data)
{
    return Write(regAddr, 1, &data);
}

bool SpiDevice::Write(uint8_t regAddr, uint8_t length, uint8_t const *data)
{
    int result;
    uint8_t txBuff[SPI_MAX_WRITE_LEN + 1];

    if (length == 0) {
        result = spiWrite(m_fd,&regAddr, 1);

        if (result < 0) {
            SPI_ERROR1("write of regAddr 0x%2X failed\n", regAddr);
            return false;
        } else if (result != 1) {
            SPI_ERROR1("write of regAddr 0x%2X failed (nothing written)\n, regAddr", regAddr);
            return false;
        }
    } else {
        txBuff[0] = regAddr;
        memcpy(txBuff + 1, data, length);

        result = spiWrite(m_fd, txBuff, length + 1);

        if (result < 0) {
            SPI_ERROR2("data write of %d bytes to regAddr 0x%2X failed\n", length, regAddr);
            return false;
        } else if (result < (int)length) {
            SPI_ERROR3("data write of %d bytes to regAddr 0x%2X failed, only %d written\n", length, regAddr, result);
            return false;
        }
    }
    return true;
}

bool SpiDevice::Read(uint8_t regAddr, uint8_t& data)
{
    uint8_t buf;
    if (!Read(regAddr,1,&buf))
        return false;
    data = buf;
    return true;
}
bool SpiDevice::Read(uint8_t regAddr, uint8_t length, uint8_t *data)
{
    uint8_t rxBuff[SPI_MAX_READ_LEN + 1];
    struct spi_ioc_transfer rdIOC;


    rxBuff[0] = regAddr | 0x80;
    memcpy(rxBuff + 1, data, length);
    memset(&rdIOC, 0, sizeof(rdIOC));
    rdIOC.tx_buf = (unsigned long) rxBuff;
    rdIOC.rx_buf = (unsigned long) rxBuff;
    rdIOC.len = length + 1;

    if (ioctl(m_fd, SPI_IOC_MESSAGE(1), &rdIOC) < 0) {
        SPI_ERROR1("read error from regAddr 0x%2X\n", regAddr);
        return false;
    }
    memcpy(data, rxBuff + 1, length);
    return true;
}
