#ifndef _SpiDevice_H_
#define _SpiDevice_H_

#include <stdint.h>

#define SPI_MAX_WRITE_LEN                   255
#define SPI_MAX_READ_LEN                    255


class SpiDevice {
private:
	int m_fd;
public:
    SpiDevice();
    virtual ~SpiDevice();

	bool Open(unsigned char spiBus = 0,unsigned char spiSelect = 0);
	void Close();

	bool Write(uint8_t regAddr, uint8_t const data);
	bool Write(uint8_t regAddr, uint8_t length, uint8_t const *data);
	bool Read(uint8_t regAddr, uint8_t length, uint8_t *data);
	bool Read(uint8_t regAddr, uint8_t& data);


};

#endif
