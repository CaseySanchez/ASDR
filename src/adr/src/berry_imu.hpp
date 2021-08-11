#pragma once

#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <string>
#include <stdexcept>

#include "i2c-dev.h"
#include "LSM6DSL.h"
#include "LIS3MDL.h"

enum {
	FS_G_250 = 0,
	FS_G_500,
	FS_G_1000,
	FS_G_2000
};

enum {
	FS_XL_2 = 0,
	FS_XL_4,
	FS_XL_8,
	FS_XL_16
};

enum {
	FS_M_4 = 0,
	FS_M_8,
	FS_M_12,
	FS_M_16
};

int32_t const FS_G_bits[4] = {
	0b00, // FS_G_250
	0b01, // FS_G_500
	0b10, // FS_G_1000
	0b11  // FS_G_2000
};

double const FS_G_sensitivity[4] = {
	8.75, // FS_G_250
	17.5, // FS_G_500
	35.0, // FS_G_1000
	70.0  // FS_G_2000
};

int32_t const FS_XL_bits[4] = {
	0b00, // FS_XL_2
	0b10, // FS_XL_4
	0b11, // FS_XL_8
	0b01  // FS_XL_16
};

double const FS_XL_sensitivity[4] = {
	0.061, // FS_XL_2
	0.122, // FS_XL_4
	0.244, // FS_XL_8
	0.488  // FS_XL_16
};

int32_t const FS_M_bits[4] = {
	0b00, // FS_M_4
	0b01, // FS_M_8
	0b10, // FS_M_12
	0b11  // FS_M_16
};

double const FS_M_sensitivity[4] = {
	6842.0, // FS_M_4
	3421.0, // FS_M_8
	2281.0, // FS_M_12
	1711.0  // FS_M_16
};

class BerryIMU
{
	int32_t m_file;
	int32_t m_FS_G;
	int32_t m_FS_XL;
	int32_t m_FS_M;

public:
	BerryIMU(int32_t FS_G, int32_t FS_XL, int32_t FS_M);

	// Convert to SI units [mdeg/sec]->[rad/sec]
	void readGyr(double gyr[3]);
	// Convert to SI units [mG]->[m/s^2]
	void readAcc(double acc[3]);
	// Convert to SI units [Gauss]->[Tesla]
	void readMag(double mag[3]);

private:
	void selectDevice(int32_t file, int32_t addr);
	void readBytes(int32_t addr, uint8_t reg, uint8_t size, uint8_t *data);
	void readByte(int32_t addr, uint8_t reg, uint8_t *data);
	void writeByte(int32_t addr, uint8_t reg, uint8_t value);
};
