#include "berry_imu.hpp"

BerryIMU::BerryIMU(int32_t FS_G, int32_t FS_XL, int32_t FS_M) : m_FS_G(FS_G), m_FS_XL(FS_XL), m_FS_M(FS_M)
{
    m_file = open("/dev/i2c-1", O_RDWR);

    if (m_file < 0) {
        throw std::runtime_error(std::string("Unable to open I2C bus with error: ") + strerror(errno));
    }

    //Detect if BerryIMUv3 (Which uses a LSM6DSL and LIS3MDL) is connected

    uint8_t LSM6DSL_WHO_M_response;
    uint8_t LIS3MDL_WHO_XG_response;

    readByte(LSM6DSL_ADDRESS, LSM6DSL_WHO_AM_I, &LSM6DSL_WHO_M_response);
    readByte(LIS3MDL_ADDRESS, LIS3MDL_WHO_AM_I, &LIS3MDL_WHO_XG_response);

    if (LSM6DSL_WHO_M_response != 0x6A || LIS3MDL_WHO_XG_response != 0x3D){
        throw std::runtime_error("LSM6DSL or LIS3MDL not detected");
    }
    
    //Enable  gyroscope
    writeByte(LSM6DSL_ADDRESS, LSM6DSL_CTRL2_G, 0b10010000 | (FS_G_bits[m_FS_G] << 2));        // ODR 3.3 kHz, 2000 dps

    // Enable the accelerometer
    writeByte(LSM6DSL_ADDRESS, LSM6DSL_CTRL1_XL, 0b10010011 | (FS_XL_bits[m_FS_XL] << 2));       // ODR 3.33 kHz, +/- 8g , BW = 400hz
    writeByte(LSM6DSL_ADDRESS, LSM6DSL_CTRL8_XL, 0b11001000);       // Low pass filter enabled, BW9, composite filter
    writeByte(LSM6DSL_ADDRESS, LSM6DSL_CTRL3_C, 0b01000100);        // Enable Block Data update, increment during multi byte read

    //Enable  magnetometer
    writeByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG1, 0b11011100);     // Temp sesnor enabled, High performance, ODR 80 Hz, FAST ODR disabled and Selft test disabled.
    writeByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG2, 0b00000000 | (FS_M_bits[m_FS_M] << 5));     // +/- 8 gauss
    writeByte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG3, 0b00000000);     // Continuous-conversion mode
}

// Convert to SI units [mdeg/sec]->[rad/sec]
void BerryIMU::readGyr(double gyr[3])
{
    double constexpr conversion { 3.14159265358979323846 / 180.0 / 1000.0 }; 
    
    int16_t raw[3];
    
    readBytes(LSM6DSL_ADDRESS, LSM6DSL_OUT_X_L_G, 6, reinterpret_cast<uint8_t *>(&raw[0]));
    
    gyr[0] = static_cast<double>(raw[0]) * FS_G_sensitivity[m_FS_G] * conversion;
    gyr[1] = static_cast<double>(raw[1]) * FS_G_sensitivity[m_FS_G] * conversion;
    gyr[2] = static_cast<double>(raw[2]) * FS_G_sensitivity[m_FS_G] * conversion;
}

// Convert to SI units [mG]->[m/s^2]
void BerryIMU::readAcc(double acc[3])
{
    double constexpr conversion { 9.8 / 1000.0 };
    
    int16_t raw[3];
    
    readBytes(LSM6DSL_ADDRESS, LSM6DSL_OUT_X_L_XL, 6, reinterpret_cast<uint8_t *>(&raw[0]));
    
    acc[0] = static_cast<double>(raw[0]) * FS_XL_sensitivity[m_FS_XL] * conversion;
    acc[1] = static_cast<double>(raw[1]) * FS_XL_sensitivity[m_FS_XL] * conversion;
    acc[2] = static_cast<double>(raw[2]) * FS_XL_sensitivity[m_FS_XL] * conversion;
}

// Convert to SI units [Gauss]->[Tesla]
void BerryIMU::readMag(double mag[3])
{
    double constexpr conversion { 1.0 / 10000.0 };
    
    int16_t raw[3];
    
    readBytes(LIS3MDL_ADDRESS, LIS3MDL_OUT_X_L, 6, reinterpret_cast<uint8_t *>(&raw[0]));
    
    mag[0] = static_cast<double>(raw[0]) * FS_M_sensitivity[m_FS_M] * conversion;
    mag[1] = static_cast<double>(raw[1]) * FS_M_sensitivity[m_FS_M] * conversion;
    mag[2] = static_cast<double>(raw[2]) * FS_M_sensitivity[m_FS_M] * conversion;
}

void BerryIMU::selectDevice(int32_t file, int32_t addr)
{
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        throw std::runtime_error(std::string("Failed to select I2C device with error: ") + strerror(errno));
    }
}

void BerryIMU::readBytes(int32_t addr, uint8_t reg, uint8_t size, uint8_t *data)
{
    selectDevice(m_file, addr);

    int32_t result = i2c_smbus_read_i2c_block_data(m_file, reg, size, data);

    if (result != size) {
        throw std::runtime_error(std::string("Failed to read I2C bytes with error: ") + strerror(errno));
    } 
}

void BerryIMU::readByte(int32_t addr, uint8_t reg, uint8_t *data)
{
    selectDevice(m_file, addr);
    
    int32_t result = i2c_smbus_read_byte_data(m_file, reg);
    
    if (result == -1) {
        throw std::runtime_error(std::string("Failed to read I2C byte with error: ") + strerror(errno));
    }

    *data = result;
}

void BerryIMU::writeByte(int32_t addr, uint8_t reg, uint8_t value)
{
    selectDevice(m_file, addr);

    int32_t result = i2c_smbus_write_byte_data(m_file, reg, value);

    if (result == -1) {
        throw std::runtime_error(std::string("Failed to write I2C byte with error: ") + strerror(errno));
    }
}

