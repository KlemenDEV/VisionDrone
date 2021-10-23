#ifndef BMP280_h
#define BMP280_h

class BMP280
{
public:
    BMP280(struct I2cDevice *dev_); // base type

    char begin();
    // call pressure.begin() to initialize BMP280 before use
    // returns 1 if success, 0 if failure (i2C connection problem.)

    short getOversampling(void);
    char  setOversampling(short oss);

    char startMeasurment(void);
    // command BMP280 to start a pressure measurement
    // oversampling: 0 - 3 for oversampling value
    // returns (number of ms to wait) for success, 0 for fail

    char calcTemperature(double &T, double &uT);
    // calculation the true temperature from the given uncalibrated Temperature

    char calcPressure(double &P, double uP);
    //calculation for measuring pressure.

    char getTemperatureAndPressure(double& T,double& P);

private:

    char readInt(char address, int &value);
    // read an signed int (16 bits) from a BMP280 register
    // address: BMP280 register address
    // value: external signed int for returned value (16 bits)
    // returns 1 for success, 0 for fail, with result in value

    char readUInt(char address, unsigned int &value);
    // read an unsigned int (16 bits) from a BMP280 register
    // address: BMP280 register address
    // value: external unsigned int for returned value (16 bits)
    // returns 1 for success, 0 for fail, with result in value

    char readBytes(unsigned char *values, char length);
    // read a number of bytes from a BMP280 register
    // values: array of char with register address in first location [0]
    // length: number of bytes to read back
    // returns 1 for success, 0 for fail, with read bytes in values[] array

    char writeBytes(unsigned char *values, char length);
    // write a number of bytes to a BMP280 register (and consecutive subsequent registers)
    // values: array of char with register address in first location [0]
    // length: number of bytes to write
    // returns 1 for success, 0 for fail

    char getUnPT(double &uP, double &uT);
    //get uncalibrated UP and UT value.


    int dig_T2 , dig_T3 , dig_T4 , dig_P2 , dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    unsigned int dig_P1,dig_T1 ;
    short oversampling, oversampling_t;
    long signed int t_fine;

    int error;

    struct I2cDevice *dev;
};

#define BMP280_ADDR 0x76 // 7-bit address

#define	BMP280_REG_CONTROL 0xF4
#define	BMP280_REG_RESULT_PRESSURE 0xF7			// 0xF7(msb) , 0xF8(lsb) , 0xF9(xlsb) : stores the pressure data.
#define BMP280_REG_RESULT_TEMPRERATURE 0xFA		// 0xFA(msb) , 0xFB(lsb) , 0xFC(xlsb) : stores the temperature data.

#define	BMP280_COMMAND_TEMPERATURE 0x2E
#define	BMP280_COMMAND_PRESSURE0 0x25
#define	BMP280_COMMAND_PRESSURE1 0x29
#define	BMP280_COMMAND_PRESSURE2 0x2D
#define	BMP280_COMMAND_PRESSURE3 0x31
#define	BMP280_COMMAND_PRESSURE4 0x5D

#endif