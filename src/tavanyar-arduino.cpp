/*
 * Copyright 2019 Melika Barzegaran <melika.barzegaran.hosseini@gmail.com>
 * Copyright 2011-2013 Jeff Rowberg <jeff@rowberg.net>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
 * to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of
 * the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 * THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

/*
 * title:
 *      Getting raw data from multiple MPU-6050s with Arduino
 *
 * description:
 *      Returns raw data (accelerometer output towards x, y, and z axes and gyroscope output towards x, y, and z
 *      axes) from multiple MPU-6050s.
 *
 *      The sketch uses MPU-6050 library, written by Jeff Rowberg. That MPU-6050 library, uses the I2C serial
 *      communication protocol library, which is also written by Jeff Rowberg. Finally, the I2C serial communication
 *      protocol library uses the official Arduino I2C library, called Wire.
 *
 *      The link to the source code of MPU-6050 library, written by Jeff Rowberg, is here:
 *          https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 *
 *      The link to the source code of I2C serial protocol library, also written by Jeff Rowberg, is here:
 *          https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
 *
 *      The reference to the official Arduino I2C library, called Wire, can be found here:
 *          https://www.arduino.cc/en/reference/wire
 *
 * configuration:
 *      +-----------------------+-------------------------------+
 *      | Arduino Uno board pin | GY-521 break-out board #1 pin |
 *      +-----------------------+-------------------------------+
 *      |          VCC          |              VCC              |
 *      +-----------------------+-------------------------------+
 *      |          GND          |              GND              |
 *      +-----------------------+-------------------------------+
 *      |           A5          |              SCL              |
 *      +-----------------------+-------------------------------+
 *      |           A4          |              SDA              |
 *      +-----------------------+-------------------------------+
 *      |          AD0          |              D13              |
 *      +-----------------------+-------------------------------+
 *
 *      +-----------------------+-------------------------------+
 *      | Arduino Uno board pin | GY-521 break-out board #2 pin |
 *      +-----------------------+-------------------------------+
 *      |          VCC          |              VCC              |
 *      +-----------------------+-------------------------------+
 *      |          GND          |              GND              |
 *      +-----------------------+-------------------------------+
 *      |           A5          |              SCL              |
 *      +-----------------------+-------------------------------+
 *      |           A4          |              SDA              |
 *      +-----------------------+-------------------------------+
 *      |          AD0          |              D12              |
 *      +-----------------------+-------------------------------+
 *
 * author:
 *      Melika Barzegaran <melika.barzegaran.hosseini@gmail.com>
 *
 * version:
 *      1.0.0
 */

/*
 * Add MPU-6050 library, written by Jeff Rowberg <jeff@rowberg.net>.
 * Link to the library: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 */
#include "MPU6050.h"

/*
 * Uncomment to print test output data (ASCII format)
 */
#define OUTPUT_TEST

/*
 * Uncomment to print production output data (binary format)
 * TODO: This functionality must be tested and validated.
 */
/* define OUTPUT_PRODUCTION */

/*
 * Define MPU-6050 object under communication.
 * Input parameter is the I2C serial communication protocol address for the device.
 *
 * When communicating with a MPU-6050 module:
 *      1. Set address pin of that module to low.
 *      2. Set address pins of other modules to high.
 *
 * +-------------------+---------------------------------+
 * | address pin (AD0) |        I2C device address       |
 * +-------------------+---------------------------------+
 * |  connected to GND |  0x68 (MPU6050_ADDRESS_AD0_LOW) |
 * +-------------------+---------------------------------+
 * |  connected to VCC | 0x69 (MPU6050_ADDRESS_ADO_HIGH) |
 * +-------------------+---------------------------------+
 */
MPU6050 motionTrackingDevice(MPU6050_ADDRESS_AD0_LOW);

/*
 * Number of motion tracking devices in the system.
 * TODO: Update this whenever adding/removing a device to/from the system.
 */
const int NUMBER_OF_DEVICES = 2;

/*
 * The array of address pins of motion tracking devices in the system.
 * TODO: Update this whenever adding/removing a device to/from the system.
 */
const uint8_t addressPinArray[NUMBER_OF_DEVICES] = {13, 12};

/*
 * The struct for data read from built-in sensors of a motion tracking device in a moment.
 */
struct Data {
    int16_t accelerometerX;
    int16_t accelerometerY;
    int16_t accelerometerZ;
    int16_t gyroscopeX;
    int16_t gyroscopeY;
    int16_t gyroscopeZ;
};

/*
 * The array of data read from built-in sensors of motion tracking devices in the system.
 */
Data dataArray[NUMBER_OF_DEVICES] = {0};

/*
 * The struct for calibration offsets for a motion tracking device.
 */
struct CalibrationData {
    int16_t accelerometerX;
    int16_t accelerometerY;
    int16_t accelerometerZ;
    int16_t gyroscopeX;
    int16_t gyroscopeY;
    int16_t gyroscopeZ;
};

/*
 * The array of calibration offsets for motion tracking devices in the system.
 * TODO: Update this whenever adding/removing a device to/from the system.
 */
CalibrationData calibrationDataArray[NUMBER_OF_DEVICES] = {
        {-3300, 1110, 1163, 75, -18, 122},
        {-1933, -50,  1245, 55, 55,  36}
};

void setupUartSerialCommunicationProtocol();

void setupI2cSerialCommunicationProtocol();

void setupAddressPins();

void setupMotionTrackingDevices();

void selectMotionTrackingDevice(int selectedIndex);

void setClockSource();

void setRangeAndSensitivity();

void setLowPassFilter();

void setSampleRate();

void setCalibrationOffsets(int selectedIndex);

void wakeUpMotionTrackingDevices();

void wakeUp();

void collectData();

void sendData();

void setup() {
    setupUartSerialCommunicationProtocol();
    setupI2cSerialCommunicationProtocol();
    setupAddressPins();
    setupMotionTrackingDevices();
    wakeUpMotionTrackingDevices();
}

void setupUartSerialCommunicationProtocol() {
    /*
     * Setup UART serial communication protocol.
     *
     * General UART serial communication protocol frame format is as below:
     * +-------+------+--------+------+
     * | start | data | parity | stop |
     * +-------+------+--------+------+
     * |   1   |  5-8 |   0-1  |  1-2 |
     * +-------+------+--------+------+
     *
     * 8N1 frame format, which is the default frame format, is used. It has:
     *
     *      - 1 bit for start,
     *      - 8 bits for data,
     *      - no parity,
     *      - and 1 bit for stop.
     *
     * As a result, frames used for UART serial communication protocol are overall 10 bits and as below:
     * +-------+------+------+
     * | start | data | stop |
     * +-------+------+------+
     * |   1   |   8  |   1  |
     * +-------+------+------+
     *
     * Set baud rate to 38400 bps. 38400 bps is chosen because:
     *
     *      - If we use 10 MPU-6050 modules,
     *
     *      - And from each module, we read 14 x 8-bit registers (6 x 8-bit registers for accelerometer, 6 x 8-bit
     *      register for gyroscope, and 2 x 8-bit registers for temperature),
     *
     *      - And we transfer those data in our 10-bit UART serial communication protocol frames,
     *
     *      - With 25 Hz as sample frequency,
     *
     * Then we would need to send 10 x 14 x 10 x 25 = 35000 bits per second. It means that the lowest baud rate we can
     * use is 35000 bps, and the lowest standard baud rate we can use is 38400 bps.
     */
    Serial.begin(38400);
}

void setupI2cSerialCommunicationProtocol() {
    /*
     * Setup I2C serial communication protocol.
     *
     * Set clock Frequency of I2C serial communication protocol to the default value, which is the standard mode
     * (100 KHz). To set the clock frequency to the fast mode (400 KHz), use the code below:
     *
     * ```
     * Wire.setClock(400000);
     * ```
     */
    Wire.begin();
}

void setupAddressPins() {
    for (uint8_t addressPin : addressPinArray) {
        pinMode(addressPin, OUTPUT);
    }
}

void setupMotionTrackingDevices() {
    for (int index = 0; index < NUMBER_OF_DEVICES; index++) {
        selectMotionTrackingDevice(index);
        setClockSource();
        setRangeAndSensitivity();
        setLowPassFilter();
        setSampleRate();
        setCalibrationOffsets(index);
    }
}

void selectMotionTrackingDevice(int selectedIndex) {
    for (int index = 0; index < NUMBER_OF_DEVICES; index++) {
        if (index == selectedIndex) {
            digitalWrite(addressPinArray[index], LOW);
        } else {
            digitalWrite(addressPinArray[index], HIGH);
        }
    }
}

void setClockSource() {
    /*
     * register name = PWR_MGMT_1 (power management 1)
     *
     * +-----------------------+-----------------------+------+------+------+------+------+------+------+------+
     * | register number (HEX) | register number (DEC) | bit7 | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0 |
     * +-----------------------+-----------------------+------+------+------+------+------+------+------+------+
     * |          0x6B         |          107          |   -  |   -  |   -  |   -  |   -  |     CLKSEL[2:0]    |
     * +-----------------------+-----------------------+------+------+------+------+------+--------------------+
     *
     * +--------+---------------------------------------------------------+
     * | CLKSEL |                       clock source                      |
     * +--------+---------------------------------------------------------+
     * |    0   |                internal 8 MHz oscillator                |
     * +--------+---------------------------------------------------------+
     * |    1   |           PLL with x axis gyroscope reference           | <- selected
     * +--------+---------------------------------------------------------+
     * |    2   |           PLL with y axis gyroscope reference           |
     * +--------+---------------------------------------------------------+
     * |    3   |           PLL with z axis gyroscope reference           |
     * +--------+---------------------------------------------------------+
     * |    4   |          PLL with external 32.768 KHz reference         |
     * +--------+---------------------------------------------------------+
     * |    5   |           PLL with external 19.2 MHz reference          |
     * +--------+---------------------------------------------------------+
     * |    6   |                         reserved                        |
     * +--------+---------------------------------------------------------+
     * |    7   | stops the clock and keeps the timing generator in reset |
     * +--------+---------------------------------------------------------+
     *
     * Set clock source to use x axis of gyroscope as clock reference.
     * Selecting one of the axes of gyroscope as clock reference provides us with a more accurate clock source.
     */
    motionTrackingDevice.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
}

void setRangeAndSensitivity() {
    /*
     * register name = ACCEL_CONFIG (accelerometer configuration)
     *
     * +-----------------------+-----------------------+------+------+------+-------+------+------+------+------+
     * | register number (HEX) | register number (DEC) | bit7 | bit6 | bit5 |  bit4 | bit3 | bit2 | bit1 | bit0 |
     * +-----------------------+-----------------------+------+------+------+-------+------+------+------+------+
     * |          0x1C         |           28          |   -  |   -  |   -  | AFS_SEL[1:0] |   -  |   -  |   -  |
     * +-----------------------+-----------------------+------+------+------+--------------+------+------+------+
     *
     * +---------+------------------+-------------+
     * | AFS_SEL | full-scale range | sensitivity |
     * +---------+------------------+-------------+
     * |    0    |      +/-2 g      | 16384 LSB/g |
     * +---------+------------------+-------------+
     * |    1    |      +/-4 g      |  8192 LSB/g |
     * +---------+------------------+-------------+
     * |    2    |      +/-8 g      |  4096 LSB/g | <-- selected
     * +---------+------------------+-------------+
     * |    3    |      +/-16 g     |  2048 LSB/g |
     * +---------+------------------+-------------+
     *
     * Set full-scale range for accelerometer to +/-8 g.
     * Set sensitivity for accelerometer to 4096 LSB/g.
     * Reading 4096 from accelerometer over one of its axes means the acceleration of +1 g towards that axis.
     */
    motionTrackingDevice.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);

    /*
     * register name = GYRO_CONFIG (gyroscope configuration)
     *
     * +-----------------------+-----------------------+------+------+------+------+------+------+------+------+
     * | register number (HEX) | register number (DEC) | bit7 | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0 |
     * +-----------------------+-----------------------+------+------+------+------+------+------+------+------+
     * |          0x1B         |           27          |   -  |   -  |   -  | FS_SEL[1:0] |   -  |   -  |   -  |
     * +-----------------------+-----------------------+------+------+------+-------------+------+------+------+
     *
     * +--------+------------------+--------------+
     * | FS_SEL | full-scale range |  sensitivity |
     * +--------+------------------+--------------+
     * |    0   |    +/-250 dps    |  131 LSB/dps |
     * +--------+------------------+--------------+
     * |    1   |    +/-500 dps    | 65.5 LSB/dps |
     * +--------+------------------+--------------+
     * |    2   |    +/-1000 dps   | 32.8 LSB/dps | <- selected
     * +--------+------------------+--------------+
     * |    3   |    +/-2000 dps   | 16.4 LSB/dps |
     * +--------+------------------+--------------+
     *
     * Set full-scale range for gyroscope to +/-1000 dps.
     * Set sensitivity for gyroscope to 32.8 LSB/dps.
     * Reading 131 from gyroscope over one of its axes means the angular speed of +1 dps around that axis.
     */
    motionTrackingDevice.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
}

void setLowPassFilter() {
    /*
     * register name = CONFIG (configuration)
     *
     * +-----------------------+-----------------------+------+------+------+------+------+------+------+------+
     * | register number (HEX) | register number (DEC) | bit7 | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0 |
     * +-----------------------+-----------------------+------+------+------+------+------+------+------+------+
     * |          0x1A         |           26          |   -  |   -  |   -  |   -  |   -  |    DLPF_CFG[2:0]   |
     * +-----------------------+-----------------------+------+------+------+------+------+--------------------+
     *
     * +----------+-----------------------------------+-----------------------------------+
     * | DLPF_CFG |           accelerometer           |             gyroscope             |
     * +          +-----------------------------------+-----------------------------------+
     * |          | bandwidth |  delay  | output rate | bandwidth |  delay  | output rate |
     * +----------+-----------+---------+-------------+-----------+---------+-------------+
     * |     0    |   260 Hz  |   0 ms  |    1 KHz    |   256 Hz  | 0.98 ms |    8 KHz    |
     * +----------+-----------+---------+-------------+-----------+---------+-------------+
     * |     1    |   184 Hz  |   2 ms  |    1 KHz    |   188 Hz  |  1.9 ms |    1 KHz    |
     * +----------+-----------+---------+-------------+-----------+---------+-------------+
     * |     2    |   94 Hz   |   3 ms  |    1 KHz    |   98 Hz   |  2.8 ms |    1 KHz    |
     * +----------+-----------+---------+-------------+-----------+---------+-------------+
     * |     3    |   44 Hz   |  4.9 ms |    1 KHz    |   42 Hz   |  4.8 ms |    1 KHz    |
     * +----------+-----------+---------+-------------+-----------+---------+-------------+
     * |     4    |   21 Hz   |  8.5 ms |    1 KHz    |   20 Hz   |  8.3 ms |    1 KHz    |
     * +----------+-----------+---------+-------------+-----------+---------+-------------+
     * |     5    |   10 Hz   | 13.8 ms |    1 KHz    |   10 Hz   | 13.4 ms |    1 KHz    |
     * +----------+-----------+---------+-------------+-----------+---------+-------------+
     * |     6    |    5 Hz   |  19 ms  |    1 KHz    |    5 Hz   | 18.6 ms |    1 KHz    | <- selected
     * +----------+-----------+---------+-------------+-----------+---------+-------------+
     * |     7    |       reserved      |    1 KHz    |       reserved      |    8 KHz    |
     * +----------+---------------------+-------------+---------------------+-------------+
     *
     * Enable low pass filter.
     * Set accelerometer low pass filter frequency to 5 Hz with delay of 19 ms.
     * Set accelerometer output rate to 1 KHz.
     * Set gyroscope low pass filter frequency to 5 Hz with delay of 18.6 ms.
     * Set gyroscope output rate to 1 KHz.
     */
    motionTrackingDevice.setDLPFMode(MPU6050_DLPF_BW_5);
}

void setSampleRate() {
    /*
     * register name = SMPRT_DIV (sample rate divider)
     *
     * +-----------------------+-----------------------+------+------+------+------+------+------+------+------+
     * | register number (HEX) | register number (DEC) | bit7 | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0 |
     * +-----------------------+-----------------------+------+------+------+------+------+------+------+------+
     * |          0x19         |           25          |                     SMPRT_DIV[7:0]                    |
     * +-----------------------+-----------------------+-------------------------------------------------------+
     *
     * sample rate = accelerometer output rate or gyroscope output rate / (1 + SMPRT_DIV)
     * -> 25 Hz = 1 KHz / (1 + SMPRT_DIV)
     * -> SMPRT_DIV = (1000 / 25) - 1 = 40 - 1 = 39
     *
     * Because accelerometer output rate and gyroscope output rate are both equal to 1 KHz, set sample rate
     * divider to 39, so that sample rate be set to 25 Hz.
     *
     */
    motionTrackingDevice.setRate(39);
}

void setCalibrationOffsets(int selectedIndex) {
    CalibrationData calibrationData = calibrationDataArray[selectedIndex];
    motionTrackingDevice.setXAccelOffset(calibrationData.accelerometerX);
    motionTrackingDevice.setYAccelOffset(calibrationData.accelerometerY);
    motionTrackingDevice.setZAccelOffset(calibrationData.accelerometerZ);
    motionTrackingDevice.setXGyroOffset(calibrationData.gyroscopeX);
    motionTrackingDevice.setYGyroOffset(calibrationData.gyroscopeY);
    motionTrackingDevice.setZGyroOffset(calibrationData.gyroscopeZ);
}

void wakeUpMotionTrackingDevices() {
    for (int index = 0; index < NUMBER_OF_DEVICES; index++) {
        wakeUp();
    }
}

void wakeUp() {
    /*
     * register name = PWR_MGMT_1 (power management 1)
     *
     * +-----------------------+-----------------------+------+-------+------+------+------+------+------+------+
     * | register number (HEX) | register number (DEC) | bit7 |  bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0 |
     * +-----------------------+-----------------------+------+-------+------+------+------+------+------+------+
     * |          0x6B         |          107          |   -  | SLEEP |   -  |   -  |   -  |   -  |   -  |   -  |
     * +-----------------------+-----------------------+------+-------+------+------+------+------+------+------+
     *
     * Wake up the device from sleep mode.
     * Also consider that all the sensors (accelerometer, gyroscope, and temperature) are enabled and consuming around
     * 3.8 (mA) current.
     */
    motionTrackingDevice.setSleepEnabled(false);
}

void loop() {
    collectData();
    sendData();

    /*
     * Because sample rate is 25 Hz and there is no need to read sensor values more than 25 times in a second.
     *
     * 1000 ms / 25 = 40 ms
     */
    delay(40);
}

void collectData() {
    for (int index = 0; index < NUMBER_OF_DEVICES; index++) {
        selectMotionTrackingDevice(index);
        motionTrackingDevice.getMotion6(
                &dataArray[index].accelerometerX,
                &dataArray[index].accelerometerY,
                &dataArray[index].accelerometerZ,
                &dataArray[index].gyroscopeX,
                &dataArray[index].gyroscopeY,
                &dataArray[index].gyroscopeZ);
    }
}

void sendData() {
#ifdef OUTPUT_TEST
    for (Data data : dataArray) {
        Serial.print(data.accelerometerX);
        Serial.print("\t");
        Serial.print(data.accelerometerY);
        Serial.print("\t");
        Serial.print(data.accelerometerZ);
        Serial.print("\t");
        Serial.print(data.gyroscopeX);
        Serial.print("\t");
        Serial.print(data.gyroscopeY);
        Serial.print("\t");
        Serial.print(data.gyroscopeZ);
        Serial.print("\t");
    }
    Serial.println();
#endif

#ifdef OUTPUT_PRODUCTION
    for (Data data : dataArray) {
        Serial.write((uint8_t)(data.accelerometerX >> 8));
        Serial.write((uint8_t)(data.accelerometerX & 0xFF));
        Serial.write((uint8_t)(data.accelerometerY >> 8));
        Serial.write((uint8_t)(data.accelerometerY & 0xFF));
        Serial.write((uint8_t)(data.accelerometerZ >> 8));
        Serial.write((uint8_t)(data.accelerometerZ & 0xFF));
        Serial.write((uint8_t)(data.gyroscopeX >> 8));
        Serial.write((uint8_t)(data.gyroscopeX & 0xFF));
        Serial.write((uint8_t)(data.gyroscopeY >> 8));
        Serial.write((uint8_t)(data.gyroscopeY & 0xFF));
        Serial.write((uint8_t)(data.gyroscopeZ >> 8));
        Serial.write((uint8_t)(data.gyroscopeZ & 0xFF));
    }
#endif
}