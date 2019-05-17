/*
 * MIT License
 *
 * Copyright 2019 Melika Barzegaran <melika.barzegaran.hosseini@gmail.com>
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
 *      Tavanyar (Arduino)
 *
 * description:
 *      Tavanyar is a system which detects and evaluates physical therapy exercises automatically using motion
 *      tracking devices. This project consists of two parts: a piece of hardware and some kind of user interface.
 *      This code is the hardware-side of the project.
 *
 *      Two-way communication between hardware and user interface is made possible via bluetooth device:
 *
 *          1. Hardware -> User interface:
 *          Hardware can send 25 samples per second to the user interface. Each sample consists of the acceleration
 *          towards x, y, and z axes of each device obtained by accelerometers, and the angular velocity around
 *          x, y, and z axes of each device obtained by gyroscopes.
 *
 *          2. User interface -> Hardware:
 *          User interface can send commands to the hardware and configure the system. It can pause and resume
 *          sending data, and also specify the number of motion tracking devices being used.
 *
 *      The hardware consists of:
 *          1. One Arduino Uno as micro-controller
 *          2. Multiple (currently 2) MPU-6050 modules as motion tracking devices
 *          3. One HC-05 module as bluetooth device
 *
 *      Arduino Uno specifications:
 *          - Clock frequency: the standard mode (100 KHz)
 *          - Uses software serial protocol for debugging
 *          - UART serial protocol standard: 8N1
 *          - UART serial protocol baud rate: 38400 bps
 *
 *      MPU-6050 specifications:
 *          - Accelerometer:
 *              - Full-scale range: +/-8 g
 *              - Sensitivity: 4096 LSB/g
 *              - Low pass filter:
 *                  - Frequency: 5 Hz
 *                  - Delay: 19 ms
 *              - Output rate: 1 KHz
 *          - Gyroscope:
 *              - Full-scale range: +/-1000 dps
 *              - Sensitivity: 32.8 LSB/dps
 *              - Low pass filter:
 *                  - Frequency: 5 Hz
 *                  - Delay: 18.6 ms
 *              - Output rate: 1 KHz
 *          - Sampling:
 *              - Frequency: 25 Hz
 *              - Divider: 39
 *          - Clock reference: x axis of gyroscope
 *          - Current consumption: 3.8 mA
 *          - Uses digital input/output pins D13-D4 as address pins
 *          - Uses 0x68 I2C address for the module under communication and 0x69 I2C address for other modules
 *
 *      HC-05 specifications:
 *          - Name: TAVANYAR
 *          - Password: 123456
 *          - Uses software serial protocol for communication
 *          - UART serial protocol standard: 8N1
 *          - UART serial protocol baud rate: 38400 bps
 *          - role: slave
 *
 *      Note 1.
 *      It is assumed that MPU-6050 modules are calibrated, and the corresponding calibration offsets are fed into the
 *      system as well, all before running the sketch.
 *
 *      See this link in order to calibrate MPU-6050 modules and obtain calibration offsets:
 *      https://github.com/melikabarzegaran/mpu6050-raw-calibration
 *
 *      Note 2.
 *      It is assumed that HC-05 module is configured before running the sketch.
 *
 *      See this link in order to configure HC-05 module:
 *      https://github.com/melikabarzegaran/hc05-configuration
 *
 *      Note 3.
 *      We use 2 MPU-6050 modules with ASCII output format in the system, for the sake of simplicity and easy
 *      debugging. For such a system, according to the table below, 38400 bps would be enough as serial baud rate.
 *      Consider raising serial baud rate or shifting to binary output format when using more than 3 MPU-6050 modules.
 *
 *      +----------------------+----------+-----------+-----------+-----------+-----------+------------+
 *      |                      | 9600 bps | 19200 bps | 38400 bps | 57600 bps | 74880 bps | 115200 bps |
 *      +----------------------+----------+-----------+-----------+-----------+-----------+------------+
 *      | binary output format |     1    |     3     |     6     |     9     |     12    |     19     |
 *      +----------------------+----------+-----------+-----------+-----------+-----------+------------+
 *      |  ASCII output format |     -    |     1     |     3     |     4     |     6     |      9     |
 *      +----------------------+----------+-----------+-----------+-----------+-----------+------------+
 *
 * configuration:
 *      +-----------------------+--------------------------------------------------+
 *      | Arduino Uno board pin | GY-521 break-out board #n pin (n=1,2,...,10) (*) |
 *      +-----------------------+--------------------------------------------------+
 *      |          VCC          |                        VCC                       |
 *      +-----------------------+--------------------------------------------------+
 *      |          GND          |                        GND                       |
 *      +-----------------------+--------------------------------------------------+
 *      |           A5          |                        SCL                       |
 *      +-----------------------+--------------------------------------------------+
 *      |           A4          |                        SDA                       |
 *      +-----------------------+--------------------------------------------------+
 *      |          AD0          |                     D(13-n+1)                    |
 *      +-----------------------+--------------------------------------------------+
 *      (*) In order to support more than 3 MPU-6050 modules, we should either raise the baud rate or shift to use
 *      binary output format instead of ASCII output format. `10` is just the number of free digital input/output
 *      pins of Arduino Uno, which are ready to connect to MPU6050 modules as address pins.
 *
 *      +-----------------------+----------------------------+
 *      | Arduino Uno board pin | ZS-040 break-out board pin |
 *      +-----------------------+----------------------------+
 *      |          VCC          |             VCC            |
 *      +-----------------------+----------------------------+
 *      |          GND          |             GND            |
 *      +-----------------------+----------------------------+
 *      |           D2          |           TXD (*)          |
 *      +-----------------------+----------------------------+
 *      |           D3          |          RXD (**)          |
 *      +-----------------------+----------------------------+
 *      (*) This connection is made directly.
 *      (**) This connection is made via voltage divider.
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
 *
 * MPU-6050 library uses I2C serial protocol library, also written by Jeff Rowberg.
 * Link to the library: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
 *
 * I2C serial protocol library uses the official Arduino I2C serial protocol library.
 * Link to the library: https://www.arduino.cc/en/reference/wire
 *
 * There is no need to add the lines below. They are added automatically by adding MPU-6050 library.
 * ```
 *      #include "I2Cdev.h"
 *      #include "Wire.h"
 *      #include "Arduino.h"
 * ```
 *
 * This library is used for the communication between Arduino Uno and MPU-6050 modules.
 */
#include "MPU6050.h"

/*
 * Add the official Arduino software UART serial library.
 * Link to the library: https://www.arduino.cc/en/Reference/SoftwareSerial
 *
 * This library is used for the communication between Arduino Uno and HC-05 module while allowing the board to be
 * programmed and debugged simultaneously.
 */
#include "SoftwareSerial.h"

/*
 * Define object for under-communication MPU-6050 module (MPU-6050 module which we are talking to).
 *
 * Used to transfer data between Arduino Uno and selected MPU-6050 module via I2C serial protocol.
 *
 * Input parameter is the address for MPU-6050 module we want to talk to in I2C serial protocol.
 *
 * When communicating with a MPU-6050 module:
 *      1. Set address pin of that module to low.
 *      2. Set address pins of other modules to high.
 *
 * +-------------------+---------------------------------+
 * | address pin (AD0) |   I2C serial protocol address   |
 * +-------------------+---------------------------------+
 * |  connected to GND |  0x68 (MPU6050_ADDRESS_AD0_LOW) |
 * +-------------------+---------------------------------+
 * |  connected to VCC | 0x69 (MPU6050_ADDRESS_AD0_HIGH) |
 * +-------------------+---------------------------------+
 */
MPU6050 selectedMotionTrackingDevice(MPU6050_ADDRESS_AD0_LOW);

/*
 * Total number of MPU-6050 modules in the system.
 * TODO: Update this when adding/removing a MPU-6050 module to/from the system.
 */
const int numberOfMotionTrackingDevices = 2;

/*
 * The array of address pins of MPU-6050 modules in the system.
 * TODO: Update this when adding/removing a MPU-6050 module to/from the system.
 */
const uint8_t addressPinArray[numberOfMotionTrackingDevices] = {13, 12};

/*
 * The struct for data read from sensors of a MPU-6050 module in a moment.
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
 * The array of data read from sensors of MPU-6050 modules in the system.
 */
Data dataArray[numberOfMotionTrackingDevices] = {0};

/*
 * The struct for calibration offsets of a MPU-6050 module.
 */
struct CalibrationData {
    const int16_t accelerometerX;
    const int16_t accelerometerY;
    const int16_t accelerometerZ;
    const int16_t gyroscopeX;
    const int16_t gyroscopeY;
    const int16_t gyroscopeZ;
};

/*
 * The array of calibration offsets of MPU-6050 modules in the system.
 * TODO: Update this when adding/removing a MPU-6050 module to/from the system.
 */
const CalibrationData calibrationDataArray[numberOfMotionTrackingDevices] = {
        {-3338, 1077, 1184, 71, -13, 121}, /* 2, 0, 16382, 0, -1, -1 */
        {-1900, -44,  1243, 46, 61,  34} /* 6, 5, 16386, 0, 0, 1 */
};

/*
 * Pin chosen as receive pin of Arduino Uno in software UART serial protocol.
 * It should be connected to the TXD pin of HC-05 module, directly.
 */
const int rxdPin = 2;

/*
 * Pin chosen as transfer pin of Arduino Uno in software UART serial protocol.
 * It should be connected to the RXD pin of HC-05 module, via voltage divider.
 */
const int txdPin = 3;

/*
 * Define object for HC-05 module.
 *
 * Used to transfer data between Arduino Uno and HC-05 module via software UART serial protocol.
 *
 * Input parameters are, respectively, receive and transfer pins of Arduino Uno in UART serial protocol.
 */
SoftwareSerial bluetoothDevice(rxdPin, txdPin);

/*
 * Receiving this command means to pause sending data over bluetooth.
 */
const int pauseCommand = 0;

/*
 * Receiving this command means to resume sending data over bluetooth.
 */
const int resumeCommand = 1;

/*
 * Receiving this command means that we should use the first MPU-6050 module while sending data over bluetooth.
 */
const int use1MotionTrackingDeviceCommand = 2;

/*
 * Receiving this command means that we should use the first, and the second MPU-6050 modules while sending data over
 * bluetooth.
 */
const int use2MotionTrackingDevicesCommand = 3;

/*
 * Indicates whether we should send data over bluetooth or not.
 * Default value is sending.
 * This parameter can be re-configured via commands received over bluetooth.
 */
bool sendData = true;

/*
 * Indicates how many MPU-6050 modules we should use.
 * Default value is all of the MPU-6050 modules available.
 * This parameter can be re-configured via commands received over bluetooth.
 */
int numberOfUsedMotionTrackingDevices = numberOfMotionTrackingDevices;

void setupUartSerialProtocol();

void setupI2cSerialProtocol();

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

void handleCommands();

int commandAvailable();

int getCommand();

void handleData();

void collectData();

void sendDataToTerminalInAscii();

void sendDataOverBluetoothInAscii();

void setup() {
    setupUartSerialProtocol();
    setupI2cSerialProtocol();
    setupAddressPins();
    setupMotionTrackingDevices();
    wakeUpMotionTrackingDevices();
}

void setupUartSerialProtocol() {
    /*
     * General UART serial protocol frame format is as below:
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
     * As a result, frames used for UART serial protocol are overall 10 bits and as below:
     * +-------+------+------+
     * | start | data | stop |
     * +-------+------+------+
     * |   1   |   8  |   1  |
     * +-------+------+------+
     *
     * Set baud rate to 38400 bps, which is the minimum baud rate possible to meet the requirements of the system.
     * The reason behind this decision is explained here:
     *
     * Number of bits we need to send in 1 second would be calculated as below:
     *
     *                                   2 x (2 x 3 x 6 + 12) x 10 x 25 = 24000 bps
     *                                   |    |   |   |    |     |    |
     *                                  (1)  (2) (3) (4)  (5)   (6)  (7)
     *
     *      (1) Number of MPU-6050 modules = 2
     *
     *      (2) Number of sensors of each module (accelerometer, gyroscope) = 2
     *
     *      (3) Number of axes of each sensor (x, y, z) = 3
     *
     *      (4) Number of bytes needed to represent data of one axis of one sensor in ASCII output format = 6
     *          Why 6? Data of one axis of one sensor is stored in a 16-bit register. It means that the range for that
     *          kind of data is (-32768, 32767). As a result in ASCII output format, we would need 6 character or 6
     *          bytes at maximum to represent that kind of data.
     *
     *      (5) Number of indicators = 12
     *          Why 12? We need 11 characters or bytes to separate data of each axis of each sensor in a sample (it
     *          would be comma character ',' in ASCII output format). We also need 1 character or byte to separate
     *          samples from each other (it would be end-of-line character '\n' in ASCII output format).
     *
     *      (6) UART serial protocol data frame size = 10
     *
     *      (7) Sampling frequency (number of samples in 1 second) = 25
     *
     * So, the first standard baud rate bigger than 24000 bps is 38400 bps, and that's why we have chosen that.
     */
    const unsigned long baudRate = 38400;

    /*
     * Setup hardware UART serial protocol.
     * Used for debugging.
     */
    Serial.begin(baudRate);

    /*
     * Setup software UART serial protocol.
     * Used for communication between Arduino Uno and HC-05 module.
     */
    bluetoothDevice.begin(baudRate);
}

void setupI2cSerialProtocol() {
    /*
     * Set clock Frequency of I2C serial protocol to the default value, which is the standard mode (100 KHz). To set
     * the clock frequency to the fast mode (400 KHz), use the code below:
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
    for (int index = 0; index < numberOfMotionTrackingDevices; index++) {
        selectMotionTrackingDevice(index);
        setClockSource();
        setRangeAndSensitivity();
        setLowPassFilter();
        setSampleRate();
        setCalibrationOffsets(index);
    }
}

void selectMotionTrackingDevice(int selectedIndex) {
    for (int index = 0; index < numberOfMotionTrackingDevices; index++) {
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
    selectedMotionTrackingDevice.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
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
    selectedMotionTrackingDevice.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);

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
     * Reading 32.8 from gyroscope over one of its axes means the angular velocity of +1 dps around that axis.
     */
    selectedMotionTrackingDevice.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
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
    selectedMotionTrackingDevice.setDLPFMode(MPU6050_DLPF_BW_5);
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
    selectedMotionTrackingDevice.setRate(39);
}

void setCalibrationOffsets(int selectedIndex) {
    CalibrationData calibrationData = calibrationDataArray[selectedIndex];
    selectedMotionTrackingDevice.setXAccelOffset(calibrationData.accelerometerX);
    selectedMotionTrackingDevice.setYAccelOffset(calibrationData.accelerometerY);
    selectedMotionTrackingDevice.setZAccelOffset(calibrationData.accelerometerZ);
    selectedMotionTrackingDevice.setXGyroOffset(calibrationData.gyroscopeX);
    selectedMotionTrackingDevice.setYGyroOffset(calibrationData.gyroscopeY);
    selectedMotionTrackingDevice.setZGyroOffset(calibrationData.gyroscopeZ);
}

void wakeUpMotionTrackingDevices() {
    for (int index = 0; index < numberOfMotionTrackingDevices; index++) {
        selectMotionTrackingDevice(index);
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
     * 3.8 mA current.
     */
    selectedMotionTrackingDevice.setSleepEnabled(false);
}

void loop() {
    handleCommands();
    handleData();
}

void handleCommands() {
    if (commandAvailable()) {
        int command = getCommand();
        switch (command) {
            case pauseCommand:
                sendData = false;
                break;

            case resumeCommand:
                sendData = true;
                break;

            case use1MotionTrackingDeviceCommand:
                numberOfUsedMotionTrackingDevices = 1;
                break;

            case use2MotionTrackingDevicesCommand:
                numberOfUsedMotionTrackingDevices = 2;

            default:
                break;
        }
    }
}

int commandAvailable() {
    return bluetoothDevice.available();
}

int getCommand() {
    return bluetoothDevice.read();
}

void handleData() {
    if (sendData) {
        collectData();
        sendDataToTerminalInAscii();
        sendDataOverBluetoothInAscii();

        /*
         * We add some kind of delay here, because sample rate is 25 Hz and there is no need to read sensor values more
         * than 25 times in a second.
         *
         * I found that, under current configurations, reading a sample takes around 20 ms. As a result, we need 20 ms
         * delay between reading each sample to meet the sampling frequency of 25 Hz:
         *
         * (1000 ms - 25 x 20 ms) / 25 = 20 ms
         */
        delay(20);
    }
}

void collectData() {
    for (int index = 0; index < numberOfUsedMotionTrackingDevices; index++) {
        selectMotionTrackingDevice(index);
        selectedMotionTrackingDevice.getMotion6(
                &dataArray[index].accelerometerX,
                &dataArray[index].accelerometerY,
                &dataArray[index].accelerometerZ,
                &dataArray[index].gyroscopeX,
                &dataArray[index].gyroscopeY,
                &dataArray[index].gyroscopeZ);
    }
}

void sendDataToTerminalInAscii() {
    for (int index = 0; index < numberOfUsedMotionTrackingDevices; index++) {
        Serial.print(dataArray[index].accelerometerX);
        Serial.print(",");
        Serial.print(dataArray[index].accelerometerY);
        Serial.print(",");
        Serial.print(dataArray[index].accelerometerZ);
        Serial.print(",");
        Serial.print(dataArray[index].gyroscopeX);
        Serial.print(",");
        Serial.print(dataArray[index].gyroscopeY);
        Serial.print(",");
        Serial.print(dataArray[index].gyroscopeZ);

        if (index == numberOfUsedMotionTrackingDevices - 1) {
            Serial.println();
        } else {
            Serial.print(",");
        }
    }
}

void sendDataOverBluetoothInAscii() {
    for (int index = 0; index < numberOfUsedMotionTrackingDevices; index++) {
        bluetoothDevice.print(dataArray[index].accelerometerX);
        bluetoothDevice.print(",");
        bluetoothDevice.print(dataArray[index].accelerometerY);
        bluetoothDevice.print(",");
        bluetoothDevice.print(dataArray[index].accelerometerZ);
        bluetoothDevice.print(",");
        bluetoothDevice.print(dataArray[index].gyroscopeX);
        bluetoothDevice.print(",");
        bluetoothDevice.print(dataArray[index].gyroscopeY);
        bluetoothDevice.print(",");
        bluetoothDevice.print(dataArray[index].gyroscopeZ);

        if (index == numberOfUsedMotionTrackingDevices - 1) {
            bluetoothDevice.println();
        } else {
            bluetoothDevice.print(",");
        }
    }
}