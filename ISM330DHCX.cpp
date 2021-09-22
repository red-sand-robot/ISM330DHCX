// I2Cdev library collection - ISM330DHCX I2C device class
// Based on STMicroelectronics ISM330DHCX datasheet DS13012 - Rev 7 - November 2020 
// https://www.st.com/en/mems-and-sensors/ism330dhcx.html
// 2021 by red-sand-robot
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib (if uploaded to that repository)
//
// Changelog:
//     [] Initial Release

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 red-sand-robot, Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "ISM330DHCX.h" 

/** Default constructor, uses default I2C address.
 * @see ISM330DHCX_DEFAULT_ADDRESS
 */
ISM330DHCX::ISM330DHCX(){
    devAddr = ISM330DHCX_DEFAULT_ADDRESS;
}


/** Specific address constructor.
 * @param address defined I2C address
 * @see ISM330DHCX_DEFAULT_ADDRESS
 */

ISM330DHCX::ISM330DHCX(uint8_t address){
    devAddr = address;    
}

/** Power on and prepare for general usage.
 * This will activate the device. This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec.
 */

void ISM330DHCX::initialize(){
    seti2cDisabled(false); // ensures that SPI/I2C interfaces are enabled. true to disable I2C if SPI is being used 
    setProperDeviceConfigurationEnabled(true);
    //configure accelerometer on startup
    setAccelOutputDataRate(ISM330DHCX_ODR_XL_208);
    setAccelFullScale(ISM330DHCX_FS_XL_4G);
    //configure gyroscope on startup
    setGyroOutputDataRate(ISM330DHCX_ODR_G_208);
    setGyroFullScale(ISM330DHCX_FS_G_2000DPS);
    // enabled timestamp on startup
    setTimestampEnabled(true);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return true if connection is valid, false otherwise
 * 
 */
bool ISM330DHCX::testConnection() {
    return getDeviceID() == 0x6B;
}

// FUNC_CFG_ACCESS register, r/w
// =============================
/** Set the status of the embedded functions enabled or disabled.
 * 
 * @param enabled 
 */
void ISM330DHCX::setEmbeddedFunctionsConfigurationEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, ISM330DHCX_RA_FUNC_CFG_ACCESS, ISM330DHCX_FUNC_CFG_ACCESS_BIT, enabled);
}

/** Get the status of the embedded functions enabled or disabled.
 * 
 * @return true if enabled, false if disabled
 */
bool ISM330DHCX::getEmbeddedFunctionsConfigurationEnabled(){
    I2Cdev::readBit(devAddr, ISM330DHCX_RA_FUNC_CFG_ACCESS, ISM330DHCX_FUNC_CFG_ACCESS_BIT, buffer);
    return buffer[0];
}

/** Set the status of the sensor hub enabled or disabled.
 * 
 * @param enabled 
 */
void ISM330DHCX::setSensorHubRegistersEnabled(bool enabled){
    I2Cdev::writeBit(devAddr, ISM330DHCX_RA_FUNC_CFG_ACCESS, ISM330DHCX_SHUB_REG_ACCESS_BIT, enabled);
}

/** Get the status of the sensor hub enabled or disabled.
 * 
 * @return true if enabled, false if disabled 
 */
bool ISM330DHCX::getSensorHubRegistersEnabled(){
    I2Cdev::readBit(devAddr, ISM330DHCX_RA_FUNC_CFG_ACCESS, ISM330DHCX_SHUB_REG_ACCESS_BIT, buffer);
    return buffer[0];
}

// INT1_CTRL, r/w
// =========================================================

/** Set the gyro data ready interupt on INT1
 * 
 * @param enabled 
 */
void ISM330DHCX::setGyroDataReadyInteruptOnINT1Enabled(bool enabled){
    I2Cdev::writeBit(devAddr, ISM330DHCX_RA_INT1_CTRL, ISM330DHCX_INT1_DRDY_G_BIT, enabled);
}


bool ISM330DHCX::getGyroDataReadyInteruptOnINT1Enabled(){
    I2Cdev::readBit(devAddr, ISM330DHCX_RA_INT1_CTRL, ISM330DHCX_INT1_DRDY_G_BIT, buffer);
    return buffer[0];
}



// WHO_AM_I register, read-only
// ============================
uint8_t ISM330DHCX::getDeviceID() {
    I2Cdev::readByte(devAddr, ISM330DHCX_RA_WHOAMI, buffer);
    return buffer[0];
}

// CTRL1_XL, r/w
// =============
/** set accelerometer operational data rate
 * @param rate 
 * ISM330DHCX_ODR_XL_1_6, ISM330DHCX_ODR_XL_12_5, ISM330DHCX_ODR_XL_26, ISM330DHCX_ODR_XL_52,
 * ISM330DHCX_ODR_XL_104, ISM330DHCX_ODR_XL_208, ISM330DHCX_ODR_XL_416, ISM330DHCX_ODR_XL_833, 
 * ISM330DHCX_ODR_XL_1666, ISM330DHCX_ODR_XL_3333, ISM330DHCX_ODR_XL_6667 
 */ 
void ISM330DHCX::setAccelOutputDataRate(uint8_t rate) {
    I2Cdev::writeBits(devAddr, ISM330DHCX_RA_CTRL1_XL, ISM330DHCX_ODR_XL_BIT, ISM330DHCX_ODR_XL_LENGTH, rate);
}

/** get the current accelerometer operation data rate setting
 * @return buffer 
 */ 
uint8_t ISM330DHCX::getAccelOutputDataRate(){
    I2Cdev::readBits(devAddr, ISM330DHCX_RA_CTRL1_XL, ISM330DHCX_ODR_XL_BIT, ISM330DHCX_ODR_XL_LENGTH, buffer);
    return buffer[0];
}

/** set accelerometer scale
 *  @param scale
 */ 
void ISM330DHCX::setAccelFullScale(uint8_t scale){
    I2Cdev::writeBits(devAddr, ISM330DHCX_RA_CTRL1_XL, ISM330DHCX_FS_XL_BIT, ISM330DHCX_FS_XL_LENGTH, scale);
}

/** get current accelerometer scale
 * 
 * @return buffer
 */ 
uint8_t ISM330DHCX::getAccelFullScale(){
    I2Cdev::readBits(devAddr, ISM330DHCX_RA_CTRL1_XL, ISM330DHCX_FS_XL_BIT, ISM330DHCX_FS_XL_LENGTH, buffer);
    return buffer[0];
}

/** accelerometer high resolution selection
 * 
 * @param enabled
 * 
 * 0: output from first stage digital filtering selected (default)
 * 1: output from LPF2 second filtering stage selected
 */  
void ISM330DHCX::setAccelLowPassFilter2Enabled(bool enabled){
    I2Cdev::writeBit(devAddr, ISM330DHCX_RA_CTRL1_XL, ISM330DHCX_LPF2_XL_EN_BIT, enabled);
}

/** get accelerometer high resolution setting 
 * 
 * @return 
 * 0: output from first stage digital filtering selected (default)
 * 1: output from LPF2 second filtering stage selected
 */ 
bool ISM330DHCX::getAccelLowPassFilter2Enabled(){
    I2Cdev::readBit(devAddr, ISM330DHCX_RA_CTRL1_XL, ISM330DHCX_LPF2_XL_EN_BIT, buffer);
    return buffer [0];
}

// CTRL2_G, r/w
// =============
/** set gyro operational data rate
 * 
 * @param rate
 */ 
void ISM330DHCX::setGyroOutputDataRate(uint8_t rate){
    I2Cdev::writeBits(devAddr, ISM330DHCX_RA_CTRL2_G, ISM330DHCX_ODR_G_BIT, ISM330DHCX_ODR_G_LENGTH, rate);
}

/** get current gyro operational data rate
 * 
 * @return buffer
 */ 
uint8_t ISM330DHCX::getGyroOutputDataRate(){
    I2Cdev::readBits(devAddr, ISM330DHCX_RA_CTRL2_G, ISM330DHCX_ODR_G_BIT, ISM330DHCX_ODR_G_LENGTH, buffer);
    return buffer[0];
}

/** set gyroscope scale
 * 
 * @param scale 
 */ 
void ISM330DHCX::setGyroFullScale(uint8_t scale){
    I2Cdev::writeBits(devAddr, ISM330DHCX_RA_CTRL2_G, ISM330DHCX_FS_G_BIT, ISM330DHCX_FS_G_LENGTH, scale);
}

/** get current gyroscope scale setting
 * 
 * @return buffer
 */       
uint8_t ISM330DHCX::getGyroFullScale(){
    I2Cdev::readBits(devAddr, ISM330DHCX_RA_CTRL2_G, ISM330DHCX_FS_G_BIT, ISM330DHCX_FS_G_LENGTH, buffer);
    return buffer[0];
}

// CTRL3_C
// =======
/* todo

void RebootMemoryContent(bool reboot);
*/

/** causes output registers to not update until MSB and LSB have been read
 * 
 * @param block
 */ 
void ISM330DHCX::setBlockDataUpdate(bool block){
    I2Cdev::writeBit(devAddr, ISM330DHCX_RA_CTRL3_C, ISM330DHCX_BDU_BIT, block);    
}

bool ISM330DHCX::getBlockDataUpdate(){
    I2Cdev::readBit(devAddr, ISM330DHCX_RA_CTRL3_C, ISM330DHCX_BDU_BIT, buffer);
    return buffer [0];
}

/*
void setInteruptActivationLevel(bool level);
bool getInteruptActivationLevel();

void setPushPullOrOpenDrainOnINT1AndINT2Selection(bool selection);
bool getPushPullOrOpenDrainOnINT1AndINT2Selection();

void setSPISerialInterfaceMode(bool mode);
bool getSPISerialInterfaceMode();
*/

/** sets register addresses to automatically increment during a multiple byte access with a serial interface (I²C or SPI) 
 * Default value: 1 (0: disabled; 1: enabled)
 * 
 * @param enabled 
 */ 
void ISM330DHCX::setAutoRegisterIncrementOnMultiByteReadEnabled(bool enabled){
    I2Cdev::writeBit(devAddr, ISM330DHCX_RA_CTRL3_C, ISM330DHCX_IF_INC_BIT, enabled);
}

/** get the state of whether or not the register address increment during multi-byte reads with a serial interface (I²C or SPI)
 * 
 * @return buffer
 */ 
bool ISM330DHCX::getAutoRegisterIncrementOnMultiByteReadEnabled(){
    I2Cdev::readBit(devAddr, ISM330DHCX_RA_CTRL3_C, ISM330DHCX_IF_INC_BIT, buffer);
    return buffer[0]; 
}

// CTRL4_C, r/w
// ============

void ISM330DHCX::setAllInteruptSignalsOnINT1Enabled(bool enabled){
    I2Cdev::writeBit(devAddr, ISM330DHCX_RA_CTRL4_C, ISM330DHCX_INT2_on_INT1_BIT, enabled);
}

bool ISM330DHCX::getAllInteruptSignalsOnINT1Enabled(){
    I2Cdev::readBit(devAddr, ISM330DHCX_RA_CTRL4_C, ISM330DHCX_INT2_on_INT1_BIT, buffer);
    return buffer[0];
}



void ISM330DHCX::seti2cDisabled(bool disabled){
    I2Cdev::writeBit(devAddr, ISM330DHCX_RA_CTRL4_C, ISM330DHCX_I2C_disable_BIT, disabled);
}

bool ISM330DHCX::geti2cDisabled(){
    I2Cdev::readBit(devAddr, ISM330DHCX_RA_CTRL4_C, ISM330DHCX_I2C_disable_BIT, buffer);
    return buffer[0];
}


/** enables gyroscope digital LPF1; bandwidth can be selected through FTYPE[2:0] in CTRL6_C
 * @param enabled
 */ 
void ISM330DHCX::setGyroLowPassFilter1Enabled(bool enabled){
    I2Cdev::writeBit(devAddr, ISM330DHCX_RA_CTRL4_C, ISM330DHCX_LPF1_SEL_G_BIT, enabled);
}

/** checks enabled state of the gyroscope low pass filter
 * @return ture if enabled, false otherwise
 */ 
bool ISM330DHCX::getGyroLowPassFilter1Enabled(){
    I2Cdev::readBit(devAddr, ISM330DHCX_RA_CTRL4_C, ISM330DHCX_LPF1_SEL_G_BIT, buffer);
    return buffer[0];
}

// CTRL6_C, r/w
// =============

/** set the bandwidth of the gyroscope low pass filter 1
 * see Table 58 in the datasheet or header file for selectable values
 * 
 * @param bandwidth
 */ 
void ISM330DHCX::setGyroLowPassFilter1Bandwidth(uint8_t bandwidth){
    I2Cdev::writeBits(devAddr, ISM330DHCX_RA_CTRL6_C, ISM330DHCX_FTYPE_BIT, ISM330DHCX_FTYPE_LENGTH, bandwidth);
}

/**  get the current gyroscope low pass filter bandwidth setting
 * 
 * @return buffer
 */ 
uint8_t ISM330DHCX::getGyroLowPassFilter1Bandwidth(){
    I2Cdev::readBits(devAddr, ISM330DHCX_RA_CTRL6_C, ISM330DHCX_FTYPE_BIT, ISM330DHCX_FTYPE_LENGTH, buffer);
    return buffer[0];
}

// CTRL7_G, r/w
// =========================================================


void ISM330DHCX::setGyroHighPassFilterEnabled(bool enabled){
    I2Cdev::writeBit(devAddr, ISM330DHCX_RA_CTRL7_C, ISM330DHCX_HP_EN_G_BIT, enabled);
}

void ISM330DHCX::setGyroHighPassFilterCutoffBandwidth(uint8_t bandwidth){
    I2Cdev::writeBits(devAddr, ISM330DHCX_RA_CTRL7_C, ISM330DHCX_HPM_G_BIT, ISM330DHCX_HPM_G_LENGTH, bandwidth);
}
// CTRL9_XL, r/w
// ============

/** ensures device is properly configured
 * @param enabled
 */ 
void ISM330DHCX::setProperDeviceConfigurationEnabled(bool enabled){
    I2Cdev::writeBit(devAddr, ISM330DHCX_RA_CTRL9_XL, ISM330DHCX_DEVICE_CONF_BIT, enabled);
}

// CTRL10_C, r/w
// =============

/** enables timestamp functionality
 * @param enabled
 */ 
void ISM330DHCX::setTimestampEnabled(bool enabled){
    I2Cdev::writeBit(devAddr, ISM330DHCX_RA_CTRL10_C, ISM330DHCX_TIMESTAMP_EN_BIT, enabled);
}

/** Get raw 6-axis motion sensor readings (accel/gyro)
 *  Retrieves all currently available motion sensor values.
 * @param rax 16-bit signed integer container for accelerometer X-axis value
 * @param ray 16-bit signed integer container for accelerometer Y-axis value
 * @param raz 16-bit signed integer container for accelerometer Z-axis value
 * @param rgx 16-bit signed integer container for gyroscope X-axis value
 * @param rgy 16-bit signed integer container for gyroscope Y-axis value
 * @param rgz 16-bit signed integer container for gyroscope Z-axis value
 */ 
void ISM330DHCX::getRawMotion6(int16_t* rgx, int16_t* rgy, int16_t* rgz, int16_t* rax, int16_t* ray, int16_t* raz){
    I2Cdev::readBytes(devAddr, ISM330DHCX_RA_OUTX_L_G, 12, buffer);
    *rgx = (((int16_t)buffer[1]) << 8) | buffer[0];
    *rgy = (((int16_t)buffer[3]) << 8) | buffer[2];
    *rgz = (((int16_t)buffer[5]) << 8) | buffer[4];
    *rax = (((int16_t)buffer[7]) << 8) | buffer[6];
    *ray = (((int16_t)buffer[9]) << 8) | buffer[8];
    *raz = (((int16_t)buffer[11]) << 8) | buffer[10];
}

/** get timestamp
 * @param ts 32-bit unsigned integer container for timestamp value  
 */ 
 void ISM330DHCX::getTimestamp(uint32_t* ts){
     I2Cdev::readBytes(devAddr, ISM330DHCX_RA_TIMESTAMP0, 4, buffer);
     *ts = (((uint32_t)buffer[3]) << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];
 }

// OUT_TEMP_L / OUT_TEMP_H, r
// Temperature data output register
// L and H registers together express a 16-bit word in two’s complement.
// =========================================================
int16_t ISM330DHCX::getTemperature(){
    I2Cdev::readBytes(devAddr, ISM330DHCX_RA_OUT_TEMP_L, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
} 