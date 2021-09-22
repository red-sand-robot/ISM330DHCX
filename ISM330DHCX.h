// I2Cdev library collection - ISM330DHCX I2C device class header file
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

#ifndef _ISM330DHCX_H
#define _ISM330DHCX_H

#include "I2Cdev.h"

#include <stdint.h>

// Tom Carpenter's conditional PROGMEM code
// http://forum.arduino.cc/index.php?topic=129407.0
/*
#ifdef __AVR__
    #include <avr/pgmspace.h>
#else
    // Teensy 3.0 library conditional PROGMEM code from Paul Stoffregen
    #ifndef __PGMSPACE_H_
        #define __PGMSPACE_H_ 1
        #include <inttypes.h>

        #define PROGMEM
        #define PGM_P  const char *
        #define PSTR(str) (str)
        #define F(x) x

        typedef void prog_void;
        typedef char prog_char;
        typedef unsigned char prog_uchar;
        typedef int8_t prog_int8_t;
        typedef uint8_t prog_uint8_t;
        typedef int16_t prog_int16_t;
        typedef uint16_t prog_uint16_t;
        typedef int32_t prog_int32_t;
        typedef uint32_t prog_uint32_t;

        #define strcpy_P(dest, src) strcpy((dest), (src))
        #define strcat_P(dest, src) strcat((dest), (src))
        #define strcmp_P(a, b) strcmp((a), (b))

        #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
        #define pgm_read_word(addr) (*(const unsigned short *)(addr))
        #define pgm_read_dword(addr) (*(const unsigned long *)(addr))
        #define pgm_read_float(addr) (*(const float *)(addr))

        #define pgm_read_byte_near(addr) pgm_read_byte(addr)
        #define pgm_read_word_near(addr) pgm_read_word(addr)
        #define pgm_read_dword_near(addr) pgm_read_dword(addr)
        #define pgm_read_float_near(addr) pgm_read_float(addr)
        #define pgm_read_byte_far(addr) pgm_read_byte(addr)
        #define pgm_read_word_far(addr) pgm_read_word(addr)
        #define pgm_read_dword_far(addr) pgm_read_dword(addr)
        #define pgm_read_float_far(addr) pgm_read_float(addr)
    #endif
#endif
*/

// <--ISM330DHCX registers and their definitions-->

#define ISM330DHCX_ADDRESS_AD0_LOW                0x6B     // address pin low (GND), default i2c address
#define ISM330DHCX_ADDRESS_AD0_HIGH               0x6A     // address pin high (jumper from DO to 3.3 / bridge the solder jumper on the back of the board) 
#define ISM330DHCX_DEFAULT_ADDRESS                ISM330DHCX_ADDRESS_AD0_HIGH

#define ISM330DHCX_RA_FUNC_CFG_ACCESS             0x01     // Enable embedded functions register
#define ISM330DHCX_RA_PIN_CTRL                    0x02     // Pin control register; SDO, OCS_AUX, SDO_AUX pins pull-up enable/disable register (r/w)
 
#define ISM330DHCX_RA_FIFO_CTRL1                  0x07     // FIFO control register 1 
#define ISM330DHCX_RA_FIFO_CTRL2                  0x08     // FIFO control register 2
#define ISM330DHCX_RA_FIFO_CTRL3                  0x09     // FIFO control register 3
#define ISM330DHCX_RA_FIFO_CTRL4                  0x0A     // FIFO control register 4
#define ISM330DHCX_RA_COUNTER_BDR_REG1            0x0B     // Counter batch data register 1
#define ISM330DHCX_RA_COUNTER_BDR_REG2            0x0C     // Counter batch data register 2
 
#define ISM330DHCX_RA_INT1_CTRL                   0x0D     // Interrupt control for INT 1
#define ISM330DHCX_RA_INT2_CTRL                   0x0E     // Interrupt control for INT 2
#define ISM330DHCX_RA_WHOAMI                      0x0F     // Chip ID register
#define ISM330DHCX_RA_CTRL1_XL                    0x10     // Main accelerometer config register
#define ISM330DHCX_RA_CTRL2_G                     0x11     // Main gyro config register
#define ISM330DHCX_RA_CTRL3_C                     0x12     // Main configuration register
 
#define ISM330DHCX_RA_CTRL4_C                     0x13     // Control register 4 - can be used to enable gyroscope sleep mode(?) refer to datasheet for full register description
#define ISM330DHCX_RA_CTRL5_C                     0x14     // Control register 5 - can be used to enable rounding of accel/gyro data registers / can be used to enable self testing for accel/gyro
#define ISM330DHCX_RA_CTRL6_C                     0x15     // Control register 6 - can be used to change trigger modes, disable/enable high performance mode for accel (default: enabled), select bandwidth for gyroscope low pass filter
#define ISM330DHCX_RA_CTRL7_C                     0x16     // Control register 7 - can be used to enable/disable high-performance operating mode for gyroscope (default: enabled), enable/disable gyro high-pass filter (default: disabled)
#define ISM330DHCX_RA_CTRL8_XL                    0x17     // High and low pass for accel
#define ISM330DHCX_RA_CTRL9_XL                    0x18     // Includes i3c disable bit
#define ISM330DHCX_RA_CTRL10_C                    0x19     // Main configuration register

#define ISM330DHCX_RA_ALL_INT_SRC                 0x1A     // Source register for all interupts 

#define ISM330DHCX_RA_WAKEUP_SRC                  0x1B     // Why we woke up - detects changes in events, like freefall
 
#define ISM330DHCX_RA_TAP_SRC                     0x1C     // Tap source register
#define ISM330DHCX_RA_DRD_SRC                     0x1D     // Portrait, landscape, face-up and face-down register - probably intended for use in cellphones?
#define ISM330DHCX_RA_STATUS_REG                  0x1E     // Register read by the primary interface i2c/SPI
#define ISM330DHCX_RA_STATUS_SPIAUX               0x1E     // Register read by the auxillary SPI - NOTE: same register as ISM330DHCX_STATUS_REG

//---Temperature data registers
#define ISM330DHCX_RA_OUT_TEMP_L                  0x20     // First data register (temperature low)
#define ISM330DHCX_RA_OUT_TEMP_H                  0x21     // Second data register (temperature high)


//---Gyroscope data registers
// x-axis
#define ISM330DHCX_RA_OUTX_L_G                    0x22     // First x-axis gyro data register
#define ISM330DHCX_RA_OUTX_H_G                    0x23     // Second x-axis gyro data register
// y-axis
#define ISM330DHCX_RA_OUTY_L_G                    0x24     // First y-axis gyro data register
#define ISM330DHCX_RA_OUTY_H_G                    0x25     // Second y-axis gyro data register
// z-axes
#define ISM330DHCX_RA_OUTZ_L_G                    0x26     // First z-axis gyro data register
#define ISM330DHCX_RA_OUTZ_H_G                    0x27     // Second z-axis gyro data register

//---Accelerometer data registers
// x-axis
#define ISM330DHCX_RA_OUTX_L_A                    0x28     // First x-axes accel data register
#define ISM330DHCX_RA_OUTX_H_A                    0x29     // Second x-axes accel data register
// y-axis
#define ISM330DHCX_RA_OUTY_L_A                    0x2A     // First y-axes accel data register
#define ISM330DHCX_RA_OUTY_H_A                    0x2B     // Second y-axes accel data register
// y-axis
#define ISM330DHCX_RA_OUTZ_L_A                    0x2C     // First z-axes accel data register
#define ISM330DHCX_RA_OUTZ_H_A                    0x2D     // Second z-axes accel data register

#define ISM330DHCX_RA_EMB_FUNC_STATUS_MAINPAGE    0x35     // Embedded function status register
#define ISM330DHCX_RA_FSM_STATUS_A_MAINPAGE       0x36     // Finite State Machine status register
#define ISM330DHCX_RA_FSM_STATUS_B_MAINPAGE       0x37     // Finite State Machine status register
#define ISM330DHCX_RA_MLC_STATUS_MAINPAGE         0x38     // Machine Learning Core status register
#define ISM330DHCX_RA_STATUS_MASTER_MAINPAGE      0x39     // Sensor hub source register
#define ISM330DHCX_RA_FIFO_STATUS1                0x3A     // FIFO status register 1
#define ISM330DHCX_RA_FIFO_STATUS2                0x3B     // FIFO status register 2

//---Timestamp registers
#define ISM330DHCX_RA_TIMESTAMP0                  0x40     // Timestamp registers 0-3. Value is expressed as a 32-bit word and the bit resolution is 25 microseconds
#define ISM330DHCX_RA_TIMESTAMP1                  0x41     //
#define ISM330DHCX_RA_TIMESTAMP2                  0x42     //
#define ISM330DHCX_RA_TIMESTAMP3                  0x43     //

#define ISM330DHCX_RA_TAP_CFG0                    0x56     // Tap/pedometer configuration
#define ISM330DHCX_RA_TAP_CFG1                    0x57     // Tap/pedometer configuration
#define ISM330DHCX_RA_TAP_CFG2                    0x58     // Tap/pedometer configuration
#define ISM330DHCX_RA_TAP_THS_6D                  0x59     // Tap/pedometer configuration

#define ISM330DHCX_RA_INT_DUR2                    0X5A     // Tap recognition function setting register

#define ISM330DHCX_RA_WAKEUP_THS                  0x5B     // Single and double-tap function threshold register
#define ISM330DHCX_RA_WAKEUP_DUR                  0x5C     // Free-fall, wakeup, timestamp and sleep mode duration
#define ISM330DHCX_RA_FREE_FALL                   0x5D     // Free-fall

#define ISM330DHCX_RA_MD1_CFG                     0x5E     // Functions routing on INT1 register
#define ISM330DHCX_RA_MD2_CFG                     0x5F     // Functions routing on INT2 register

#define ISM330DHCX_RA_INTERNAL_FREQ               0x63     // Internal frequency register
#define ISM330DHCX_RA_INT_OIS                     0x6F     // OIS interrupt configuration register and accelerometer self-test enable setting. Primary interface for read-only (r);only Aux SPI can write to this register.

#define ISM330DHCX_RA_CTRL1_OIS                   0x70     // OIS configuration register. Primary interface for read-only (r); only Aux SPI can write to this register (r/w)
#define ISM330DHCX_RA_CTRL2_OIS                   0x71     // OIS configuration register. Primary interface for read-only (r); only Aux SPI can write to this register (r/w)
#define ISM330DHCX_RA_CTRL3_OIS                   0x72     // OIS configuration register. Primary interface for read-only (r); only Aux SPI can write to this register (r/w)

#define ISM330DHCX_RA_X_OFS_USR                   0x73     // Accelerometer X-axis user offset correction (r/w). The offset value set in the X_OFS_USR offset register isinternally subtracted from the acceleration value measured on the X-axis.                                                       
#define ISM330DHCX_RA_Y_OFS_USR                   0x74     // Accelerometer Y-axis user offset correction (r/w). The offset value set in the Y_OFS_USR offset register isinternally subtracted from the acceleration value measured on the Y-axis.
#define ISM330DHCX_RA_Z_OFS_USR                   0x75     // Accelerometer Z-axis user offset correction (r/w). The offset value set in the Z_OFS_USR offset register isinternally subtracted from the acceleration value measured on the Z-axis

#define ISM330DHCX_RA_FIFO_DATA_OUT_TAG           0x78     // FIFO tag register
#define ISM330DHCX_RA_FIFO_DATA_OUT_X_L           0x79     // FIFO data output X (low)
#define ISM330DHCX_RA_FIFO_DATA_OUT_X_H           0x7A     // FIFO data output X (high)
#define ISM330DHCX_RA_FIFO_DATA_OUT_Y_L           0x7B     // FIFO data output Y (low)
#define ISM330DHCX_RA_FIFO_DATA_OUT_Y_H           0x7C     // FIFO data output Y (high)
#define ISM330DHCX_RA_FIFO_DATA_OUT_Z_L           0x7D     // FIFO data output Z (low)
#define ISM330DHCX_RA_FIFO_DATA_OUT_Z_H           0x7E     // FIFO data output Z (high)
 

#define ISM330DHCX_RA_CHIP_ID                     0x6B     // ISM330DHCX default device id from WHOAMI



//FUNC_CFG_ACCESS - 0x01 - Enable embedded functions register (r/w)
#define ISM330DHCX_FUNC_CFG_ACCESS_BIT 7                // Enable access to the embedded functions configuration registers; default value: 0
#define ISM330DHCX_SHUB_REG_ACCESS_BIT 6                // Enable access to the sensor hub (I²C master) registers; default value: 0
//NOTE: bits 0-5 must be set to 0 for correct operation of the IMU


//PIN_CTRL - 0x02 - SDO, OCS_AUX, SDO_AUX pins pull-up enable/disable register (r/w)
#define ISM330DHCX_OIS_PU_DIS_BIT 7                     // Disable pull-up on both OCS_Aux and SDO_Aux pins. Default value: (0: OCS_Aux and SDO_Aux pins with pull-up;1: OCS_Aux and SDO_Aux pins pull-up disconnected)
#define ISM330DHCX_SDO_PU_EN_BIT 6                      // Enable pull-up on SDO pin. Default value: 0; (0: SDO pin pull-up disconnected (default); 1: SDO pin with pull-up)
//NOTE: bits 0-5 must be set to 0 for correct operation of the IMU


//FIF0_CTRL2 - 0x08 - FIFO control register 2 (r/w)
#define ISM330DHCX_STOP_ON_WTM_BIT 7                        // Sensing chain FIFO stop values memorization at threshold level; (0: FIFO depth is not limited (default);1: FIFO depth is limited to the threshold level, defined in FIFO_CTRL1 (07h) and FIFO_CTRL2(08h)
#define ISM330DHCX_FIFO_COMPR_RT_EN_BIT 6                   // Enables/Disables compression algorithm runtime; effective if the FIFO_COMPR_EN bit of EMB_FUNC_EN_B (05h) is set to 1
#define ISM330DHCX_ODRCHG_EN_BIT 4                      //Enables ODR CHANGE virtual sensor to be batched in FIFO
#define ISM330DHCX_UNCOPTR_RATE_BIT 2                   // Configures the compression algorithm to write non-compressed data at each rate.
#define ISM330DHCX_UNCOPTR_RATE_LENGTH 2

#define ISM330DHCX_UNCOPTR_RATE_NOTFORCED 0             // Non-compressed data writing is not forced
#define ISM330DHCX_UNCOPTR_RATE_8_BDR 1                 // Non-compressed data every 8 batch data rate
#define ISM330DHCX_UNCOPTR_RATE_16_BDR 2                // Non-compressed data every 16 batch data rate
#define ISM330DHCX_UNCOPTR_RATE_32_BDR 3                // Non-compressed data every 32 batch data rate
//NOTE: bits 5 and 3 must be set to 0 for correct operation of the IMU


//FIFO_CTRL3 - 0x09 Selects Batch Data Rate for the gyroscope and accelerometer. Bits 3:0 are for the accelerometer, bits 7:4 are for the gyroscope.
#define ISM330DHCX_BDR_GY_BIT 7
#define ISM330DHCX_BDR_GY_LENGTH 4
#define ISM330DHCX_BDR_XL_BIT 4
#define ISM330DHCX_BDR_XY_LENGTH 4

#define ISM330DHCX_BDR_GY_NOTBATCHED 0b0000
#define ISM330DHCX_BDR_GY_12_5 0b0001
#define ISM330DHCX_BDR_GY_26 0b0010
#define ISM330DHCX_BDR_GY_52 0b0011
#define ISM330DHCX_BDR_GY_104 0b0100
#define ISM330DHCX_BDR_GY_208  0b0101
#define ISM330DHCX_BDR_GY_417 0b0110
#define ISM330DHCX_BDR_GY_833 0b0111
#define ISM330DHCX_BDR_GY_1667 0b1000
#define ISM330DHCX_BDR_GY_3333 0b1001
#define ISM330DHCX_BDR_GY_6667 0b1010
#define ISM330DHCX_BDR_GY_6_5 0b1011

#define ISM330DHCX_BDR_XL_NOTBATCHED 0b0000
#define ISM330DHCX_BDR_XL_12_5 0b0001
#define ISM330DHCX_BDR_XL_26 0b0010
#define ISM330DHCX_BDR_XL_52 0b0011
#define ISM330DHCX_BDR_XL_104 0b0100
#define ISM330DHCX_BDR_XL_208  0b0101
#define ISM330DHCX_BDR_XL_417 0b0110
#define ISM330DHCX_BDR_XL_833 0b0111
#define ISM330DHCX_BDR_XL_1667 0b1000
#define ISM330DHCX_BDR_XL_3333 0b1001
#define ISM330DHCX_BDR_XL_6667 0b1010
#define ISM330DHCX_BDR_XL_6_5 0b1011


// FIFO_CTRL4
#define ISM330DHCX_DEC_TS_BATCH_BIT 7
#define ISM330DHCX_DEC_TS_BATCH_LENGTH 2
#define ISM330DHCX_ODR_T_BATCH_BIT 5
#define ISM330DHCX_ODR_T_BATCH_LENGTH 2
#define ISM330DHCX_FIFO_MODE_BIT 2
#define ISM330DHCX_FIFO_MODE_LENGTH 3

#define ISM330DHCX_DEC_TS_NOTBATCHED 0b00
#define ISM330DHCX_DEC_TS_BATCH_1 0b01
#define ISM330DHCX_DEC_TS_BATCH_8 0b10
#define ISM330DHCX_DEC_TS_BATCH_32 0b11

#define ISM330DHCX_ODR_T_NOTBATCHED 0b00
#define ISM330DHCX_ODR_T_ODR_T_BATCH_1_6 0b01
#define ISM330DHCX_ODR_T_ODR_T_BATCH_12_5 0b10
#define ISM330DHCX_ODR_T_ODR_T_BATCH_52 0b11

#define ISM330DHCX_FIFO_MODE_BYPASS 0b000
#define ISM330DHCX_FIFO_MODE_FIFO 0b001 // FIFO mode: stops collecting data when FIFO is full
#define ISM330DHCX_FIFO_MODE_CONT_TO_FIFO 0b011 // Continuous-to-FIFO mode: Continuous mode until trigger is deasserted, then FIFO mode
#define ISM330DHCX_FIFO_MODE_BYPASS_TO_CONT 0b100 // Bypass-to-Continuous mode: Bypass mode until trigger is deasserted, then Continuous mode
#define ISM330DHCX_FIFO_MODE_CONT_OVERWRITE 0b110 // Continuous mode: if the FIFO is full, the new sample overwrites the older one
#define ISM330DHCX_FIFO_MODE_BYPASS_TO_FIFO 0b111 // Bypass-to-FIFO mode: Bypass mode until trigger is deasserted, then FIFO mode

// COUNTER_BDR_REG1
#define ISM330DHCX_dataready_pulsed_BIT 7 // Enables pulsed data-ready mode. (0: Data-ready latched mode (returns to 0 only after an interface reading) (default);1: Data-ready pulsed mode (the data ready pulses are 75 μs long)
#define ISM330DHCX_RST_COUNTER_BDR_BIT 6 // Resets the internal counter of batch events for a single sensor.This bit is automatically reset to zero if it was set to ‘1’.
#define ISM330DHCX_TRIG_COUNTER_BDR_BIT 5 // Selects the trigger for the internal counter of batch events between XL and gyro.(0: XL batch event;1: GYRO batch event)

#define RST_COUNTER_BDR_RESET 1
#define TRIG_COUNTER_BDR_XL 0
#define TRIG_COUNTER_BDR_GYRO 1

// INT1_CTRL
#define ISM330DHCX_DEN_DRDY_flag_BIT 7 // Sends DEN_DRDY (DEN stamped on Sensor Data flag) to INT1 pin.
#define ISM330DHCX_INT1_CNT_BDR_BIT 6 // Enables COUNTER_BDR_IA interrupt on INT1.
#define ISM330DHCX_INT1_FIFO_FULL_BIT 5 // Enables FIFO full flag interrupt on INT1 pin.
#define ISM330DHCX_INT1_FIFO_OVR_BIT 4 // Enables FIFO overrun interrupt on INT1 pin.
#define ISM330DHCX_INT1_FIFO_TH_BIT 3 // Enables FIFO threshold interrupt on INT1 pin.
#define ISM330DHCX_INT1_BOOT_BIT 2 // Enables boot status on INT1 pin.
#define ISM330DHCX_INT1_DRDY_G_BIT 1 // Enables gyroscope data-ready interrupt on INT1 pin.
#define ISM330DHCX_INT1_DRDY_XL_BIT 0 //Enables accelerometer data-ready interrupt on INT1 pin.

// INT2_CTRL
#define ISM330DHCX_INT2_CNT_BDR_BIT 6 // Enables COUNTER_BDR_IA interrupt on INT2 pin.
#define ISM330DHCX_INT2_FIFO_FULL_BIT 5 // Enables FIFO full flag interrupt on INT2 pin.
#define ISM330DHCX_INT2_FIFO_OVR_BIT 4 // Enables FIFO overrun interrupt on INT2 pin.
#define ISM330DHCX_INT_FIFO_TH_BIT 3 // Enables FIFO threshold interrupt on INT2 pin.
#define ISM330DHCX_INT2_DRDY_TEMP_BIT 2 // Enables temperature sensor data-ready interrupt on INT2 pin.
#define ISM330DHCX_INT2_DRDY_G_BIT 1 // Enables gyroscope data-ready interrupt on INT2 pin.
#define ISM330DHCX_INT2_DRDY_XL_BIT 0 // Enables accelerometer data-ready interrupt on INT2 pin.

// CTRL1_XL
#define ISM330DHCX_ODR_XL_BIT 7 // accelerometer operational data rate selection starting bit
#define ISM330DHCX_ODR_XL_LENGTH 4
#define ISM330DHCX_FS_XL_BIT 3 // accelerometer full-scale selection start bit. 
#define ISM330DHCX_FS_XL_LENGTH 2
#define ISM330DHCX_LPF2_XL_EN_BIT 1 // accelerometer high-resolution selection. (0: output from first stage digital filtering selected (default); 1: output from LPF2 second filtering stage selected)

#define ISM330DHCX_ODR_XL_POWERDOWN 0b0000
#define ISM330DHCX_ODR_XL_1_6 0b1011 // low power only
#define ISM330DHCX_ODR_XL_12_5 0b0001
#define ISM330DHCX_ODR_XL_26 0b0011
#define ISM330DHCX_ODR_XL_52 0b0011
#define ISM330DHCX_ODR_XL_104 0b0100
#define ISM330DHCX_ODR_XL_208 0b0101
#define ISM330DHCX_ODR_XL_416 0b0110
#define ISM330DHCX_ODR_XL_833 0b0111
#define ISM330DHCX_ODR_XL_1666 0b1000
#define ISM330DHCX_ODR_XL_3333 0b1001
#define ISM330DHCX_ODR_XL_6667 0b1010

#define ISM330DHCX_FS_XL_2G 0b00
#define ISM330DHCX_FS_XL_16G 0b01
#define ISM330DHCX_FS_XL_4G 0b10
#define ISM330DHCX_FS_XL_8G 0b11

#define ISM330DHCX_LPF2_XL_ENABLE 1

// CTRL2_G
#define ISM330DHCX_ODR_G_BIT 7 // gyroscope output data rate selection starting bit. 
#define ISM330DHCX_ODR_G_LENGTH 4
#define ISM330DHCX_FS_G_BIT 3 // gyro chain full-scale selection start bit
#define ISM330DHCX_FS_G_LENGTH 4

#define ISM330DHCX_ODR_G_POWERDOWN 0b0000
#define ISM330DHCX_ODR_G_12_5 0b0001
#define ISM330DHCX_ODR_G_26 0b0010
#define ISM330DHCX_ODR_G_52 0b0011
#define ISM330DHCX_ODR_G_104 0b0100
#define ISM330DHCX_ODR_G_208 0b0101
#define ISM330DHCX_ODR_G_416 0b0110
#define ISM330DHCX_ODR_G_833 0b0111
#define ISM330DHCX_ODR_G_1666 0b1000
#define ISM330DHCX_ODR_G_3333 0b1001
#define ISM330DHCX_ODR_G_6667 0b1010

#define ISM330DHCX_FS_G_125DPS 0b0010
#define ISM330DHCX_FS_G_250DPS 0b0000
#define ISM330DHCX_FS_G_500DPS 0b0100
#define ISM330DHCX_FS_G_1000DPS 0b1000
#define ISM330DHCX_FS_G_2000DPS 0b1100
#define ISM330DHCX_FS_G_4000DPS 0b0001

// CTRL3_C
#define ISM330DHCX_BOOT_BIT 7 // Reboots memory content. Default value: 0(0: normal mode; 1: reboot memory content)Note: the accelerometer must be ON. This bit is automatically cleared.
#define ISM330DHCX_BDU_BIT 6 // Block Data Update. Default value: 0(0: continuous update; 1: output registers are not updated until MSB and LSB have been read)
#define ISM330DHCX_H_LACTIVE_BIT 5 // Interrupt activation level. Default value: 0(0: interrupt output pins active high; 1: interrupt output pins active low
#define ISM330DHCX_PP_OD_BIT 4 // Push-pull/open-drain selection on INT1 and INT2 pins. This bit must be set to '0' when H_LACTIVE is set to '1'. Default value: 0 (0: push-pull mode; 1: open-drain mode)
#define ISM330DHCX_SIM_BIT 3 // SPI Serial Interface Mode selection. Default value: 0 (0: 4-wire interface; 1: 3-wire interface)
#define ISM330DHCX_IF_INC_BIT 2 // Register address automatically incremented during a multiple byte access with a serial interface (I²C or SPI). Default value: 1 (0: disabled; 1: enabled)
#define ISM330DHCX_SW_RESET_BIT 0 // Software reset. Default value: 0 (0: normal mode; 1: reset device) This bit is automatically cleared.

// CTRL4_C
#define ISM330DHCX_SLEEP_G_BIT 6 // Enables gyroscope Sleep mode. Default value:0 (0: disabled; 1: enabled)
#define ISM330DHCX_INT2_on_INT1_BIT 5 // All interrupt signals available on INT1 pin enable. Default value: 0 (0: interrupt signals divided between INT1 and INT2 pins;1: all interrupt signals in logic or on INT1 pin)
#define ISM330DHCX_DRDY_MASK_BIT 3 // Enables data available (0: disabled; 1: mask DRDY on pin (both XL & Gyro) until filter settling ends (XL and Gyro independently masked).
#define ISM330DHCX_I2C_disable_BIT 2 // Disables I²C interface. Default value: 0(0: SPI, I²C interfaces enabled (default); 1: I²C interface disabled)
#define ISM330DHCX_LPF1_SEL_G_BIT 1 // Enables gyroscope digital LPF1; the bandwidth can be selected through FTYPE[2:0] in CTRL6_C (15h). (0: disabled; 1: enabled)

// CTRL5_C
#define ISM330DHCX_ROUNDING_BIT 5 // Circular burst-mode (rounding) read of the output registers - starting bit
#define ISM330DHCX_ROUNDING_LENGTH 2
#define ISM330DHCX_ST_G_BIT 2 //  Angular rate sensor self-test enable. Default value: 00 (00: Self-test disabled; Other: refer to Table 53) - starting bit
#define ISM330DHCX_ST_G_LENGTH 2
#define ISM330DHCX_ST_XL_BIT 0 // Linear acceleration sensor self-test enable. Default value: 00 (00: Self-test disabled; Other: refer to Table 54) - starting bit
#define ISM330DHCX_ST_XL_LENGTH 2

#define ISM330DHCX_ROUNDING_NOROUNDING 0b00
#define ISM330DHCX_ROUNDING_XL_ONLY 0b01
#define ISM330DHCX_ROUNDING_G_ONLY 0b10
#define ISM330DHCX_ROUNDING_XL_PLUS_G 0b11

#define ISM330DHCX_ST_G_NORMAL 0b00
#define ISM330DHCX_ST_G_POS_SIGN_ST 0b01
#define ISM330DHCX_ST_G_NEG_SIGN_ST 0b11

#define ISM330DHCX_ST_XL_NORMAL 0b00
#define ISM330DHCX_ST_XL_POS_SIGN_ST 0b01
#define ISM330DHCX_ST_XL_NEG_SIGN_ST 0b10

// CTRL6_C
#define ISM330DHCX_TRIG_EN_BIT 7 // trigger mode select start bit
#define ISM330DHCX_TRIG_LENGTH 3
#define ISM330DHCX_XL_HM_MODE_BIT 4 // Disables high-performance operating mode for accelerometer. Default value: 0 (0: high-performance operating mode enabled; 1: high-performance operating mode disabled)
#define ISM330DHCX_USR_OFF_W_BIT 3 // Weight of XL user offset bits of registers X_OFS_USR (73h), Y_OFS_USR (74h), Z_OFS_USR (75h)(0 = 2-10g/LSB;1 = 2-6g/LSB)
#define ISM330DHCX_FTYPE_BIT 2 // :0]Gyroscope low-pass filter (LPF1) bandwidth selection. Table 58 shows the selectable bandwidth values. starting bit for bandwidth selection 
#define ISM330DHCX_FTYPE_LENGTH 3

#define ISM330DHCX_EDGE_SENS_TRIG 0b100 // Edge-sensitive trigger mode is selected
#define ISM330DHCX_LVL_SENS_TRIG 0b010 // Level-sensitive trigger mode is selected
#define ISM330DHCX_LVL_SENS_LATCHED 0b011 // Level-sensitive latched mode is selected
#define ISM330DHCX_LVL_SENS_FIFO 0b110 // Level-sensitive FIFO enable mode is selected

                                     // @| 12.5Hz |  26Hz  |  52Hz  | 104Hz | 208Hz | 416Hz | 833Hz | 1.67kHz | 3.33kHz | 6.67kHz |
                                     //  ========================================================================================== 
#define ISM330DHCX_LPF1_BAND_0 0b000 //  |  4.3   |  8.3   |  16.7  |  33   |  67   |  133  |  222  |   274   |   292   |   297   |
#define ISM330DHCX_LPF1_BAND_1 0b001 //  |  4.3   |  8.3   |  16.7  |  33   |  67   |  128  |  186  |   212   |   220   |   223   |
#define ISM330DHCX_LPF1_BAND_2 0b010 //  |  4.3   |  8.3   |  16.7  |  33   |  67   |  112  |  140  |   150   |   153   |   154   |
#define ISM330DHCX_LPF1_BAND_3 0b011 //  |  4.3   |  8.3   |  16.7  |  33   |  67   |  134  |  260  |   390   |   451   |   470   |
#define ISM330DHCX_LPF1_BAND_4 0b100 //  |  4.3   |  8.3   |  16.7  |  34   |  62   |  86   |  96   |   90    |======= N/A =======|      
#define ISM330DHCX_LPF1_BAND_5 0b101 //  |  4.3   |  8.3   |  16.9  |  31   |  43   |  48   |  49   |   50    |======= N/A =======|
#define ISM330DHCX_LPF1_BAND_6 0b110 //  |  4.3   |  8.3   |  13.4  |  19   |  23   |  24.6 |  25   |   25    |======= N/A =======|
#define ISM330DHCX_LPF1_BAND_7 0b111 //  |  4.3   |  8.3   |  11.6  | 11.6  | 12.2  |  12.4 |  12.6 |   12.6  |======= N/A =======|

// CTRL7_C
#define ISM330DHCX_G_HM_MODE_BIT 7 // Disables high-performance operating mode for gyroscope. Default: 0 (0: high-performance operating mode enabled; 1: high-performance operating mode disabled)
#define ISM330DHCX_HP_EN_G_BIT 6 // Enables gyroscope digital high-pass filter. The filter is enabled only if the gyro is in HP mode. Default value: 0 (0: HPF disabled; 1: HPF enabled)
#define ISM330DHCX_HPM_G_BIT 5 // [1:0]Gyroscope digital HP filter cutoff selection. Default: 00 (00 = 16 mHz; 01 = 65 mHz;10 = 260 mHz;11 = 1.04 Hz)
#define ISM330DHCX_HPM_G_LENGTH 2
#define ISM330DHCX_OIS_ON_EN_BIT 2 // (1)Selects how to enable and disable the OIS chain, after first configuration and enabling through SPI2.(0: OIS chain is enabled/disabled with SPI2 interface;1: OIS chain is enabled/disabled with primary interface)
#define ISM330DHCX_USR_OFF_ON_OUT_BIT 1 // Enables accelerometer user offset correction block; it's valid for the low-pass path - see Figure 16. Accelerometer composite filter. Default value: 0(0: accelerometer user offset correction block bypassed;1: accelerometer user offset correction block enabled)
#define ISM330DHCX_OIS_ON_BIT = 0 //Enables/disables the OIS chain from primary interface when the OIS_ON_EN bit is '1'.(0: OIS disabled; 1: OIS enabled)

#define ISM330DHCX_HPM_CUTOFF_16 0b00   // mHz
#define ISM330DHCX_HPM_CUTOFF_65 0b01   // mHz
#define ISM330DHCX_HPM_CUTOFF_260 0b10  // mHz
#define ISM330DHCX_HPM_CUTOFF_1_04 0b11 // Hz

// CTRL8_XL
#define ISM330DHCX_HPCF_XL_BIT 7 // Accelerometer LPF2 and HP filter configuration and cutoff setting - start bit. Refer to Table 61.
#define ISM330DHCX_HPCF_XL_LENGTH 3
#define ISM330DHCX_HP_REF_MODE_XL_BIT 4 // Enables accelerometer high-pass filter reference mode (valid for high-pass path -HP_SLOPE_XL_EN bit must be ‘1’). Default value: 0(0: disabled, 1: enabled)
#define ISM330DHCX_FASTSETTL_MODE_XL_BIT 3, // Enables accelerometer LPF2 and HPF fast-settling mode. The filter sets the second samples afterwriting this bit. Active only during device exit from power- down mode. Default value: 0(0: disabled, 1: enabled)
#define ISM330DHCX_HP_SLOPE_XL_EN_BIT 2 // Accelerometer slope filter / high-pass filter selection. Refer to Figure 23
#define ISM330DHCX_LOW_PASS_ON_6D_BIT 0 // LPF2 on 6D function selection. Refer to Figure 23. Default value: 0 (0: ODR/2 low-pass filtered data sent to 6D interrupt function;1: LPF2 output data sent to 6D interrupt function)

#define ISM330DHCX_HPCF_XL_ODR_DIV_4 0b000
#define ISM330DHCX_HPCF_XL_ODR_DIV_10 0b001
#define ISM330DHCX_HPCF_XL_ODR_DIV_20 0b010
#define ISM330DHCX_HPCF_XL_ODR_DIV_45 0b011
#define ISM330DHCX_HPCF_XL_ODR_DIV_100 0b100
#define ISM330DHCX_HPCF_XL_ODR_DIV_200 0b101
#define ISM330DHCX_HPCF_XL_ODR_DIV_400 0b110
#define ISM330DHCX_HPCF_XL_ODR_DIV_800 0b111

// CTRL9_XL
#define ISM330DHCX_DEN_X_BIT 7 // DEN value stored in LSB of X-axis. Default value: 1 (0: DEN not stored in X-axis LSB; 1: DEN stored in X-axis LSB)
#define ISM330DHCX_DEN_Y_BIT 6 // DEN value stored in LSB of Y-axis. Default value: 1 (0: DEN not stored in Y-axis LSB; 1: DEN stored in Y-axis LSB)
#define ISM330DHCX_DEN_Z_BIT 5 // DEN value stored in LSB of Z-axis. Default value: 1 (0: DEN not stored in Z-axis LSB; 1: DEN stored in Z-axis LSB)
#define ISM330DHCX_DEN_XL_G_BIT 4 // DEN stamping sensor selection. Default value: 0 (0: DEN pin info stamped in the gyroscope axis selected by bits [7:5];1: DEN pin info stamped in the accelerometer axis selected by bits [7:5])
#define ISM330DHCX_DEN_XL_EN_BIT 3 // Extends DEN functionality to accelerometer sensor. Default value: 0(0: disabled; 1: enabled)
#define ISM330DHCX_DEN_LH_BIT 2 // DEN active level configuration. Default value: 0 (0: active low; 1: active high)
#define ISM330DHCX_DEVICE_CONF_BIT 1 // Enables the proper device configuration. Default value: 0 (0: default; 1: enabled) It is recommended to always set this bit to 1 during device configuration.

// CTRL10_C
#define ISM330DHCX_TIMESTAMP_EN_BIT 5 // Enables timestamp counter. Default value: 0 (0: disabled; 1: enabled) The counter is readable in TIMESTAMP0 (40h), TIMESTAMP1 (41h), TIMESTAMP2 (42h), andTIMESTAMP3 (43h).

// ALL_INT_SRC
#define ISM330DHCX_TIMESTAMP_ENDCOUNT_BIT 7 // Alerts timestamp overflow within 6.4 ms
#define ISM330DHCX_AIS_SLEEP_CHANGE_IA_BIT 5 // for ALL_INT_SRC; Detects change event in activity/inactivity status. Default value: 0 (0: change status not detected; 1: change status detected)
#define ISM330DHCX_AIS_D6D_IA_BIT 4 // Interrupt active for change in position of portrait, landscape, face-up, face-down. Default value: 0(0: change in position not detected; 1: change in position detected)
#define ISM330DHCX_AIS_DOUBLE_TAP_BIT 3 // Double-tap event status. Default value: 0 (0:event not detected, 1: event detected)
#define ISM330DHCX_AIS_SINGLE_TAP_BIT 2 // Single-tap event status. Default value:0 (0: event not detected, 1: event detected)
#define ISM330DHCX_AIS_WU_IA_BIT 1 // Wake-up event status. Default value: 0 (0: event not detected, 1: event detected)
#define ISM330DHCX_AIS_FF_IA_BIT 0 // Free-fall event status. Default value: 0 (0: event not detected, 1: event detected)

// WAKE_UP_SRC
#define ISM330DHCX_WUS_SLEEP_CHANGE_IA_BIT 6 // for WAKE_UP_SRS; Detects change event in activity/inactivity status. Default value: 0 (0: change status not detected; 1: change status detected)
#define ISM330DHCX_WUS_FF_IA_BIT 5 // Free fall event detection status. Default: 0(0: free-fall event not detected; 1: free-fall event detected)
#define ISM330DHCX_SLEEP_STATE_BIT 4 // Sleep event status. Default value: 0 (0: sleep event not detected; 1: sleep event detected)
#define ISM330DHCX_WUS_WU_IA_BIT 3 // Wakeup event detection status. Default value: 0 (0: wakeup event not detected; 1: wakeup event detected.)
#define ISM330DHCX_X_WU_BIT 2 // Wakeup event detection status on X-axis. Default value: 0 (0: wakeup event on X-axis not detected; 1: wakeup event on X-axis detected)
#define ISM330DHCX_Y_WU_BIT 1 // Wakeup event detection status on Y-axis. Default value: 0 (0: wakeup event on Y-axis not detected; 1: wakeup event on Y-axis detected)
#define ISM330DHCX_Z_WU_BIT 0 // Wakeup event detection status on Z-axis. Default value: 0(0: wakeup event on Z-axis not detected; 1: wakeup event on Z-axis detected)

// TAP_SRC
#define ISM330DHCX_TAP_IA_BIT 6 // Tap event detection status. Default: 0 (0: tap event not detected; 1: tap event detected)
#define ISM330DHCX_TS_SINGLE_TAP_BIT 5 // Single-tap event status. Default value: 0 (0: single tap event not detected; 1: single tap event detected)
#define ISM330DHCX_TS_DOUBLE_TAP_BIT 4 // Double-tap event detection status. Default value: 0 (0: double-tap event not detected; 1: double-tap event detected.)
#define ISM330DHCX_TAP_SIGN_BIT 3 // Sign of acceleration detected by tap event. Default: 0 (0: positive sign of acceleration detected by tap event; 1: negative sign of acceleration detected by tap event)
#define ISM330DHCX_X_TAP_BIT 2 // Tap event detection status on X-axis. Default value: 0 (0: tap event on X-axis not detected; 1: tap event on X-axis detected)
#define ISM330DHCX_Y_TAP_BIT 1 // Tap event detection status on Y-axis. Default value: 0 (0: tap event on Y-axis not detected; 1: tap event on Y-axis detected)
#define ISM330DHCX_Z_TAP_BIT 0 // Tap event detection status on Z-axis. Default value: 0 (0: tap event on Z-axis not detected; 1: tap event on Z-axis detected)

// DRD_SRC
#define ISM330DHCX_DEN_DRDY_BIT 7 // DEN data-ready signal. It is set high when data output is related to the data coming from a DEN active condition. The DEN data-ready signal can be latched or pulsed depending on the value of the dataready_pulsed bit of theCOUNTER_BDR_REG1 (0Bh) register. 
#define ISM330DHCX_DS_D6D_IA_BIT 6 // Interrupt active for change position portrait, landscape, face-up, face-down. Default value: 0 (0: change position not detected; 1: change position detected)
#define ISM330DHCX_ZH_BIT 5 // Z-axis high event (over threshold). Default value: 0 (0: event not detected; 1: event (over threshold) detected)
#define ISM330DHCX_ZL_BIT 4 // Z-axis low event (under threshold). Default value: 0 (0: event not detected; 1: event (under threshold) detected)
#define ISM330DHCX_YH_BIT 3 // Y-axis high event (over threshold). Default value: 0 (0: event not detected; 1: event (over-threshold) detected)
#define ISM330DHCX_YL_BIT 2 // Y-axis low event (under threshold). Default value: 0 (0: event not detected; 1: event (under threshold) detected)
#define ISM330DHCX_XH_BIT 1 // X-axis high event (over threshold). Default value: 0 (0: event not detected; 1: event (over threshold) detected)
#define ISM330DHCX_XL_BIT 0 // X-axis low event (under threshold). Default value: 0(0: event not detected; 1: event (under threshold) detected)  

// STATUS_REG (1Eh) / STATUS_SPIAux (1Eh)
#define ISM330DHCX_TDA_BIT 2 // Temperature new data available. Default: 0 (0: no set of data is available at temperature sensor output; 1: a new set of data is available at temperature sensor output)
#define ISM330DHCX_GDA_BIT 1 // Gyroscope new data available. Default value: 0 (0: no set of data available at gyroscope output; 1: a new set of data is available at gyroscope output)
#define ISM330DHCX_XLDA_BIT 0 // Accelerometer new data available. Default value: 0(0: no set of data available at accelerometer output;1: a new set of data is available at accelerometer output)

#define ISM330DHCX_GYRO_SETTLING_BIT 2 // High when the gyroscope output is in the settling phase
#define ISM330DHCX_GDA_BIT 1 // Gyroscope data available (reset when one of the high parts of the output data is read)
#define ISM330DHCX_XLDA_BIT 0 // Accelerometer data available (reset when one of the high parts of the output data is read)

// EMB_FUNC_STATUS_MAINPAGE
#define ISM330DHCX_IS_FSM_LC_BIT 7
#define ISM330DHCX_S_SIGMOT_BIT 5
#define ISM330DHCX_IS_TILT_BIT 4
#define ISM330DHCX_S_STEP_DET_BIT 3

// FSM_STATUS_A_MAINPAGE
#define ISM330DHCX_IS_FSM8_BIT 7
#define ISM330DHCX_IS_FSM7_BIT 6
#define ISM330DHCX_IS_FSM6_BIT 5
#define ISM330DHCX_IS_FSM5_BIT 4
#define ISM330DHCX_IS_FSM4_BIT 3
#define ISM330DHCX_IS_FSM3_BIT 2
#define ISM330DHCX_IS_FSM2_BIT 1
#define ISM330DHCX_IS_FSM1_BIT 0

// FSM_STATUS_B_MAINPAGE
#define ISM330DHCX_IS_FSM16_BIT 7
#define ISM330DHCX_IS_FSM15_BIT 6
#define ISM330DHCX_IS_FSM14_BIT 5
#define ISM330DHCX_IS_FSM13_BIT 4
#define ISM330DHCX_IS_FSM12_BIT 3
#define ISM330DHCX_IS_FSM11_BIT 2
#define ISM330DHCX_IS_FSM10_BIT 1
#define ISM330DHCX_IS_FSM9_BIT 0

// MLC_STATUS_MAINPAGE
#define ISM330DHCX_IS_MLC8_BIT 7
#define ISM330DHCX_IS_MLC7_BIT 6
#define ISM330DHCX_IS_MLC6_BIT 5
#define ISM330DHCX_IS_MLC5_BIT 4
#define ISM330DHCX_IS_MLC4_BIT 3
#define ISM330DHCX_IS_MLC3_BIT 2
#define ISM330DHCX_IS_MLC2_BIT 1
#define ISM330DHCX_IS_MLC1_BIT 0

// STATUS_MASTER_MAINPAGE
#define ISM330DHCX_WR_ONCE_DONE_BIT 7
#define ISM330DHCX_SLAVE3_NACK_BIT 6
#define ISM330DHCX_SLAVE2_NACK_BIT 5
#define ISM330DHCX_SLAVE1_NACK_BIT 4
#define ISM330DHCX_SLAVE0_NACK_BIT 3
#define ISM330DHCX_SENS_HUB_ENDOP_BIT 0

// FIFO_STATUS2
#define ISM330DHCX_FIFO_WTM_IA_BIT 7
#define ISM330DHCX_FIFO_OVR_IA_BIT 6
#define ISM330DHCX_FIFO_FULL_IA_BIT 5
#define ISM330DHCX_COUNTER_BDR_IA_BIT 4
#define ISM330DHCX_FIFO_OVR_LATCHED_BIT 3
#define ISM330DHCX_DIFF_FIFO_9_BIT 1
#define ISM330DHCX_DIFF_FIFO_8_BIT 0

// TAP_CFG0
#define ISM330DHCX_INT_CLR_ON_READ_BIT 6
#define ISM330DHCX_SLEEP_STATUS_ON_INT_BIT 5
#define ISM330DHCX_SLOPE_FDS_BIT 4
#define ISM330DHCX_TAP_X_EN_BIT 3
#define ISM330DHCX_TAP_Y_EN_BIT 2
#define ISM330DHCX_TAP_Z_EN_BIT 1
#define ISM330DHCX_LIR_BIT 0

// TAP_CFG1
#define ISM330DHCX_TAP_PRIORITY_BIT 7
#define ISM330DHCX_TAP_PRIORITY_LENGTH 3
#define ISM330DHCX_TAP_THS_X_BIT 4
#define ISM330DHCX_TAP_THS_X_LENGTH 5

#define ISM330DHCX_TAP_PRIORITY1 0b000
#define ISM330DHCX_TAP_PRIORITY2 0b001
#define ISM330DHCX_TAP_PRIORITY3 0b010
#define ISM330DHCX_TAP_PRIORITY4 0b011
#define ISM330DHCX_TAP_PRIORITY5 0b100
#define ISM330DHCX_TAP_PRIORITY6 0b101
#define ISM330DHCX_TAP_PRIORITY7 0b110
#define ISM330DHCX_TAP_PRIORITY8 0b111

// TAP_CFG2
#define ISM330DHCX_INTERRUPTS_ENABLE_BIT 7
#define ISM330DHCX_INACT_EN_BIT 6
#define ISM330DHCX_INACT_EN_LENGTH 2
#define ISM330DHCX_TAP_THS_Y_BIT 4
#define ISM330DHCX_TAP_THS_Y_LENGTH 5

#define ISM330DHCX_INACT_EN_DEFAULT 0b00
#define ISM330DHCX_INACT_EN_XL_ODR_12_5_G_NOCHANGE 0b01
#define ISM330DHCX_INACT_EN_XL_ODR_12_5_G_SLEEP 0b10
#define ISM330DHCX_INACT_EN_XL_ODR_12_5_G_LOWPOWER 0b11

// TAP_THS_6D
#define ISM330DHCX_D4D_EN_BIT 7
#define ISM330DHCX_SIXD_THS_BIT 6
#define ISM330DHCX_SIXD_THS_LENGTH 2
#define ISM330DHCX_TAP_THS_Z_BIT 4
#define ISM330DHCX_TAP_THS_Z_LENGTH 5

#define ISM330DHCX_SIXD_THS_80DEG 0b00 //default
#define ISM330DHCX_SIXD_THS_70DEG 0b01
#define ISM330DHCX_SIXD_THS_60DEG 0b10
#define ISM330DHCX_SIXD_THS_50DEG 0b11

// INT_DUR2
#define ISM330DHCX_DUR_BIT 7
#define ISM330DHCX_DUR_LENGTH 4
#define ISM330DHCX_QUIET_BIT 3
#define ISM330DHCX_QUIET_LENGTH 2
#define ISM330DHCX_SHOCK_BIT 1
#define ISM330DHCX_SHOCK_LENGTH 2

// WAKE_UP_THS
#define ISM330DHCX_SINGLE_DOUBLE_TAP_BIT 7
#define ISM330DHCX_USR_OFF_ON_WU_BIT 6
#define ISM330DHCX_WK_THS_BIT 5
#define ISM330DHCX_WK_THS_LENGTH 6

// WAKE_UP_DUR
#define ISM330DHCX_FF_DUR5_BIT 7
#define ISM330DHCX_WAKE_DUR_BIT 6
#define ISM330DHCX_WAKE_DUR_LENGTH 2
#define ISM330DHCX_WAKE_THS_W_BIT 4
#define ISM330DHCX_SLEEP_DUR_BIT 3
#define ISM330DHCX_SLEEP_DUR_LENGTH 4

// FREE_FALL
#define ISM330DHCX_FF_DUR_BIT 7
#define ISM330DHCX_FF_DUR_LENGTH 5
#define ISM330DHCX_FF_THS_BIT 2
#define ISM330DHCX_FF_THS_LENGTH 3

#define ISM330DHCX_FF_THS_156 0b000
#define ISM330DHCX_FF_THS_219 0b001
#define ISM330DHCX_FF_THS_250 0b010
#define ISM330DHCX_FF_THS_312 0b011
#define ISM330DHCX_FF_THS_344 0b100
#define ISM330DHCX_FF_THS_406 0b101
#define ISM330DHCX_FF_THS_469 0b110
#define ISM330DHCX_FF_THS_500 0b111

// MD1_CFG
#define ISM330DHCX_INT1_SLEEP_CHANGE_BIT 7
#define ISM330DHCX_INT1_SINGLE_TAP_BIT 6
#define ISM330DHCX_INT1_WU_BIT 5
#define ISM330DHCX_INT1_FF_BIT 4
#define ISM330DHCX_INT1_DOUBLE_TAP_BIT 3
#define ISM330DHCX_INT1_6D_BIT 2
#define ISM330DHCX_INT1_EMB_FUNC_BIT 1
#define ISM330DHCX_INT1_SHUB_BIT 0

// MD2_CFG
#define ISM330DHCX_INT2_SLEEP_CHANGE_BIT 7
#define ISM330DHCX_INT2_SINGLE_TAP_BIT 6
#define ISM330DHCX_INT2_WU_BIT 5
#define ISM330DHCX_INT2_FF_BIT 4
#define ISM330DHCX_INT2_DOUBLE_TAP_BIT 3
#define ISM330DHCX_INT2_6D_BIT 2
#define ISM330DHCX_INT2_EMB_FUNC_BIT 1
#define ISM330DHCX_INT2_TIMESTAMP_BIT 0

// INTERNAL_FREQ_FINE
#define ISM330DHCX_ODR_COEFF_AT_12_5ODR 512
#define ISM330DHCX_ODR_COEFF_AT_26ODR 256
#define ISM330DHCX_ODR_COEFF_AT_52ODR 128
#define ISM330DHCX_ODR_COEFF_AT_104ODR 64
#define ISM330DHCX_ODR_COEFF_AT_208ODR 32
#define ISM330DHCX_ODR_COEFF_AT_416ODR 16
#define ISM330DHCX_ODR_COEFF_AT_833ODR 8
#define ISM330DHCX_ODR_COEFF_AT_1667ODR 4
#define ISM330DHCX_ODR_COEFF_AT_3333ODR 2
#define ISM330DHCX_ODR_COEFF_AT_6667ODR 1

// INT_OIS
#define ISM330DHCX_INT2_DRDY_OIS_BIT 7
#define ISM330DHCX_LVL2_OIS_BIT 6
#define ISM330DHCX_DEN_LH_OIS_BIT 5
#define ISM330DHCX_ST_XL_OIS_BIT 1
#define ISM330DHCX_ST_XL_OIS_LENGTH 2

#define ISM330DHCX_ST_XL_OIS_NORMAL 0b00
#define ISM330DHCX_ST_XL_OIS_POS_ST 0b01
#define ISM330DHCX_ST_XL_OIS_NEG_ST 0b10

// CTRL1_OIS
#define ISM330DHCX_LVL1_OIS_BIT 6
#define ISM330DHCX_SIM_OIS_BIT 5
#define ISM330DHCX_Mode4_EN_BIT 4
#define ISM330DHCX_FS_G_OIS_BIT 3
#define ISM330DHCX_FS_G_OIS_LENGTH 2
#define ISM330DHCX_FS_125_OIS_BIT 1
#define ISM330DHCX_OIS_EN_SPI2_BIT 0

// CTRL2_OIS
#define ISM330DHCX_HPM_OIS_BIT 5
#define ISM330DHCX_HPM_OIS_LENGTH 2
#define ISM330DHCX_FTYPE_OIS_BIT 2
#define ISM330DHCX_FTYPE_OIS_LENGTH 2
#define ISM330DHCX_HP_EN_OIS_BIT 0

#define ISM330DHCX_LPF1_FTYPE_OIS_297 0b00
#define ISM330DHCX_LPF1_FTYPE_OIS_222 0b01
#define ISM330DHCX_LPF1_FTYPE_OIS_154 0b10
#define ISM330DHCX_LPF1_FTYPE_OIS_470 0b11

// CTRL3_OIS
#define ISM330DHCX_FS_XL_OIS_BIT 7
#define ISM330DHCX_FS_XL_OIS_LENGTH 2
#define ISM330DHCX_FILTER_XL_CONF_OIS_BIT 5
#define ISM330DHCX_FILTER_XL_CONF_OIS_LENGTH 3
#define ISM330DHCX_ST_OIS_BIT 2
#define ISM330DHCX_ST_OIS_LENGTH
#define ISM330DHCX_ST_OIS_CLAMPDIS_BIT 0

#define ISM330DHCX_FS_XL_OIS_2G 0b00
#define ISM330DHCX_FS_XL_OIS_16G 0b01
#define ISM330DHCX_FS_XL_OIS_4G 0b11
#define ISM330DHCX_FS_XL_OIS_8G 0b11

#define ISM330DHCX_FILTER_XL_CONF_OIS1 0b000
#define ISM330DHCX_FILTER_XL_CONF_OIS2 0b001
#define ISM330DHCX_FILTER_XL_CONF_OIS3 0b010
#define ISM330DHCX_FILTER_XL_CONF_OIS4 0b011
#define ISM330DHCX_FILTER_XL_CONF_OIS5 0b100
#define ISM330DHCX_FILTER_XL_CONF_OIS6 0b101
#define ISM330DHCX_FILTER_XL_CONF_OIS7 0b110
#define ISM330DHCX_FILTER_XL_CONF_OIS8 0b111

#define ISM330DHCX_ST_OIS_NORMAL 0b00 
#define ISM330DHCX_ST_OIS_POS_ST 0b01
#define ISM330DHCX_ST_OIS_NEG_ST 0b11

// FIFO_DATA_OUT_TAG
#define ISM330DHCX_TAG_SENSOR_BIT 7
#define ISM330DHCX_TAG_SENSOR_LENGTH 5
#define ISM330DHCX_TAG_CNT_BIT 2
#define ISM330DHCX_TAG_CNT_LENGTH 2
#define ISM330DHCX_TAG_PARITY_BIT 0

#define ISM330DHCX_TAG_SENSOR_GYRO_NC 0x01
#define ISM330DHCX_TAG_SENSOR_ACCEL_NC 0x02
#define ISM330DHCX_TAG_SENSOR_TEMP 0x03
#define ISM330DHCX_TAG_SENSOR_TIMESTAMP 0x04
#define ISM330DHCX_TAG_SENSOR_CFG_CHANGE 0x05
#define ISM330DHCX_TAG_SENSOR_ACCEL_NC_T_2 0x06
#define ISM330DHCX_TAG_SENSOR_ACCEL_NC_T_1 0x07
#define ISM330DHCX_TAG_SENSOR_ACCEL_2xC 0x08
#define ISM330DHCX_TAG_SENSOR_ACCEL_3xC 0x09
#define ISM330DHCX_TAG_SENSOR_GYRO_NC_T_2 0x0A
#define ISM330DHCX_TAG_SENSOR_GYRO_NC_T_1 0x0B
#define ISM330DHCX_TAG_SENSOR_GYRO_2xC 0x0C
#define ISM330DHCX_TAG_SENSOR_GYRO_3xC 0x0D
#define ISM330DHCX_TAG_SENSOR_SENS_HUB_SLAVE_0 0x0E
#define ISM330DHCX_TAG_SENSOR_SENS_HUB_SLAVE_1 0x0F
#define ISM330DHCX_TAG_SENSOR_SENS_HUB_SLAVE_2 0x10
#define ISM330DHCX_TAG_SENSOR_SENS_HUB_SLAVE_3 0x11
#define ISM330DHCX_TAG_SENSOR_STEP_COUNTER 0x12
#define ISM330DHCX_TAG_SENSOR_SENS_HUB_NACK 0x19

//=========================
/*
    TODO: 
    add embedded functions registers
    add embedd advanced features registers

    note: DEN = data enabled
*/
//========================



/*
#define ISM330DHCX__BIT 7
#define ISM330DHCX__BIT 6
#define ISM330DHCX__BIT 5
#define ISM330DHCX__BIT 4
#define ISM330DHCX__BIT 3
#define ISM330DHCX__BIT 2
#define ISM330DHCX__BIT 1
#define ISM330DHCX__BIT 0
#define ISM330DHCX__LENGTH
*/

class ISM330DHCX {
    public:
      ISM330DHCX();
      ISM330DHCX(uint8_t address);

      void initialize();
      bool testConnection();

      //FUNC_CFG_ACCESS register, r/w
      // =========================================================
      void setEmbeddedFunctionsConfigurationEnabled(bool enabled);
      bool getEmbeddedFunctionsConfigurationEnabled();

      void setSensorHubRegistersEnabled(bool enabled);
      bool getSensorHubRegistersEnabled();
      // =========================================================

      //PIN_CTRL register, r/w
      // =========================================================
      void setOCSAuxAndSDOAuxPinsPullupDisabled(bool disabled);
      bool getOCSAuxAndSDOAuxPinsPullupDisabled();

      void setSDOPinPullupDisabled(bool disabled);
      bool getSDOPinPullupDisabled();
      // =========================================================

      // FIFO_CTRL1 register, r/w
      // datasheet mentions this register + bit 0 of FIFO_CTRL2 is
      // the FIFO watermark threshold. not sure if I need to include
      // a function here or include it under FIFO_CTRL2

      // FIFO_CTRL2 register, r/w
      // =========================================================
      void setFIFOWatermarkEnabled(bool enabled);
      bool getFIFOWatermarkEnabled();

      void setFIFOCompressionAlgorithmEnabled(bool enabled);
      bool getFIFOCompressionAlgorithmEnabled();

      void setFIFOODRChangeVirtualSensorEnabled(bool enabled);
      bool getFIFOODRChangeVirtualSensorEnabled();

      void setFIFONonCompressedDataWriteRate(uint8_t rate);
      uint8_t getFIFONonCompressedDataWriteRate();

      void setFIFOWatermarkThreshold(uint16_t threshold);
      uint16_t getFIFOWatermarkThreshold();
      // =========================================================

      // FIFO_CTRL3, r/w
      // =========================================================
      void setGyroFIFOBatchDataRate(uint8_t rate);
      uint8_t getGyroFIFOBatchDataRate();

      void setAccelFIFOBatchDataRate(uint8_t rate);
      uint8_t getAccelFIFOBatchDataRate();
      // =========================================================

      // FIFO_CTRL4, r/w
      // =========================================================
      void setFIFOTimestampBatchingDecimation(uint8_t decimation);
      uint8_t getFIFOTimestampBatchingDecimation();

      void setFIFOTemperatureBatchDataRate(uint8_t rate);
      uint8_t getFIFOTemperatureBatchDataRate();

      void setFIFOMode(uint8_t mode);
      uint8_t getFIFOMode();
      // =========================================================

      // COUNTER_BDR_REG1, r/w
      // =========================================================
      void setPulsedDataReadyModeEnabled(bool enabled);
      bool getPulsedDataReadyModeEnabled();

      void setInternalCounterForBatchEventsReset(bool reset);

      void setInternalBatchEventCounterTrigger(bool trigger);
      bool getInternalBatchEventCounterTrigger();

      void setInternalBatchEventCounterThreshold(uint16_t threshold);
      uint16_t getInternalBatchEventCounterThreshold();
      // =========================================================

      // INT1_CTRL, r/w
      // =========================================================
      void setDENDRDYflagInteruptOnINT1Enabled(bool enabled);
      bool getDENDRDYflagInteruptOnINT1Enabled();

      void setCOUNTERBDRIAInteruptOnINT1Enabled(bool enabled);
      bool getCOUNTERBDRIAInteruptOnINT1Enabled();

      void setFIFOFullFlagInteruptOnINT1Enabled(bool enabled);
      bool getFIFOFullFlagInteruptOnINT1Enabled();

      void setFIFOOverrunInteruptOnINT1Enabled(bool enabled);
      bool getFIFOOverrunInteruptOnINT1Enabled();

      void setFIFOThresholdInteruptOnINT1Enabled(bool enabled);
      bool getFIFOThresholdInteruptOnINT1Enabled();

      void setBOOTStatusOnINT1Enabled(bool enabled);
      bool getBOOTStatusOnINT1Enabled();

      void setGyroDataReadyInteruptOnINT1Enabled(bool enabled);
      bool getGyroDataReadyInteruptOnINT1Enabled();

      void setAccelDataReadyInteruptOnINT1Enabled(bool enabled);
      bool getAccelDataReadyInteruptOnINT1Enabled();
      // =========================================================

      // INT2_CTRL, r/w
      // =========================================================
      void setCOUNTERBDRIAInteruptOnINT2Enabled(bool enabled);
      bool getCOUNTERBDRIAInteruptOnINT2Enabled();

      void setFIFOFullFlagInteruptOnINT2Enabled(bool enabled);
      bool getFIFOFullFlagInteruptOnINT2Enabled();

      void setFIFOOverrunInteruptOnINT2Enabled(bool enabled);
      bool getFIFOOverrunInteruptOnINT2Enabled();

      void setFIFOThresholdInteruptOnINT2Enabled(bool enabled);
      bool getFIFOThresholdInteruptOnINT2Enabled();

      void setTempDataReadyInteruptOnINT2Enabled(bool enabled);
      bool getTempDataReadyInteruptOnINT2Enabled();

      void setGyroDataReadyInteruptOnINT2Enabled(bool enabled);
      bool getGyroDataReadyInteruptOnINT2Enabled();

      void setAccelDataReadyInteruptOnINT2Enabled(bool enabled);
      bool getAccelDataReadyInteruptOnINT2Enabled();
      // =========================================================

      // WHO_AM_I register, read-only
      // =========================================================
      uint8_t getDeviceID();
      // =========================================================

      // CTRL1_XL, r/w
      // =========================================================
      void setAccelOutputDataRate(uint8_t rate);
      uint8_t getAccelOutputDataRate();

      void setAccelFullScale(uint8_t scale);
      uint8_t getAccelFullScale();

      void setAccelLowPassFilter2Enabled(bool enabled);
      bool getAccelLowPassFilter2Enabled();
      // =========================================================

      // CTRL2_G, r/w
      // =========================================================
      void setGyroOutputDataRate(uint8_t rate);
      uint8_t getGyroOutputDataRate();

      void setGyroFullScale(uint8_t scale);
      uint8_t getGyroFullScale();
      // =========================================================

      // CTRL3_C, r/w
      // =========================================================
      void RebootMemoryContent(bool reboot);

      void setBlockDataUpdate(bool block);
      bool getBlockDataUpdate();

      void setInteruptActivationLevel(bool level);
      bool getInteruptActivationLevel();

      void setPushPullOrOpenDrainOnINT1AndINT2Selection(bool selection);
      bool getPushPullOrOpenDrainOnINT1AndINT2Selection();

      void setSPISerialInterfaceMode(bool mode);
      bool getSPISerialInterfaceMode();

      void setAutoRegisterIncrementOnMultiByteReadEnabled(bool enabled);
      bool getAutoRegisterIncrementOnMultiByteReadEnabled();

      void SoftwareReset(bool reset); 
      // =========================================================

      // CTRL4_C, r/w
      // =========================================================
      void setGyroSleepModeEnabled(bool enabled);
      bool getGyroSleepModeEnabled();

      void setAllInteruptSignalsOnINT1Enabled(bool enabled);
      bool getAllInteruptSignalsOnINT1Enabled();

      void setDRDYOnPinMask(bool mask);
      bool getDRDYOnPinMask();

      void seti2cDisabled(bool disabled);
      bool geti2cDisabled();

      void setGyroLowPassFilter1Enabled(bool enabled);
      bool getGyroLowPassFilter1Enabled();
      // =========================================================

      // CTRL5_C, r/w
      // =========================================================
      void setRoundingFunctionsMode(uint8_t mode);
      uint8_t getRoundingFunctionsMode();

      void setAccelSelfTestMode(uint8_t mode);
      uint8_t getAccelSelfTestMode();

      void setGyroSelfTestMode(uint8_t mode);
      uint8_t getGyroSelfTestMode();
      // =========================================================

      // CTRL6_C, r/w
      // =========================================================
      void setDENTriggerMode(uint8_t mode);
      uint8_t getDDENTriggerMode();

      void setAccelHighPerformanceModeDisabled(bool disabled);
      bool getAccelHighPerformanceModeDisabled();

      void setAccelUserOffsetBitWeight(bool weight);
      bool getAccelUserOffsetBitWeight();

      void setGyroLowPassFilter1Bandwidth(uint8_t bandwidth);
      uint8_t getGyroLowPassFilter1Bandwidth();
      // =========================================================

      // CTRL7_G, r/w
      // =========================================================
      void setGyroHighPerformanceModeDisabled(bool disabled);
      bool getGyroHighPerformanceModeDisabled();

      void setGyroHighPassFilterEnabled(bool enabled);
      bool getGyroHighPassFilterEnabled();

      void setGyroHighPassFilterCutoffBandwidth(uint8_t bandwidth);
      uint8_t getGyroHighPassFilterCutoffBandwidth();

      void setOISChainEnableOrDisableInterface(bool primary);
      bool getsetOISChainEnableOrDisableInterface();

      void setAccelUserOffsetCorrectionBlockEnabled(bool enabled);
      bool getAccelUserOffsetCorrectionBlockEnabled();

      void setOISChainEnabled(bool enabled);
      bool getOISChainEnabled();
      // =========================================================

      // CTRL8_XL, r/w
      // =========================================================
      void setAccelLPF2AndHighPassFilterConfigAndCutoff(bool highpass, bool LPF2enabled, uint8_t bandwidth);
      uint8_t getAccelLPF2AndHighPassFilterConfigAndCutoff();

      void setAccelHighPassFilterReferenceModeEnabled(bool enabled);
      bool getAccelHighPassFilterReferenceModeEnabled();

      void setAccelLPF2AndHighPassFilterFastSettlingModeEnabled(bool enabled);
      bool getAccelLPF2AndHighPassFilterFastSettlingModeEnabled();

      void setAccelLPF2OutputTo6DInteruptFunctionEnabled(bool enabled);
      bool getAccelLPF2OutputTo6DInteruptFunctionEnabled();
      // =========================================================

      // CTRL9_XL, r/w
      // =========================================================
      void setDENStoredinXAxisLSBDisabled(bool disabled);
      bool getDENStoredinXAxisLSBDisabled();

      void setDENStoredinYAxisLSBDisabled(bool disabled);
      bool getDENStoredinYAxisLSBDisabled();

      void setDENStoredinZAxisLSBDisabled(bool disabled);
      bool getDENStoredinZAxisLSBDisabled();

      void setDENPinInfoStampedInAccelAxisBitsEnabled(bool enabled);
      bool getDENPinInfoStampedInAccelAxisBitsEnabled();

      void setExtendDENFunctionalityToAccelSensorEnabled(bool enabled);
      bool getExtendDENFunctionalityToAccelSensorEnabled();

      void setDENActiveLow(bool active_low);
      bool getDENActiveLow();

      void setProperDeviceConfigurationEnabled(bool enabled); // datasheet says that this bit needs to be set to 1 during device config, but what does "proper" mean?
      bool getProperDeviceConfigurationEnabled();
      // =========================================================

      // CTRL10_C, r/w
      // =========================================================
      void setTimestampEnabled(bool enabled);

      bool getTimestampEnabled();
      // =========================================================

      // ALL_INT_SRC, r
      // =========================================================
      bool getALLIntSRCTimestampOverflowAlert();

      bool getALLIntSRCSleepChangeStatus();

      bool getALLIntSRCOrientationStateChange();

      bool getALLIntSRCDoubleTapEvent();

      bool getALLIntSRCSingleTapEvent();

      bool getALLIntSRCWakeUpEventStatus();

      bool getALLIntSRCFreeFallEventStatus();
      // =========================================================
      
      // WAKE_UP_SRC, r
      // =========================================================
      bool getWakeUpSRCSleepChangeStatus();

      bool getWakeUpSRCFreeFallEventStatus();

      bool getWakeUpSRCSleepEvent();

      bool getWakeUpSRCWakeUpEventStatus();

      bool getWakeUpSRCWakeUpEventXAxisStatus();

      bool getWakeUpSRCWakeUpEventYAxisStatus();

      bool getWakeUpSRCWakeUpEventZAxisStatus();
      // =========================================================

      // TAP_SRC, r
      // =========================================================
      bool getTapSRCTapEventStatus();

      bool getTapSRCCSingleTapEventStatus();

      bool getTapSRCDoubleTapEventStaus();

      bool getTapSRCTapAcclerationSign();

      bool getTapSRCTapDirectionXAxisStatus();

      bool getTapSRCTapDirectionYAxisStatus();

      bool getTapSRCTapDirectionZAxisStatus();
      // =========================================================

      // DRD_SRC, r
      // =========================================================
      bool getDRDSRCDENDataReadySignal();

      bool getDRDSRCOrientationEventStatus();

      bool getDRDSRCZAxisHighEvent();

      bool getDRDSRCZAxisLowEvent();

      bool getDRDSRCYAxisHighEvent();

      bool getDRDSRCYAxisLowEvent();

      bool getDRDSRCXAxisHighEvent();

      bool getDRDSRCXAxisLowEvent();
      // =========================================================

      // STATUS_REG / STATUS_SPIAux, r
      // =========================================================
      bool getIsNewTempDataAvailable();

      bool getIsNewGyroDataAvailable();

      bool getIsNewAccelDataAvailable();

      bool getIsGyroOutputInSettleingPhase(); // only used by Aux SPI
      // =========================================================

      // OUT_TEMP_L / OUT_TEMP_H, r
      // Temperature data output register
      // L and H registers together express a 16-bit word in two’s complement.
      // =========================================================
      int16_t getTemperature();
      // =========================================================

      // Gyroscope registers, r
      // (0x22,0x23) - (0x24,0x25) - (0x26,0x27)
      // =========================================================
      void getRotation(int16_t* x, int16_t* y, int16_t* z);
      
      int16_t getRotationX();
      
      int16_t getRotationY();
      
      int16_t getRotationZ();

      void getRawGyro(int16_t* rx, int16_t* ry, int16_t* rz);
      
      int16_t getRawGyroX();
      
      int16_t getRawGyroY();
      
      int16_t getRawGyroZ();
      // =========================================================

      // Acceleromter registers, r
      // (0x28,0x29) - (0x30,0x31) - (0x32,0x33)
      // =========================================================
      void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
      
      int16_t getAccelerationX();
      
      int16_t getAccelerationY();
      
      int16_t getAccelerationZ();

      void getRawAccel(int16_t* rx, int16_t* ry, int16_t* rz);
      
      int16_t getRawAccelX();
      
      int16_t getRawAccelY();
      
      int16_t getRawAccelZ();
      // =========================================================

      // Gyro + Accel
      // =========================================================
      void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);

      void getRawMotion6(int16_t* rax, int16_t* ray, int16_t* raz, int16_t* rgx, int16_t* rgy, int16_t* rgz);
      // =========================================================

      // Timestamp, r
      // (0x40,0x41,0x42,0x43)
      // =========================================================
      void getTimestamp(uint32_t* ts);
      // =========================================================

    private:
        uint8_t devAddr;
        uint8_t buffer[14];
};
    

#endif 