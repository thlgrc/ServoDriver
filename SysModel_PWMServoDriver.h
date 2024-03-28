/*!
 *  @file Adafruit_PWMServoDriver.h
 *
 *  This is a library for our Adafruit 16-channel PWM & Servo driver.
 *
 *  Designed specifically to work with the Adafruit 16-channel PWM & Servo
 * driver.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/815
 *
 *  These driver use I2C to communicate, 2 pins are required to interface.
 *  For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include <cstdint>

/* REGISTER ADDRESSES */
#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
#define PCA9685_MODE2 0x01      /**< Mode Register 2 */
// LED ON/OFF REGISTER ADDRESS
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H 0xFB  /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_RESTART 0x80 /**< Restart enabled */

#define PCA9685_I2C_ADDRESS 0x40      /**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 26700000 /**< Int. osc. frequency in datasheet */
#define I2C_BUS "/dev/i2c-5"

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

#define SERVO_ANGLE_MAX 270 /*degrees*/
#define SERVO_ANGLE_MIN 0 /*degrees*/
#define SERVO_PULSE_MAX 2500 /*microseconds*/
#define SERVO_PULSE_MIN 500 /*microseconds*/

/*!
 *  @brief  Class that stores state and functions for interacting with PCA9685
 * PWM chip
 */

class SysModel_PWMServoDriver {
public:
    SysModel_PWMServoDriver();
    void sleep();
    void setFreq(float freq);
    void begin();
    void setAngle(int servoNum, int angle);
    void wakeup();
    /*Move these to private and test*/
    // void writeReg(uint8_t addr,uint8_t value);
    // uint8_t readReg(uint8_t addr);

private:
    uint8_t _i2caddr;
    uint32_t _oscillator_freq;
    uint8_t _freq;
    int _servonum;
    void writeReg(uint8_t addr,uint8_t value);
    uint8_t readReg(uint8_t addr);
    long map(long x, long in_min, long in_max, long out_min, long out_max); 
};
