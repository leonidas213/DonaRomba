#include "pico/stdlib.h"
#include <stdio.h>
using namespace std;
#include <string.h>
#include <string>
#include "hardware/i2c.h"
#include "math.h"

#ifndef ADXL345_h
#define ADXL345_h

/* ------- Register names ------- */
#define ADXL345_DEVID 0x00
#define ADXL345_RESERVED1 0x01
#define ADXL345_THRESH_TAP 0x1d
#define ADXL345_OFSX 0x1e
#define ADXL345_OFSY 0x1f
#define ADXL345_OFSZ 0x20
#define ADXL345_DUR 0x21
#define ADXL345_LATENT 0x22
#define ADXL345_WINDOW 0x23
#define ADXL345_THRESH_ACT 0x24
#define ADXL345_THRESH_INACT 0x25
#define ADXL345_TIME_INACT 0x26
#define ADXL345_ACT_INACT_CTL 0x27
#define ADXL345_THRESH_FF 0x28
#define ADXL345_TIME_FF 0x29
#define ADXL345_TAP_AXES 0x2a
#define ADXL345_ACT_TAP_STATUS 0x2b
#define ADXL345_BW_RATE 0x2c
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_INT_ENABLE 0x2e
#define ADXL345_INT_MAP 0x2f
#define ADXL345_INT_SOURCE 0x30
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37
#define ADXL345_FIFO_CTL 0x38
#define ADXL345_FIFO_STATUS 0x39

#define ADXL345_BW_1600 0xF // 1111
#define ADXL345_BW_800 0xE  // 1110
#define ADXL345_BW_400 0xD  // 1101
#define ADXL345_BW_200 0xC  // 1100
#define ADXL345_BW_100 0xB  // 1011
#define ADXL345_BW_50 0xA   // 1010
#define ADXL345_BW_25 0x9   // 1001
#define ADXL345_BW_12 0x8   // 1000
#define ADXL345_BW_6 0x7    // 0111
#define ADXL345_BW_3 0x6    // 0110

/*
 Interrupt PINs
 INT1: 0
 INT2: 1
 */
#define ADXL345_INT1_PIN 0x00
#define ADXL345_INT2_PIN 0x01

/*Interrupt bit position*/
#define ADXL345_INT_DATA_READY_BIT 0x07
#define ADXL345_INT_SINGLE_TAP_BIT 0x06
#define ADXL345_INT_DOUBLE_TAP_BIT 0x05
#define ADXL345_INT_ACTIVITY_BIT 0x04
#define ADXL345_INT_INACTIVITY_BIT 0x03
#define ADXL345_INT_FREE_FALL_BIT 0x02
#define ADXL345_INT_WATERMARK_BIT 0x01
#define ADXL345_INT_OVERRUNY_BIT 0x00

#define ADXL345_DATA_READY 0x07
#define ADXL345_SINGLE_TAP 0x06
#define ADXL345_DOUBLE_TAP 0x05
#define ADXL345_ACTIVITY 0x04
#define ADXL345_INACTIVITY 0x03
#define ADXL345_FREE_FALL 0x02
#define ADXL345_WATERMARK 0x01
#define ADXL345_OVERRUNY 0x00

#define ADXL345_OK 1    // no error
#define ADXL345_ERROR 0 // indicates error is predent

#define ADXL345_NO_ERROR 0   // initial state
#define ADXL345_READ_ERROR 1 // problem reading accel
#define ADXL345_BAD_ARG 2    // bad method argument

class interrupt_ADXL345
{
public:
    interrupt_ADXL345();
    interrupt_ADXL345(bool data_ready, bool single_tap, bool double_tap, bool activity, bool inactivity, bool free_fall, bool watermark, bool overrun);
    bool data_ready;
    bool single_tap;
    bool double_tap;
    bool activity;
    bool inactivity;
    bool free_fall;
    bool watermark;
    bool overrun;
};

class ADXL345
{
public:
    bool status; // set when error occurs
    // see error code for details
    uint8_t error_code; // Initial state
    float gains[3];     // counts to Gs
    i2c_inst *i2CInst;
    ADXL345(i2c_inst *i2CInst);
    void begin();
    void readAccel(int *xyx);
    void readXYZ(int *x, int *y, int *z);
    void getAcceleration(float *xyz);

    void setAxisGains(float *_gains);
    void setAxisOffset(int x, int y, int z);
    void setActivityThreshold(int activityThreshold);
    void setInactivityThreshold(int inactivityThreshold);
    void setTimeInactivity(int timeInactivity);

    void setActivity(bool x, bool y, bool z, bool ac);
    void setInactivity(bool x, bool y, bool z, bool ac);

    void setSuppressBit(bool state);

    bool isAsleep();
    bool isLowPower();
    void setLowPower(bool state);
    float getRate();
    void setRate(float rate);
    void set_bw(uint8_t bw_code);
    uint8_t get_bw_code();

    interrupt_ADXL345 getInterruptSource();
    void setInterruptMapping(interrupt_ADXL345 interruptBit);
    void setInterrupt(interrupt_ADXL345 interruptBit);
    void setInterruptLevelBit(bool interruptLevelBit);
    void setFreeFall(int freeFallthreshold, int freeFallDuration);
    void setTap(bool x, bool y, bool z, int threshold, int duration, int Dlatency, int Dwindow);

    void getRangeSetting(uint8_t *rangeSetting);
    void setRangeSetting(int val);
    bool getSelfTestBit();
    void setSelfTestBit(bool selfTestBit);

    bool getFullResBit();
    void setFullResBit(bool fullResBit);
    bool getJustifyBit();
    void setJustifyBit(bool justifyBit);
    void printAllRegister();

private:
    void writeTo(uint8_t address, uint8_t val);
    void readFrom(uint8_t address, int num, uint8_t buff[]);
    void setRegisterBit(uint8_t regAdress, int bitPos, bool state);
    bool getRegisterBit(uint8_t regAdress, int bitPos);
    uint8_t _buff[6]; // 6 uint8_ts buffer for saving data read from the device
};
void print_uint8_t(uint8_t val);
#endif