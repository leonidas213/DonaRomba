
#include "ADXL345.hpp"

#define ADXL345_DEVICE 0x53 // ADXL345 device address
#define ADXL345_TO_READ 6   // num of uint8_ts we are going to read each time (two uint8_ts for each axis)
interrupt_ADXL345::interrupt_ADXL345(bool data_ready,bool single_tap, bool double_tap, bool activity, bool inactivity, bool free_fall, bool watermark, bool overrun)
{   this->data_ready = data_ready;
    this->single_tap = single_tap;
    this->double_tap = double_tap;
    this->activity = activity;
    this->inactivity = inactivity;
    this->free_fall = free_fall;
    this->watermark = watermark;
    this->overrun = overrun;
}
interrupt_ADXL345::interrupt_ADXL345(){

}

ADXL345::ADXL345(i2c_inst *i2CInst)
{
    status = ADXL345_OK;
    error_code = ADXL345_NO_ERROR;
    this->i2CInst = i2CInst;
    gains[0] = 0.00376390;
    gains[1] = 0.00376009;
    gains[2] = 0.00349265;
}

void ADXL345::begin()
{
    writeTo(ADXL345_POWER_CTL, 0);
    writeTo(ADXL345_POWER_CTL, 16);
    writeTo(ADXL345_POWER_CTL, 8);
}

// Reads the acceleration into three variable x, y and z
void ADXL345::readAccel(int *xyz)
{
    readXYZ(xyz, xyz + 1, xyz + 2);
}
void ADXL345::readXYZ(int *x, int *y, int *z)
{
    readFrom(ADXL345_DATAX0, ADXL345_TO_READ, _buff); // read the acceleration data from the ADXL345
    *x = (short)((((unsigned short)_buff[1]) << 8) | _buff[0]);
    *y = (short)((((unsigned short)_buff[3]) << 8) | _buff[2]);
    *z = (short)((((unsigned short)_buff[5]) << 8) | _buff[4]);
}

void ADXL345::getAcceleration(float *xyz)
{
    int i;
    int xyz_int[3];
    readAccel(xyz_int);
    for (i = 0; i < 3; i++)
    {
        xyz[i] = xyz_int[i] * gains[i];
    }
}
// Writes val to address register on device
void ADXL345::writeTo(uint8_t address, uint8_t val)
{
    uint8_t data[2] = {address, val};
    i2c_write_blocking(this->i2CInst, ADXL345_DEVICE, reinterpret_cast<const uint8_t *>(data), 2, false);
}

// Reads num uint8_ts starting from address register on device in to _buff array
void ADXL345::readFrom(uint8_t address, int num, uint8_t _buff[])
{
    uint8_t data[1] = {address};
    i2c_write_blocking(this->i2CInst, ADXL345_DEVICE, reinterpret_cast<const uint8_t *>(data), 1, true);
    i2c_read_blocking(this->i2CInst, ADXL345_DEVICE, _buff, num, false);
}

// Gets the range setting and return it into rangeSetting
// it can be 2, 4, 8 or 16
void ADXL345::getRangeSetting(uint8_t *rangeSetting)
{
    uint8_t _b;
    readFrom(ADXL345_DATA_FORMAT, 1, &_b);
    *rangeSetting = _b & 0b00000011;
}

// Sets the range setting, possible values are: 2, 4, 8, 16
void ADXL345::setRangeSetting(int val)
{
    uint8_t _s;
    uint8_t _b;

    switch (val)
    {
    case 2:
        _s = 0b00000000;
        break;
    case 4:
        _s = 0b00000001;
        break;
    case 8:
        _s = 0b00000010;
        break;
    case 16:
        _s = 0b00000011;
        break;
    default:
        _s = 0b00000000;
    }
    readFrom(ADXL345_DATA_FORMAT, 1, &_b);
    _s |= (_b & 0b11101100);
    writeTo(ADXL345_DATA_FORMAT, _s);
}

void ADXL345::setSelfTestBit(bool selfTestBit)
{
    setRegisterBit(ADXL345_DATA_FORMAT, 7, selfTestBit);
}

// Sets the INT_INVERT bit
// if set to 0 sets the interrupts to active high
// if set to 1 sets the interrupts to active low
void ADXL345::setInterruptLevelBit(bool interruptLevelBit)
{
    setRegisterBit(ADXL345_DATA_FORMAT, 5, interruptLevelBit);
}

// Sets the FULL_RES bit
// if set to 1, the device is in full resolution mode, where the output resolution increases with the
//   g range set by the range bits to maintain a 4mg/LSB scal factor
// if set to 0, the device is in 10-bit mode, and the range buts determine the maximum g range
//   and scale factor
void ADXL345::setFullResBit(bool fullResBit)
{
    setRegisterBit(ADXL345_DATA_FORMAT, 3, fullResBit);
}

// Sets the JUSTIFY bit
// if sets to 1 selects the left justified mode
// if sets to 0 selects right justified mode with sign extension
void ADXL345::setJustifyBit(bool justifyBit)
{
    setRegisterBit(ADXL345_DATA_FORMAT, 2, justifyBit);
}
template <class T>
const T &constrain(const T &x, const T &a, const T &b)
{
    if (x < a)
    {
        return a;
    }
    else if (b < x)
    {
        return b;
    }
    else
        return x;
}

void ADXL345::setTap(bool x, bool y, bool z, int threshold, int duration, int Dlatency, int Dwindow)
{ // Sets the THRESH_TAP uint8_t value
    // it should be between 0 and 255
    // the scale factor is 62.5 mg/LSB
    // A value of 0 may result in undesirable behavior
    threshold = constrain(threshold, 0, 255);
    uint8_t _b = uint8_t(threshold);
    writeTo(ADXL345_THRESH_TAP, _b);

    // Sets the DUR uint8_t
    // The DUR uint8_t contains an unsigned time value representing the maximum time
    // that an event must be above THRESH_TAP threshold to qualify as a tap event
    // The scale factor is 625µs/LSB
    // A value of 0 disables the tap/float tap funcitons. Max value is 255.
    duration = constrain(duration, 0, 255);
    _b = uint8_t(duration);
    writeTo(ADXL345_DUR, _b);

    // Sets the latency (latent register) which contains an unsigned time value
    // representing the wait time from the detection of a tap event to the start
    // of the time window, during which a possible second tap can be detected.
    // The scale factor is 1.25ms/LSB. A value of 0 disables the float tap function.
    // It accepts a maximum value of 255.
    _b = uint8_t(Dlatency);
    writeTo(ADXL345_LATENT, _b);

    // Sets the Window register, which contains an unsigned time value representing
    // the amount of time after the expiration of the latency time (Latent register)
    // during which a second valud tap can begin. The scale factor is 1.25ms/LSB. A
    // value of 0 disables the float tap function. The maximum value is 255.
    Dwindow = constrain(Dwindow, 0, 255);
    _b = uint8_t(Dwindow);
    writeTo(ADXL345_WINDOW, _b);

    _b = (x << 2) | (y << 1) | z;
    writeTo(ADXL345_TAP_AXES, _b);
}

// set/get the gain for each axis in Gs / count
void ADXL345::setAxisGains(float *_gains)
{
    int i;
    for (i = 0; i < 3; i++)
    {
        gains[i] = _gains[i];
    }
}

// Sets the OFSX, OFSY and OFSZ uint8_ts
// OFSX, OFSY and OFSZ are user offset adjustments in twos complement format with
// a scale factor of 15,6mg/LSB
// OFSX, OFSY and OFSZ should be comprised between
void ADXL345::setAxisOffset(int x, int y, int z)
{
    writeTo(ADXL345_OFSX, uint8_t(x));
    writeTo(ADXL345_OFSY, uint8_t(y));
    writeTo(ADXL345_OFSZ, uint8_t(z));
}

// Sets the THRESH_ACT uint8_t which holds the threshold value for detecting activity.
// The data format is unsigned, so the magnitude of the activity event is compared
// with the value is compared with the value in the THRESH_ACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
// activity interrupt is enabled. The maximum value is 255.
void ADXL345::setActivityThreshold(int activityThreshold)
{
    activityThreshold = constrain(activityThreshold, 0, 255);
    uint8_t _b = uint8_t(activityThreshold);
    writeTo(ADXL345_THRESH_ACT, _b);
}

// Sets the THRESH_INACT uint8_t which holds the threshold value for detecting inactivity.
// The data format is unsigned, so the magnitude of the inactivity event is compared
// with the value is compared with the value in the THRESH_INACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
// inactivity interrupt is enabled. The maximum value is 255.
void ADXL345::setInactivityThreshold(int inactivityThreshold)
{
    inactivityThreshold = constrain(inactivityThreshold, 0, 255);
    uint8_t _b = uint8_t(inactivityThreshold);
    writeTo(ADXL345_THRESH_INACT, _b);
}

// Sets the TIME_INACT register, which contains an unsigned time value representing the
// amount of time that acceleration must be less thant the value in the THRESH_INACT
// register for inactivity to be declared. The scale factor is 1sec/LSB. The value must
// be between 0 and 255.
void ADXL345::setTimeInactivity(int timeInactivity)
{
    timeInactivity = constrain(timeInactivity, 0, 255);
    uint8_t _b = uint8_t(timeInactivity);
    writeTo(ADXL345_TIME_INACT, _b);
}

void ADXL345::setFreeFall(int freeFallThreshold, int freeFallDuration)
{ // Sets the THRESH_FF register which holds the threshold value, in an unsigned format, for
    // free-fall detection. The root-sum-square (RSS) value of all axes is calculated and
    // compared whith the value in THRESH_FF to determine if a free-fall event occured. The
    // scale factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the free-fall
    // interrupt is enabled. The maximum value is 255.

    freeFallThreshold = constrain(freeFallThreshold, 0, 255);
    uint8_t _b = uint8_t(freeFallThreshold);
    writeTo(ADXL345_THRESH_FF, _b);

    // Sets the TIME_FF register, which holds an unsigned time value representing the minimum
    // time that the RSS value of all axes must be less than THRESH_FF to generate a free-fall
    // interrupt. The scale factor is 5ms/LSB. A value of 0 may result in undesirable behavior if
    // the free-fall interrupt is enabled. The maximum value is 255.

    freeFallDuration = constrain(freeFallDuration, 0, 255);
    _b = uint8_t(freeFallDuration);
    writeTo(ADXL345_TIME_FF, _b);
}

void ADXL345::setActivity(bool x, bool y, bool z, bool ac)
{
    uint8_t _b;
    readFrom(ADXL345_ACT_INACT_CTL, 1, &_b);
    _b = _b & 0x8F;
    _b = _b | (ac << 7) | (x << 6) | (y << 5) | (z << 4);

    writeTo(ADXL345_ACT_INACT_CTL, _b);
}
void ADXL345::setInactivity(bool x, bool y, bool z, bool ac)
{
    uint8_t _b;
    readFrom(ADXL345_ACT_INACT_CTL, 1, &_b);
    _b = _b & 0xF8;
    _b = _b | (ac << 3) | (x << 2) | (y << 1) | z;
    writeTo(ADXL345_ACT_INACT_CTL, _b);
}

void ADXL345::setSuppressBit(bool state)
{
    setRegisterBit(ADXL345_TAP_AXES, 3, state);
}

bool ADXL345::isAsleep()
{
    return getRegisterBit(ADXL345_ACT_TAP_STATUS, 3);
}

bool ADXL345::isLowPower()
{
    return getRegisterBit(ADXL345_BW_RATE, 4);
}
void ADXL345::setLowPower(bool state)
{
    setRegisterBit(ADXL345_BW_RATE, 4, state);
}

float ADXL345::getRate()
{
    uint8_t _b;
    readFrom(ADXL345_BW_RATE, 1, &_b);
    _b &= 0b00001111;
    return (pow(2, ((int)_b) - 6)) * 6.25;
}

void ADXL345::setRate(float rate)
{
    uint8_t _b, _s;
    int v = (int)(rate / 6.25);
    int r = 0;
    while (v >>= 1)
    {
        r++;
    }
    if (r <= 9)
    {
        readFrom(ADXL345_BW_RATE, 1, &_b);
        _s = (uint8_t)(r + 6) | (_b & 0b11110000);
        writeTo(ADXL345_BW_RATE, _s);
    }
}

void ADXL345::set_bw(uint8_t bw_code)
{
    if ((bw_code < ADXL345_BW_3) || (bw_code > ADXL345_BW_1600))
    {
        status = false;
        error_code = ADXL345_BAD_ARG;
    }
    else
    {
        writeTo(ADXL345_BW_RATE, bw_code);
    }
}

uint8_t ADXL345::get_bw_code()
{
    uint8_t bw_code;
    readFrom(ADXL345_BW_RATE, 1, &bw_code);
    return bw_code;
}

interrupt_ADXL345 ADXL345::getInterruptSource()
{
    uint8_t _b;
    interrupt_ADXL345 _i;
    readFrom(ADXL345_INT_SOURCE, 1, &_b);
    _i.data_ready = (_b >> 7) & 1;
    _i.single_tap = (_b >> 6) & 1;
    _i.double_tap = (_b >> 5) & 1;
    _i.activity = (_b >> 4) & 1;
    _i.inactivity = (_b >> 3) & 1;
    _i.free_fall = (_b >> 2) & 1;
    _i.watermark = (_b >> 1) & 1;
    _i.overrun = (_b >> 0) & 1;

    return _i;
}

// Set the mapping of an interrupt to pin1 or pin2
// eg: setInterruptMapping(ADXL345_INT_float_TAP_BIT,ADXL345_INT2_PIN);
void ADXL345::setInterruptMapping(interrupt_ADXL345 interruptBit)
{
    uint8_t _b;
    _b |= interruptBit.overrun;
    _b |= interruptBit.watermark << 1;
    _b |= interruptBit.free_fall << 2;
    _b |= interruptBit.inactivity << 3;
    _b |= interruptBit.activity << 4;
    _b |= interruptBit.double_tap << 5;
    _b |= interruptBit.single_tap << 6;
    _b |= interruptBit.data_ready << 7;
    writeTo(ADXL345_INT_MAP, _b);
}

void ADXL345::setInterrupt(interrupt_ADXL345 interruptBit)
{
    uint8_t _b;
    _b |= interruptBit.overrun;
    _b |= interruptBit.watermark << 1;
    _b |= interruptBit.free_fall << 2;
    _b |= interruptBit.inactivity << 3;
    _b |= interruptBit.activity << 4;
    _b |= interruptBit.double_tap << 5;
    _b |= interruptBit.single_tap << 6;
    _b |= interruptBit.data_ready << 7;
    writeTo(ADXL345_INT_ENABLE, _b);
}

void ADXL345::setRegisterBit(uint8_t regAdress, int bitPos, bool state)
{
    uint8_t _b;
    readFrom(regAdress, 1, &_b);
    if (state)
    {
        _b |= (1 << bitPos); // forces nth bit of _b to be 1.  all other bits left alone.
    }
    else
    {
        _b &= ~(1 << bitPos); // forces nth bit of _b to be 0.  all other bits left alone.
    }
    writeTo(regAdress, _b);
}

bool ADXL345::getRegisterBit(uint8_t regAdress, int bitPos)
{
    uint8_t _b;
    readFrom(regAdress, 1, &_b);
    return ((_b >> bitPos) & 1);
}

// print all register value to the serial ouptut, which requires it to be setup
// this can be used to manually to check the current configuration of the device
void ADXL345::printAllRegister()
{
    uint8_t _b;
    /// Serial.print("0x00: ");
    readFrom(0x00, 1, &_b);
    print_uint8_t(_b);
    // Serial.println("");
    int i;
    for (i = 29; i <= 57; i++)
    {
        /// Serial.print("0x");
        /// Serial.print(i, HEX);
        /// Serial.print(": ");
        readFrom(i, 1, &_b);
        print_uint8_t(_b);
        // Serial.println("");
    }
}

void print_uint8_t(uint8_t val)
{
    int i;
    // Serial.print("B");
    for (i = 7; i >= 0; i--)
    {
        // Serial.print(val >> i & 1, BIN);
    }
}
