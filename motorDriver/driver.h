#include "pico/stdlib.h"
#include "hardware/pwm.h"

class driver
{

public:
    driver(int EnA, int pinA1, int pinA2, int EnB, int pinB1, int pinB2);
    void forward();
    void turnLeft();
    void turnRight();
    void backward();
    void stop();

private:
    int EnA, pinA1, pinA2, EnB, pinB1, pinB2;

    const uint count_top = 1000;
    const int duty_cycles[10] = {
        0,
        400,
        478,
        556,
        634,
        712,
        790,
        868,
        946,
        1024};
};