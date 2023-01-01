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
    const float duty_cycles[11] = {
        0.f,
        0.1f,
        0.2f,
        0.3f,
        0.4f,
        0.5f,
        0.6f,
        0.7f,
        0.8f,
        0.9f,
        1.f};
};