#include "driver.h"

driver::driver(int EnA, int pinA1, int pinA2, int EnB, int pinB1, int pinB2)
{

    this->EnA = EnA;
    this->EnB = EnB;
    this->pinA1 = pinA1;
    this->pinA2 = pinA2;
    this->pinB1 = pinB1;
    this->pinB2 = pinB2;

    gpio_init_mask((1u << EnA) | (1 << pinA1) | (1 << pinA2) | (1 << EnB) | (1 << pinB1) | (1 << pinB2));
    gpio_set_dir_out_masked((1 << pinA1) | (1 << pinA2) | (1 << pinB1) | (1 << pinB2));
    gpio_put(pinA1, 0);
    gpio_put(pinA2, 0);
    gpio_put(pinB1, 0);
    gpio_put(pinB2, 0);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, count_top);
    pwm_init(pwm_gpio_to_slice_num(EnA), &cfg, true);
    gpio_set_function(EnA, GPIO_FUNC_PWM);
    pwm_set_gpio_level(EnA, 0);

    pwm_init(pwm_gpio_to_slice_num(EnB), &cfg, true);
    gpio_set_function(EnB, GPIO_FUNC_PWM);
    pwm_set_gpio_level(EnB, 0);
}
void driver::forward()
{
    gpio_put(pinA1, 1);
    gpio_put(pinB1, 1);
    gpio_put(pinA2, 0);
    gpio_put(pinB2, 0);
    pwm_set_gpio_level(EnA, (count_top + 1) * 0.50f);
    pwm_set_gpio_level(EnB, (count_top + 1) * 0.49f);
}
void driver::stop()
{

    pwm_set_gpio_level(EnA, (count_top + 1) * duty_cycles[0]);
    pwm_set_gpio_level(EnB, (count_top + 1) * duty_cycles[0]);
}
void driver::turnLeft()
{
    gpio_put(pinA1, 1);
    gpio_put(pinB1, 0);
    gpio_put(pinA2, 0);
    gpio_put(pinB2, 1);
    pwm_set_gpio_level(EnA, (count_top + 1) * duty_cycles[5]);
    pwm_set_gpio_level(EnB, (count_top + 1) * duty_cycles[5]);
}
void driver::turnRight()
{
    gpio_put(pinA1, 0);
    gpio_put(pinB1, 1);
    gpio_put(pinA2, 1);
    gpio_put(pinB2, 0);
    pwm_set_gpio_level(EnA, (count_top + 1) * duty_cycles[5]);
    pwm_set_gpio_level(EnB, (count_top + 1) * duty_cycles[5]);
}
void driver::backward()
{
    gpio_put(pinA1, 0);
    gpio_put(pinB1, 0);
    gpio_put(pinA2, 1);
    gpio_put(pinB2, 1);
    pwm_set_gpio_level(EnA, (count_top + 1) * 0.50f);
    pwm_set_gpio_level(EnB, (count_top + 1) * 0.49f);
}