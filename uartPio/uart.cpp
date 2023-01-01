#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "uart.pio.h"
#include "hardware/clocks.h"
#include <string>

void uartInit(uint txPin, uint rxPin, uint baudrate)
{
    PIO pio = pio0;
    uint smTx = 4;
    uint smRx = 5;
    uint offsetTx = pio_add_program(pio, &uart_tx_program);
    uint offsetRx = pio_add_program(pio, &uart_rx_program);
    pio_sm_config cTx = uart_tx_program_get_default_config(offsetTx);
    pio_sm_config cRx = uart_rx_program_get_default_config(offsetRx);

    pio_sm_set_pins_with_mask(pio, smTx, 1u << txPin, 1u << txPin);
    pio_sm_set_pindirs_with_mask(pio, smTx, 1u << txPin, 1u << txPin);

    pio_gpio_init(pio, txPin);

    pio_sm_set_consecutive_pindirs(pio, smRx, rxPin, 1, false);
    pio_gpio_init(pio, rxPin);
    gpio_pull_up(rxPin);

    sm_config_set_out_shift(&cTx, true, false, 32);
    sm_config_set_in_shift(&cRx, true, false, 32);

    sm_config_set_in_pins(&cRx, rxPin);
    sm_config_set_jmp_pin(&cRx, rxPin);

    sm_config_set_out_pins(&cTx, txPin, 1);
    sm_config_set_sideset_pins(&cTx, txPin);

    sm_config_set_fifo_join(&cTx, PIO_FIFO_JOIN_TX);
    sm_config_set_fifo_join(&cRx, PIO_FIFO_JOIN_RX);

    float div = (float)clock_get_hz(clk_sys) / (8 * baudrate);
    sm_config_set_clkdiv(&cRx, div);
    sm_config_set_clkdiv(&cTx, div);

    pio_sm_init(pio, smTx, offsetTx, &cTx);
    pio_sm_set_enabled(pio, smTx, true);

    pio_sm_init(pio, smRx, offsetRx, &cRx);
    pio_sm_set_enabled(pio, smRx, true);
}
static inline char Read(PIO pio, uint sm)
{
    // 8-bit read from the uppermost byte of the FIFO, as data is left-justified
    io_rw_8 *rxfifo_shift = (io_rw_8 *)&pio->rxf[sm] + 3;
    while (pio_sm_is_rx_fifo_empty(pio, sm))
        tight_loop_contents();
    return (char)*rxfifo_shift;
}
static inline void uart_tx_program_putc(PIO pio, uint sm, char c)
{
    pio_sm_put_blocking(pio, sm, (uint32_t)c);
}
static inline void Write(PIO pio, uint sm, const char *s)
{
    while (*s)
        uart_tx_program_putc(pio, sm, *s++);
}

int main()
{
    stdio_init_all();
    uartInit(5,4, 115200);
    // pioi2cInit();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, 1);

    // Write(pio0, 0, "hELLLO UwU\n~");
    std::string a = "";
    while (1)
    {
        Write(pio0, 0, "hELLOUwU~");
        /*char c = //Read(pio0, 1);
        if (c != '~')
            a += c;
        else
        {

            a += "~";

            Write(pio0, 0, a.c_str());
            a = "";
        }*/

        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(200);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(200);
    }
    puts("Hello, world!");

    return 0;
}
