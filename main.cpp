#include "pico/stdlib.h"

#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/regs/rosc.h"       //random number gen
#include "hardware/regs/addressmap.h" //random number gen
#include "pico/binary_info.h"

#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include "motorDriver/driver.h"
#include "adxl345/ADXL345.hpp"

#define UART_ID uart1
#define BAUD_RATE 9600

#define UART_TX_PIN 4
#define UART_RX_PIN 5
void seed_random_from_rosc()
{
    uint32_t random = 0x811c9dc5;
    uint8_t next_byte = 0;
    volatile uint32_t *rnd_reg = (uint32_t *)(ROSC_BASE + ROSC_RANDOMBIT_OFFSET);

    for (int i = 0; i < 16; i++)
    {
        for (int k = 0; k < 8; k++)
        {
            next_byte = (next_byte << 1) | (*rnd_reg & 1);
        }

        random ^= next_byte;
        random *= 0x01000193;
    }

    srand(random);
}
bool reserved_addr(uint8_t addr)
{
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}
int busScan()
{
    // Enable UART so we can print status output

    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(i2c1, 100 * 1000);
    gpio_init(2);
    gpio_init(3);
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
   
    // Make the I2C pins available to picotool
    //bi_decl(bi_2pins_with_func(2, 3, GPIO_FUNC_I2C));

    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr)
    {
        if (addr % 16 == 0)
        {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");
    return 0;
}

driver drove = driver(14, 13, 12, 9, 11, 10);
int main()
{
    stdio_init_all();
    seed_random_from_rosc();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_is_dir_out(PICO_DEFAULT_LED_PIN);
    /* while (!stdio_usb_connected())
        { // blink the pico's led until usb connection is established
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(250);
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(250);
        }*/
    
        sleep_ms(200);
        drove.forward();

        sleep_ms(1200);
        drove.stop();

        sleep_ms(200);
        drove.turnLeft();

        sleep_ms(2680);
        drove.stop();

        sleep_ms(200);
        drove.forward();

        sleep_ms(1200);
        drove.stop();

    while (1)
    {
        

        busScan();
        sleep_ms(1000);
    }
}
