// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// --------------- //
// distance_sensor //
// --------------- //

#define distance_sensor_wrap_target 0
#define distance_sensor_wrap 6

static const uint16_t distance_sensor_program_instructions[] = {
            //     .wrap_target
    0x6020, //  0: out    x, 32           side 0     
    0xba42, //  1: nop                    side 1 [10]
    0x20a0, //  2: wait   1 pin, 0        side 0     
    0x0044, //  3: jmp    x--, 4          side 0     
    0x00c3, //  4: jmp    pin, 3          side 0     
    0x4020, //  5: in     x, 32           side 0     
    0xc010, //  6: irq    nowait 0 rel    side 0     
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program distance_sensor_program = {
    .instructions = distance_sensor_program_instructions,
    .length = 7,
    .origin = -1,
};

static inline pio_sm_config distance_sensor_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + distance_sensor_wrap_target, offset + distance_sensor_wrap);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}

    #include "hardware/pio.h"
    #include "hardware/clocks.h"
    /**
     * Initializer for the above program
     * @param[in] pio the PIO instance to use
     * @param[in] sm state machine to use for the PIO instance
     * @param[in] offset Offset into PIO memory to place the program into
     * @param[in] pins the first of 2 sequential gpios to use. pins = trigger pin, pins + 1 = echo pin.
     * @param[in] isr Routine executed when the distance sensing starts and ends
     *            Can be null if you want to enable interrupts yourself.
     *            check interrupt source sm for sense start and (sm + 1) for sense end.
     *            the distance measured in centimeters is microseconds between start and end divided by 58 for the HC-SR04
     */
    static inline void distance_sensor_program_init(PIO pio, uint sm, uint offset, uint pins, irq_handler_t isr) {
        // Set the trigger pin and echo pin as gpio pins
        pio_gpio_init(pio, pins);
        pio_gpio_init(pio, pins + 1);
        // Set the trigger pin as an output pin
        pio_sm_set_consecutive_pindirs(pio, sm, pins, 1, true);
        // Set the echo pin as an input pin
        pio_sm_set_consecutive_pindirs(pio, sm, pins + 1, 1, false);
        // Enable the IRQ source. 0 for echo high, 1 for echo low
        pio_set_irq0_source_enabled(pio, (enum pio_interrupt_source)(pis_interrupt0 + sm), true);
        if (isr != NULL) {
            // Set the function to be executed for the isr.
            irq_set_exclusive_handler(pio == pio0 ? PIO0_IRQ_0 : PIO1_IRQ_0, isr);
            // Enable the ISR
            irq_set_enabled(pio == pio0 ? PIO0_IRQ_0 : PIO1_IRQ_1, true);
            pio_interrupt_clear(pio, sm);
        }
        // Build the configuration for the state machine
        pio_sm_config config = distance_sensor_program_get_default_config(offset);
        // Set the trigger pin as a sideset pin
        sm_config_set_sideset_pins(&config, pins);
        // Set the in pin to pins + 1, the echo pin.
        sm_config_set_in_pins(&config, pins + 1);
        // Set up autopull to pull the TX Fifo into the OSR so assembly
        // does not need to waste an instruction with PULL
        sm_config_set_out_shift(&config, true, true, 32);
        sm_config_set_in_shift(&config, true, true, 32);
        // Set the jump pin for the counter to work
        sm_config_set_jmp_pin(&config, pins + 1);
        // Desired frequency is 1MHz, or 1 microsecond per cycle.
        float freq = 1000000;
        float div = clock_get_hz(clk_sys) / (freq);
        sm_config_set_clkdiv(&config, div);
        // Load the config and execute the state machine
        pio_sm_init(pio, sm, offset, &config);
        pio_sm_set_enabled(pio, sm, true);
    }

#endif
