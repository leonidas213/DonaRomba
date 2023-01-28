#include "pico/stdlib.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stdlib.h"

#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/regs/rosc.h"       //random number gen
#include "hardware/regs/addressmap.h" //random number gen
#include "pico/binary_info.h"

#include "pico/util/queue.h"
#include "pico/multicore.h"

#include "motorDriver/driver.h"
#include "adxl345/ADXL345.hpp"
#include "MPU6050/MPU6050_6Axis_MotionApps_V6_12.h"
#include "hmc5883l/hmc5883l.h"

#pragma region Variables
//---------------------------------------------------------------------------------------------------
driver drove = driver(14, 13, 12, 9, 11, 10);

int buf[3];
hmc5883l compass = hmc5883l(i2c1);
//---------mpu6050--------

#define OUTPUT_READABLE_YAWPITCHROLL
// #define OUTPUT_READABLE_REALACCEL
// #define OUTPUT_READABLE_WORLDACCEL
// #define OUTPUT_READABLE_CUSTOM

MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 gy;      // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw, pitch, roll;

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

//---------mpu6050--------
typedef struct
{
    bool turn;
    int val;
} ControlCommand;

queue_t commandQueue;
queue_t callbackQueue;

//---------------------------------------------------------------------------------------------------
#pragma endregion

#pragma region HelpfulFunctions

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

    // Make the I2C pins available to picotool
    // bi_decl(bi_2pins_with_func(2, 3, GPIO_FUNC_I2C));

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
            ret = i2c_read_blocking(i2c1, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");
    return 0;
}
void waitUsb()
{
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    while (!stdio_usb_connected())
    {
        gpio_put(25, 0);
        sleep_ms(250);
        gpio_put(25, 1);
        sleep_ms(250);
    }
}

bool isBetween(float input, float refValue, float offset)
{ // 120  100    90
    // min=10 max=190

    // difference between 2 degrees
    float diff = abs(input - refValue);
    if (diff > 180)
    {
        diff = 360 - diff;
    }
    return diff < offset;
}
#pragma endregion

#pragma region Interrupts
void hmcRead(uint gpio, uint32_t events)
{
    gpio_set_irq_enabled(20, GPIO_IRQ_EDGE_FALL, false);

    compass.read(buf);
    printf("x: %d, y: %d, z: %d\n", buf[0], buf[1], buf[2]);

    gpio_set_irq_enabled(20, GPIO_IRQ_EDGE_FALL, true);
}
#pragma endregion

void core1_entry()
{ //---------mpu6050--------100hz

    printf("starting mpu\n");
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0)
    {
        mpu.setDMPEnabled(true); // turn on the DMP, now that it's ready
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;                         // set our DMP Ready flag so the main loop() function knows it's okay to use it
        packetSize = mpu.dmpGetFIFOPacketSize(); // get expected DMP packet size for later comparison
    }
    else
    { // ERROR!        1 = initial memory load failed         2 = DMP configuration updates failed        (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)", devStatus);
        sleep_ms(2000);
    }
    yaw = 0.0;
    pitch = 0.0;
    roll = 0.0;
    //---------mpu6050--------
    bool completed = true;
    uint32_t start = time_us_32();
    while (1)
    {

        //---------mpu6050--------
        if (!dmpReady)
            ; // if programming failed, don't try to do anything
        mpuInterrupt = true;
        fifoCount = mpu.getFIFOCount();                 // get current FIFO count
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) // check for overflow (this should never happen unless our code is too inefficient)
        {
            mpu.resetFIFO(); // reset so we can continue cleanly
            printf("FIFO overflow!");
        }
        else if (mpuIntStatus & 0x01) // otherwise, check for DMP data ready interrupt (this should happen frequently)
        {
            while (fifoCount < packetSize)
                fifoCount = mpu.getFIFOCount();       // wait for correct available data length, should be a VERY short wait
            mpu.getFIFOBytes(fifoBuffer, packetSize); // read a packet from FIFO

            fifoCount -= packetSize; // track FIFO count here in case there is > 1 packet available
#ifdef OUTPUT_READABLE_YAWPITCHROLL  // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw = (ypr[0] * 180 / PI) > 0 ? (ypr[0] * 180 / PI) : 360 + (ypr[0] * 180 / PI);
            // yaw = (ypr[0] * 180 / PI);
            pitch = (ypr[1] * 180 / PI) > 0 ? (ypr[1] * 180 / PI) : 360 + (ypr[1] * 180 / PI);
            roll = (ypr[2] * 180 / PI) > 0 ? (ypr[2] * 180 / PI) : 360 + (ypr[2] * 180 / PI);
            // printf("ypr: %f,\t %f,\t %f\n", yaw, pitch, roll);
#endif
#ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            printf("areal: %d,\t %d,\t %d\n", aaReal.x, aaReal.y, aaReal.z);
#endif
#ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            printf("aworld: %d,\t %d,\t %d\n", aaWorld.x, aaWorld.y, aaWorld.z);
#endif
#ifdef OUTPUT_READABLE_CUSTOM
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            printf("W: %f\t X: %f\t Y: %f\t Z: %f\n", q.w, q.x, q.y, q.z);
#endif
        }
        //---------mpu6050--------

        ControlCommand command;
        if (!queue_is_empty(&commandQueue) & completed)
        {
            queue_remove_blocking(&commandQueue, &command);
        }
        /* switch (ch)
         {
         case 'f':
             drove.forward();
             printf("turning forward\n");

             sleep_ms(250);
             drove.stop();
             break;
         case 'b':
             drove.backward();
             printf("turning backward\n");

             sleep_ms(250);
             drove.stop();
             break;
         case 'l':

             printf("ypr: %f, %f, %f\n", yaw, pitch, roll);
             tempyaw = yaw;
             DegDiff = 0;
             printf("turned1 %f degrees\n", tempyaw);
             printf("turned2 %f degrees\n", yaw);
             timStart = time_us_64();
             while (isBetween(yaw, tempyaw, 90))
             {

                 drove.turnLeft();
                 if (once)
                 {

                     tempyaw = yaw;
                     once = false;
                 }
                 if (-timStart + time_us_64() > 1000000)
                 {
                     break;
                 }
             }
             printf("turning left\n");
             drove.stop();
             DegDiff = 0;

             printf("turned1 %f degrees\n", tempyaw);
             printf("turned2 %f degrees\n", yaw);
             break;
         case 'r':
             drove.turnRight();
             printf("turning right\n");

             sleep_ms(250);
             drove.stop();

         default:
             drove.stop();
             break;
         }*/

        if (!completed)
        {
            if (command.turn)
            {
                if (command.val > 0)
                {
                }
                else
                {
                }
            }
            else
            {
                if (command.val > 0)
                {
                }
                else
                {
                }
            }
        }

        // queue_add_blocking(&results_queue, &result);
    }
}

int main()
{
    stdio_init_all();
    seed_random_from_rosc();
    uint64_t timStart = time_us_64();
    //------------i2c------------
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(3);
    gpio_pull_up(2);
    //------------i2c------------s

    //---------bluetooth------
    gpio_init(4);
    gpio_init(5);
    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);
    stdio_uart_init_full(uart1, 115200, 4, 5);
    //---------bluetooth------

    // waitUsb();

    multicore_launch_core1(core1_entry);
    float tempyaw = 0;
    string uartvalue = "";

    compass.begin();
    compass.declination = 5.967f; // 5 58' E // https://www.magnetic-declination.com/
    gpio_set_irq_enabled_with_callback(20, GPIO_IRQ_EDGE_FALL, true, &hmcRead);

    while (1)
    {
        float heading = compass.heading(buf);
        printf("heading: %f\n", heading);
        sleep_ms(200);

        // uart_write_blocking(uart1, (const uint8_t *)"Şerefli Burak\n", 15);
        if (uart_is_readable(uart1))
        {
            uint8_t ch = uart_getc(uart1);
            if (ch != '\r' & ch != '\n')
            {
                // uartvalue.push_back(ch);
                // printf(uartvalue.c_str());

                printf("ypr: %f, %f, %f\n", yaw, pitch, roll);
                int DegDiff = 0;
                bool once = true;
            }
            /// else
            ///{
            ///     if (uartvalue != "")
            ///     {
            ///         printf("%s\n", uartvalue);
            ///         //drove.pwm = std::stoi(uartvalue, nullptr, 10);
            ///         uartvalue = "";
            ///         drove.forward();
            ///     }
            /// }
        }
    }
}
