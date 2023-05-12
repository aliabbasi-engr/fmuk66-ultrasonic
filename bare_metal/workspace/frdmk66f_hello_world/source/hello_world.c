//////////////////////////////////////////////////////////////////////////////////
// Company: McMaster University
// Engineer: Ali Abbasi
//
// Create Date: 04/24/2023 09:48:25 PM
// Design Name: fmuk66_ultrasonic_baremetal
// Target Devices: NXP RDDRONE-FMUK66
// Description: This program calculates the distance using the ultrasonic module
// connected to the "ULTRASND SENS" JST-GH pin header on NXP RDDRONE-FMUK66
//////////////////////////////////////////////////////////////////////////////////

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "math.h"
#include "stdio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ULTRASONIC_TRIGGER_GPIO     GPIOD
#define ULTRASONIC_TRIGGER_GPIO_PIN 0

#define ULTRASONIC_ECHO_GPIO     	GPIOA
#define ULTRASONIC_ECHO_GPIO_PIN 	10

#define ULTRASONIC_ECHO_VALUE		1U

#define ULTRASONIC_ECHO_PROPAGATION_DELAY	35

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief delay a while.
 */
void gpio_configure(void);
void delay(void);
void reset_timer(void);
void start_timer(void);
void stop_timer(void);
unsigned int get_cycles(void);
int read_echo(void);
int get_int_part(float);
int get_float_part(float, int);

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile unsigned int *DWT_CYCCNT  ;
volatile unsigned int *DWT_CONTROL ;
volatile unsigned int *SCB_DEMCR   ;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    /* Board pin, clock, debug console init */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    gpio_configure();

    int elapsed_cycles;
    float elapsed_time_us;
    float distance_cm;

    /* Print a note to terminal. */
    PRINTF("\r\n** Ultrasonic Driver Example **\r\n");

    delay();

    while (1)
    {
    	reset_timer();
    	GPIO_PortSet(ULTRASONIC_TRIGGER_GPIO, 1u << ULTRASONIC_TRIGGER_GPIO_PIN);
    	start_timer();

        while (get_cycles() <= 180) {}

        GPIO_PortClear(ULTRASONIC_TRIGGER_GPIO, 1u << ULTRASONIC_TRIGGER_GPIO_PIN);
        stop_timer();

        reset_timer();
        while (read_echo() != ULTRASONIC_ECHO_VALUE) {}
        start_timer();

        while (read_echo() == ULTRASONIC_ECHO_VALUE) {}

        stop_timer();
        elapsed_cycles = get_cycles();
        //PRINTF("Elapsed cycles = %d\r\n", elapsed_cycles);

        elapsed_time_us = 0.0555555555555556f * elapsed_cycles;

        elapsed_time_us = elapsed_time_us - ULTRASONIC_ECHO_PROPAGATION_DELAY;

        //PRINTF("Elapsed time = %d.%dus\r\n", get_int_part(elapsed_time_us), get_float_part(elapsed_time_us, 2));

        distance_cm = elapsed_time_us * 0.034f / 2;
        PRINTF("Distance = %d.%dcm\r\n", get_int_part(distance_cm), get_float_part(distance_cm, 2));

        delay();
    }
}

void gpio_configure(void)
{
    /* Define the init structure for the ultrasonic trigger output pin*/
    gpio_pin_config_t ultrasonic_trigger_pin_config = {
        kGPIO_DigitalOutput,
        0,
    };

    /* Define the init structure for the ultrasonic echo input pin*/
    gpio_pin_config_t ultrasonic_echo_pin_config = {
        kGPIO_DigitalInput,
        0,
    };

    /* Init output ultrasonic trigger. */
    GPIO_PinInit(ULTRASONIC_TRIGGER_GPIO, ULTRASONIC_TRIGGER_GPIO_PIN, &ultrasonic_trigger_pin_config);

    /* Init input ultrasonic echo. */
    GPIO_PinInit(ULTRASONIC_ECHO_GPIO, ULTRASONIC_ECHO_GPIO_PIN, &ultrasonic_echo_pin_config);
}

void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 80000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

void reset_timer(void)
{
    DWT_CYCCNT   = (int *)0xE0001004;
    DWT_CONTROL  = (int *)0xE0001000;
    SCB_DEMCR    = (int *)0xE000EDFC;
    *SCB_DEMCR   = *SCB_DEMCR | 0x01000000;
    *DWT_CYCCNT  = 0; // reset the counter
    *DWT_CONTROL = 0;
}

void start_timer(void)
{
    *DWT_CONTROL = *DWT_CONTROL | 1 ;
}

void stop_timer(void)
{
    *DWT_CONTROL = *DWT_CONTROL | 0 ;
}

unsigned int get_cycles(void)
{
    return *DWT_CYCCNT;
}

int read_echo(void)
{
	return GPIO_PinRead(ULTRASONIC_ECHO_GPIO, ULTRASONIC_ECHO_GPIO_PIN);
}

int get_int_part(float val)
{
    return (int)val;
}

int get_float_part(float val, int precision)
{
	int int_part = (int)val;
	float float_part = val - (float)int_part;
	return float_part * pow(10, precision);
}
