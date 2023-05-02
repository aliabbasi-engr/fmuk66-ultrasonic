#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ULTRASONIC_TRIGGER_GPIO     GPIOD
#define ULTRASONIC_TRIGGER_GPIO_PIN 4

#define ULTRASONIC_ECHO_GPIO     	GPIOD
#define ULTRASONIC_ECHO_GPIO_PIN 	5

#define ULTRASONIC_ECHO_VALUE		1U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief delay a while.
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile unsigned int *DWT_CYCCNT  ;
volatile unsigned int *DWT_CONTROL ;
volatile unsigned int *SCB_DEMCR   ;

/*******************************************************************************
 * Code
 ******************************************************************************/
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
    for (i = 0; i < 800000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

void reset_timer()
{
    DWT_CYCCNT   = (int *)0xE0001004; //address of the register
    DWT_CONTROL  = (int *)0xE0001000; //address of the register
    SCB_DEMCR    = (int *)0xE000EDFC; //address of the register
    *SCB_DEMCR   = *SCB_DEMCR | 0x01000000;
    *DWT_CYCCNT  = 0; // reset the counter
    *DWT_CONTROL = 0;
}

void start_timer()
{
    *DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter
}

void stop_timer()
{
    *DWT_CONTROL = *DWT_CONTROL | 0 ; // disable the counter
}

unsigned int get_cycles()
{
    return *DWT_CYCCNT;
}

int read_echo()
{
	return GPIO_PinRead(ULTRASONIC_ECHO_GPIO, ULTRASONIC_ECHO_GPIO_PIN);
}

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
    PRINTF("\r\n Ultrasonic Driver example\r\n");

    delay();

    while (1)
    {
    	PRINTF("\r\n Sending the trigger signal.\r\n");

    	reset_timer(); //reset timer
    	GPIO_PortSet(ULTRASONIC_TRIGGER_GPIO, 1u << ULTRASONIC_TRIGGER_GPIO_PIN);
    	start_timer(); //start timer

        while (get_cycles() <= 180) {}

        GPIO_PortClear(ULTRASONIC_TRIGGER_GPIO, 1u << ULTRASONIC_TRIGGER_GPIO_PIN);
        stop_timer(); //stop timer

        /*reset_timer(); //reset timer
        while (read_echo() != ULTRASONIC_ECHO_VALUE) {}
        start_timer(); //start timer

        while (read_echo() == ULTRASONIC_ECHO_VALUE) {}

        stop_timer();
        elapsed_cycles = get_cycles();

        elapsed_time_us = 0.0555f * elapsed_cycles;

        distance_cm = elapsed_time_us * 0.034f / 2;

        PRINTF("Distance = %dcm", distance_cm);*/
    }
}
