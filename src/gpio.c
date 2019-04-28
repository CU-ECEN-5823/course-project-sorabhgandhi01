/*
 * gpio.c
 *
 *  Created on: April 5, 2019
 *      Author: Sorabh Gandhi
 */
#include "gpio.h"
#include "em_gpio.h"
#include <string.h>
#include "log.h"
#include "native_gecko.h"
#include "main.h"



/***************************************************************************
 * gpioInit
 * *************************************************************************
 * @brief	This function initializes on-Board LED0 and Push Button 0 and 1
 *
 * @param	none
 *
 * @result	none
 *
 */
void gpioInit()
{
	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateStrong);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);

	GPIO_PinModeSet(PB0_BUTTON_PORT, PB0_BUTTON_PIN, gpioModeInputPull, true);
	GPIO_PinModeSet(PB1_BUTTON_PORT, PB1_BUTTON_PIN, gpioModeInputPull, true);
}



/***************************************************************************
 * gpioLed0SetOn
 * *************************************************************************
 * @brief	This function turns the on-board LED0 ON and sets a hardware
 * 			soft-timer to turn-off the LED
 *
 * @param	none
 *
 * @result	none
 *
 */
void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
	gecko_cmd_hardware_set_soft_timer(1 * 32768, TIMER_ID_SPRAY_RESET, 1);
}


/***************************************************************************
 * gpioLed0SetOff
 * *************************************************************************
 * @brief	This function turns OFF the on-board LED0
 *
 * @param	none
 *
 * @result	none
 *
 */
void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}

void gpioEnableDisplay()
{
	GPIO_PinOutSet(gpioPortD, 15);
}

void gpioSetDisplayExtcomin(bool high)
{
	if (high) {
		GPIO_PinOutSet(gpioPortD, 13);
	} else {
		GPIO_PinOutClear(gpioPortD, 13);
	}
}



/***************************************************************************
 * GPIOINT_Deint
 * *************************************************************************
 * @brief	This function clears all the pending GPIO Interrupts and Disables
 * 			all the GPIO Interrupts
 *
 * @param	none
 *
 * @result	none
 *
 */
void GPIOINT_Deint(void)
{
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
}


/***************************************************************************
 * enable_sensor_interrupts
 * *************************************************************************
 * @brief	This function initializes GPIO pins for both the sensor, sets the
 * 			falling-edge interrupt mode and registers the interrupt handler
 *
 * @param	none
 *
 * @result	none
 *
 */
void enable_sensor_interrupts(void)
{
	GPIO_PinModeSet(IR_SENSOR_PORT, IR_SENSOR_1_PIN, gpioModeInputPull, true);
	GPIO_PinModeSet(IR_SENSOR_PORT, IR_SENSOR_2_PIN, gpioModeInputPull, true);

	GPIOINT_Init();

	/* configure interrupt for IR_SENSOR_1 and IR_SENSOR_2, for falling edges */
	GPIO_ExtIntConfig(IR_SENSOR_PORT, IR_SENSOR_1_PIN, IR_SENSOR_1_PIN, false, true, true);
	GPIO_ExtIntConfig(IR_SENSOR_PORT, IR_SENSOR_2_PIN, IR_SENSOR_2_PIN, false, true, true);

	/* register the callback function that is invoked when interrupt occurs */
	GPIOINT_CallbackRegister(IR_SENSOR_1_PIN, right_sensor_int);
	GPIOINT_CallbackRegister(IR_SENSOR_2_PIN, left_sensor_int);
}


/***************************************************************************
 * disable_sensor_interrupts
 * *************************************************************************
 * @brief	This function disables the sensor 1 and sensor 2 pin
 *
 * @param	none
 *
 * @result	none
 *
 */
void disable_sensor_interrupts(void)
{
	GPIO_PinModeSet(IR_SENSOR_PORT, IR_SENSOR_1_PIN, gpioModeDisabled, true);
	GPIO_PinModeSet(IR_SENSOR_PORT, IR_SENSOR_2_PIN, gpioModeDisabled, true);

	GPIOINT_Deint();

	GPIO_ExtIntConfig(IR_SENSOR_PORT, IR_SENSOR_1_PIN, IR_SENSOR_1_PIN, false, true, false);
	GPIO_ExtIntConfig(IR_SENSOR_PORT, IR_SENSOR_2_PIN, IR_SENSOR_2_PIN, false, true, false);
}



/***************************************************************************
 * right_sensor_int
 * *************************************************************************
 * @brief	This function is a call-back for IR_SENSOR_1 and it sets the status
 * 			flag and notifies to the gecko_external_event
 *
 * @param	none
 *
 * @result	none
 *
 */
void right_sensor_int(uint8_t pin)
{
	CORE_ATOMIC_IRQ_DISABLE();
	if ((pin == IR_SENSOR_1_PIN))
	{
		EXT_SIGNAL_SENSOR_1 |= SENSOR_1_STATUS;
		gecko_external_signal(EXT_SIGNAL_SENSOR_1);
	}

	CORE_ATOMIC_IRQ_ENABLE();
}



/***************************************************************************
 * left_sensor_int
 * *************************************************************************
 * @brief	This function is a call-back for IR_SENSOR_2 and it sets the status
 * 			flag and notifies to the gecko_external_event
 *
 * @param	none
 *
 * @result	none
 *
 */
void left_sensor_int(uint8_t pin)
{
	CORE_ATOMIC_IRQ_DISABLE();
	if ((pin == IR_SENSOR_2_PIN))
	{
		EXT_SIGNAL_SENSOR_2 |= SENSOR_2_STATUS;
		gecko_external_signal(EXT_SIGNAL_SENSOR_2);
	}

	CORE_ATOMIC_IRQ_ENABLE();
}
