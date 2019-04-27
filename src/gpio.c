/*
 * gpio.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */
#include "gpio.h"
#include "em_gpio.h"
#include <string.h>
#include "log.h"

#define	LED0_port gpioPortF
#define LED0_pin	4
#define LED1_port gpioPortF
#define LED1_pin 5

void gpioInit()
{
	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateStrong);
	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateStrong);
	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);

	GPIO_PinModeSet(PB0_BUTTON_PORT, PB0_BUTTON_PIN, gpioModeInputPull, true);
	GPIO_PinModeSet(PB1_BUTTON_PORT, PB1_BUTTON_PIN, gpioModeInputPull, true);

	GPIO_PinModeSet(IR_SENSOR_PORT, IR_SENSOR_1_PIN, gpioModeInputPull, true);
	GPIO_PinModeSet(IR_SENSOR_PORT, IR_SENSOR_2_PIN, gpioModeInputPull, true);
}

void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}
void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}
void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}
void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
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

void gpioint(uint8_t pin)
{
	if (pin == PB0_BUTTON_PIN)
	{
		EXT_SIGNAL_PB0_BUTTON |= PB0_BUTTON_STATUS;
		gecko_external_signal(EXT_SIGNAL_PB0_BUTTON);
	}
	else if (pin == PB1_BUTTON_PIN)
	{
		EXT_SIGNAL_PB1_BUTTON |= PB1_BUTTON_STATUS;
		gecko_external_signal(EXT_SIGNAL_PB1_BUTTON);
	}
}


void enable_button_interrupts(void)
{
  GPIOINT_Init();

  /* configure interrupt for PB0 and PB1, both falling and rising edges */
  GPIO_ExtIntConfig(PB0_BUTTON_PORT, PB0_BUTTON_PIN, PB0_BUTTON_PIN, true, true, true);
  GPIO_ExtIntConfig(PB1_BUTTON_PORT, PB1_BUTTON_PIN, PB1_BUTTON_PIN, true, true, true);

  /* register the callback function that is invoked when interrupt occurs */
  GPIOINT_CallbackRegister(PB0_BUTTON_PIN, gpioint);
  GPIOINT_CallbackRegister(PB1_BUTTON_PIN, gpioint);
}

void enable_sensor_interrupts(void)
{
	GPIOINT_Init();

	/* configure interrupt for PB0 and PB1, both falling and rising edges */
	GPIO_ExtIntConfig(IR_SENSOR_PORT, IR_SENSOR_1_PIN, IR_SENSOR_1_PIN, false, true, true);
	GPIO_ExtIntConfig(IR_SENSOR_PORT, IR_SENSOR_2_PIN, IR_SENSOR_2_PIN, false, true, true);

	/* register the callback function that is invoked when interrupt occurs */
	GPIOINT_CallbackRegister(IR_SENSOR_1_PIN, right_sensor_int);
	GPIOINT_CallbackRegister(IR_SENSOR_2_PIN, left_sensor_int);

	right_sensor_active = 1;
	left_sensor_active = 1;
}

void right_sensor_int(uint8_t pin)
{
	CORE_ATOMIC_IRQ_DISABLE();
	if ((pin == IR_SENSOR_1_PIN) && (right_sensor_active == 1))
	{
		LOG_INFO("In S1\t");
		EXT_SIGNAL_SENSOR_1 |= SENSOR_1_STATUS;
		gecko_external_signal(EXT_SIGNAL_SENSOR_1);
	}

	CORE_ATOMIC_IRQ_ENABLE();
}

void left_sensor_int(uint8_t pin)
{
	CORE_ATOMIC_IRQ_DISABLE();
	if ((pin == IR_SENSOR_2_PIN) && (left_sensor_active == 1))
	{
		LOG_INFO("In S2\t");
		EXT_SIGNAL_SENSOR_2 |= SENSOR_2_STATUS;
		gecko_external_signal(EXT_SIGNAL_SENSOR_2);
	}

	CORE_ATOMIC_IRQ_ENABLE();
}
