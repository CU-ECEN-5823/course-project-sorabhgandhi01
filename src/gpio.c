/*
 * gpio.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */
#include "gpio.h"
#include "em_gpio.h"
#include <string.h>


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

	GPIO_PinModeSet(PIR_SENSOR_PORT, PIR_SENSOR_1, gpioModeInputPull, true);
	GPIO_PinModeSet(PIR_SENSOR_PORT, PIR_SENSOR_2, gpioModeInputPull, true);
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

