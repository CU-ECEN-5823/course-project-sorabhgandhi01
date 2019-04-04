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
//		if ((GPIO_PinInGet(PB0_BUTTON_PORT, PB0_BUTTON_PIN)) == 0) {
//			gecko_external_signal(EXT_SIGNAL_PB0_BUTTON_PRESSED);
//		}
//		else if ((GPIO_PinInGet(PB0_BUTTON_PORT, PB0_BUTTON_PIN)) == 1) {
//			gecko_external_signal(EXT_SIGNAL_PB0_BUTTON_RELEASED);
//		}

		EXT_SIGNAL_PB0_BUTTON |= BUTTON_STATUS;
		gecko_external_signal(EXT_SIGNAL_PB0_BUTTON);
	}
}


void enable_button_interrupts(void)
{
  GPIOINT_Init();

  /* configure interrupt for PB0 and PB1, both falling and rising edges */
  GPIO_ExtIntConfig(PB0_BUTTON_PORT, PB0_BUTTON_PIN, PB0_BUTTON_PIN, true, true, true);

  /* register the callback function that is invoked when interrupt occurs */
  GPIOINT_CallbackRegister(PB0_BUTTON_PIN, gpioint);
}

