/*
 * gpio.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include <stdbool.h>
#include <stdint.h>

#define PB0_BUTTON_PORT gpioPortF
#define PB0_BUTTON_PIN 6
#define PB1_BUTTON_PORT gpioPortF
#define PB1_BUTTON_PIN 7

//#define GPIO_SET_DISPLAY_EXT_COMIN_IMPLEMENTED 	1
//#define GPIO_DISPLAY_SUPPORT_IMPLEMENTED		1

#define EXT_SIGNAL_PB0_BUTTON_PRESSED 0x01
#define EXT_SIGNAL_PB0_BUTTON_RELEASED 0x02
#define BUTTON_STATUS 0x20

uint8_t EXT_SIGNAL_PB0_BUTTON;

void gpioInit();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();

void gpioEnableDisplay();
void gpioSetDisplayExtcomin(bool high);

void enable_button_interrupts(void);
void gpioint(uint8_t pin);

#endif /* SRC_GPIO_H_ */
