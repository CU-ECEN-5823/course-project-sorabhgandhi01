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

#include "em_core.h"

#define PB0_BUTTON_PORT gpioPortF
#define PB1_BUTTON_PORT gpioPortF
#define IR_SENSOR_PORT	gpioPortD


#define PB0_BUTTON_PIN 6
#define PB1_BUTTON_PIN 7

//PD11
#define IR_SENSOR_1_PIN	11		//(Pin 9)
//PD12
#define IR_SENSOR_2_PIN	12		//(Pin 11)

//#define GPIO_SET_DISPLAY_EXT_COMIN_IMPLEMENTED 	1
//#define GPIO_DISPLAY_SUPPORT_IMPLEMENTED		1

#define EXT_SIGNAL_PB0_BUTTON_PRESSED 0x01
#define EXT_SIGNAL_PB0_BUTTON_RELEASED 0x02

#define PB0_BUTTON_STATUS 0x20
#define PB1_BUTTON_STATUS 0x40
#define SENSOR_1_STATUS 0x60
#define SENSOR_2_STATUS 0x80

uint8_t EXT_SIGNAL_PB0_BUTTON;
uint8_t EXT_SIGNAL_PB1_BUTTON;
uint8_t EXT_SIGNAL_SENSOR_1;
uint8_t EXT_SIGNAL_SENSOR_2;

uint8_t right_sensor_active;
uint8_t left_sensor_active;

void gpioInit();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();

void gpioEnableDisplay();
void gpioSetDisplayExtcomin(bool high);

void enable_button_interrupts(void);
void gpioint(uint8_t pin);

void enable_sensor_interrupts(void);
void right_sensor_int(uint8_t pin);
void left_sensor_int(uint8_t pin);

#endif /* SRC_GPIO_H_ */
