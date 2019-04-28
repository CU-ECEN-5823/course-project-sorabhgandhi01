/*
 * gpio.h
 *
 *  Created on: April 5, 2019
 *      Author: Sorabh Gandhi
 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include <stdbool.h>
#include <stdint.h>

#include "em_core.h"

#define	LED0_port gpioPortF
#define PB0_BUTTON_PORT gpioPortF
#define PB1_BUTTON_PORT gpioPortF
#define IR_SENSOR_PORT	gpioPortD


#define LED0_pin	4
#define PB0_BUTTON_PIN 6
#define PB1_BUTTON_PIN 7

//PD11
#define IR_SENSOR_1_PIN	11		//(Pin 9)
//PD12
#define IR_SENSOR_2_PIN	12		//(Pin 11)

#define SENSOR_1_STATUS 0x60
#define SENSOR_2_STATUS 0x80

uint8_t EXT_SIGNAL_SENSOR_1;		//Notification flag for Sensor 1
uint8_t EXT_SIGNAL_SENSOR_2;		//Notification flag for Sensor 2

void gpioInit();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();

void gpioEnableDisplay();
void gpioSetDisplayExtcomin(bool high);

void enable_sensor_interrupts(void);
void disable_sensor_interrupts(void);
void right_sensor_int(uint8_t pin);
void left_sensor_int(uint8_t pin);

#endif /* SRC_GPIO_H_ */
