/*
 * main.h
 *
 *  Created on: 26-Apr-2019
 *      Author: Sorabh
 */

#ifndef SRC_MAIN_H_
#define SRC_MAIN_H_

/// Timer Frequency used
#define TIMER_CLK_FREQ ((uint32)32768)
/// Convert miliseconds to timer ticks
#define TIMER_MS_2_TIMERTICK(ms) ((TIMER_CLK_FREQ * ms) / 1000)


/*******************************************************************************
 * Timer handles defines.
 ******************************************************************************/
#define TIMER_ID_RESTART          78
#define TIMER_ID_FACTORY_RESET    77
#define TIMER_ID_PROVISIONING     66
#define TIMER_ID_RETRANS          10
#define TIMER_ID_FRIEND_FIND      20
#define TIMER_ID_NODE_CONFIGURED  30
#define TIMER_ID_SAVE_STATE		60
#define TIMER_ID_INT_RESET		15
#define TIMER_ID_SPRAY_START	04
#define TIMER_ID_SPRAY_RESET	05
#define DISPLAY_UPDATE			02
#define LOGGER_UPDATE			03

#endif /* SRC_MAIN_H_ */
