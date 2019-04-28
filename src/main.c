
/***************************************************************************//**
 * @Author	Sorabh Gandhi
 * @Date	04/27/2019
 * @file main.c
 * @brief Bluetooth Mesh Low Power Node
 * This is a LPN implementation that publishes the "people count" and Controls
 * the sprayer based on the subscribed message from the Friend Node. This implementation
 * uses two modes - Level model to publish people count and ON/OFF model to
 * Subscribe the Sprayer frequency
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 *****************************************************************************
 *	@Reference	https://www.silabs.com/community/mcu/32-bit/forum.topic.html/prs_and_le_periphera-2AhC - Bluetooth SDK example - soc-thermometer
 *				https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf - Temperature data sheet
 *				https://www.silabs.com/documents/login/quick-start-guides/qsg139-getting-started-with-bluetooth.pdf - GATT Documentation
 *				https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.service.health_thermometer.xml
 *				https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.temperature_measurement.xml
 *				https://www.silabs.com/documents/login/reference-manuals/bluetooth-api-reference.pdf - API Reference Manual
 *
 *				https://www.silabs.com/community/wireless/bluetooth/knowledge-base.entry.html/2017/05/02/how_to_save_floatva-Udm8 - Reference for PS Data
 *
 *				soc-btmesh-light -- SDK Example Project
 *				soc-btmesh-switch -- SDK Example Project
 *
 *	@Configuration Method - PERIOD, LED_ON_TIME and ENERGY_MODE are defined in main.h
 ******************************************************************************/


/*********************************************************************************
 * 						System Headers
 ********************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/*********************************************************************************
 * 						SDK Headers
 ********************************************************************************/
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <gecko_configuration.h>
#include "mesh_generic_model_capi_types.h"
#include "mesh_lighting_model_capi_types.h"
#include "mesh_lib.h"
#include <mesh_sizes.h>

#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_core.h"
#include "em_timer.h"

/*********************************************************************************
 * 						Own Headers
 ********************************************************************************/
#include "log.h"
#include "gpio.h"
#include "display.h"
#include "ble_mesh_device_type.h"
#include "main.h"


/*********************************************************************************
 * 						Structures and Definitions
 ********************************************************************************/
#define PS_SAVE_IN_COUNT(in_count)	ps_save_object(0x5001, &in_count, sizeof(in_count))
#define PS_SAVE_OUT_COUNT(out_count)	ps_save_object(0x5002, &out_count, sizeof(out_count))
#define PS_SAVE_PEOPLE_COUNT(people_count)	ps_save_object(0x5000, &people_count, sizeof(people_count))



/*********************************************************************************
 * 							Helper Functions
 ********************************************************************************/
extern void gecko_main_init();
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);
extern void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
void handle_gecko_my_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
void set_device_name(bd_addr *pAddr);
void lpn_node_init(void);
void lpn_init(void);
void calculate_peope_count(void);
void publish_sensor_data(void);
uint16_t ps_save_object(uint16_t key, void *pValue, uint8_t size);
uint16_t ps_load_object(uint16_t key, void *pValue, uint8_t size);


/*********************************************************************************
 * 							GLOBAL VARIABLES
 ********************************************************************************/

/// For indexing elements of the node (this example has only one element)
static uint16 _elem_index = 0xffff;

/// Flag for indicating that lpn feature is active
static uint8 lpn_active = 0;

/// number of active Bluetooth connections
static uint8 num_connections = 0;

/// handle of the last opened LE connection
static uint8 conn_handle = 0xFF;

//people count calculated through the logic
static uint16 people_count = 0;

//Holds the current Entry Count
static uint16 in_count = 0;

//Holds the current Exit Count
static uint16 out_count = 0;


extern uint8_t EXT_SIGNAL_PB0_BUTTON;

extern uint8_t EXT_SIGNAL_SENSOR_1;

extern uint8_t EXT_SIGNAL_SENSOR_2;

extern uint8_t right_sensor_active;

extern uint8_t left_sensor_active;


/**********************************************************************************
 * 								MAIN LOOP
 *********************************************************************************/
int main(void)
{

  // Initialize stack
  gecko_main_init();

  // Initialize the logger
  logInit();

  //Initialize the GPIO
  gpioInit();

  //Initialize the display
  displayInit();


  /* Infinite loop */
  while (1) {
	struct gecko_cmd_packet *evt = gecko_wait_event();
	bool pass = mesh_bgapi_listener(evt);
	if (pass) {
		handle_gecko_my_event(BGLIB_MSG_ID(evt->header), evt);
	}
  };
}

void handle_gecko_my_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
	uint16 result;

	//handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
	if (evt == NULL) {
		return;
	}

	switch (evt_id) {
		case gecko_evt_system_boot_id:

			/* Check for PB0 or PB1 pressed */
			 if ((GPIO_PinInGet(PB0_BUTTON_PORT, PB0_BUTTON_PIN) == 0) || \
					 (GPIO_PinInGet(PB1_BUTTON_PORT, PB1_BUTTON_PIN) == 0)) {

				 gecko_cmd_flash_ps_erase_all();		//Factory Reset
				 gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);	//Set a 1 second timer to reset
				 displayPrintf(DISPLAY_ROW_ACTION, "Factory Reset");
			 } else {
				 struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();
				 gecko_cmd_hardware_set_soft_timer(1 * 32768, DISPLAY_UPDATE, 0);		//Update the display every 1 second
				 gecko_cmd_hardware_set_soft_timer(30 * 32768, TIMER_ID_SPRAY_START, 0);	// Configure the Srayer to trigger at every 30 Seconds
				 set_device_name(&pAddr->address);		//Display the BT address on the Screen
				 gecko_cmd_mesh_node_init()->result;	//Initialize the Mesh stack

				 /* Load the previously stored People Count, In Count and Out Count */
				 if((ps_load_object(0x5000, &people_count, sizeof(people_count)) == 0) &&
						 (ps_load_object(0x5001, &in_count, sizeof(in_count)) == 0) &&
						 (ps_load_object(0x5002, &out_count, sizeof(out_count)) == 0))
				 {
					 LOG_INFO("PS DATA Loaded Properly\r\n");
					 displayPrintf(DISPLAY_ROW_TEMPVALUE, "Count = %d", people_count);

				 } else {
					 LOG_INFO("Error in Loading PS DATA\r\n");
				 }
			 }
			break;

		case gecko_evt_hardware_soft_timer_id:

			switch (evt->data.evt_hardware_soft_timer.handle) {
				case TIMER_ID_FACTORY_RESET:
					gecko_cmd_system_reset(0);
					break;

				 case DISPLAY_UPDATE:		//Update the Display
					displayUpdate();
					break;

				 case LOGGER_UPDATE:		//Increment the timestamp for the Logger
					count += 10;
					break;

				 case TIMER_ID_RESTART:
					gecko_cmd_system_reset(0);
					break;

				 case TIMER_ID_INT_RESET:		//Enable the sensor Interrupts
					enable_sensor_interrupts();
				 	break;

				 case TIMER_ID_NODE_CONFIGURED:
					 if (!lpn_active) {
						 LOG_INFO("try to initialize lpn...\r\n");
						 lpn_init();
					 }
					 break;

				 case TIMER_ID_FRIEND_FIND:		//Scan for a friend node in the network to establish friendship
				 	 {
				 		LOG_INFO("trying to find friend...\r\n");
				 		result = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

				 		if (result != 0) {
				 			LOG_INFO("ret.code %x\r\n", result);
				 		}
				 	 }
			        break;

				 case TIMER_ID_SPRAY_START:		//Trigger the Sprayer ON (Emulated by an On-Board LED)
					 gpioLed0SetOn();
					 break;

				 case TIMER_ID_SPRAY_RESET:		//Turn Off the Sprayer (Emulated by an On-Board LED)
					 gpioLed0SetOff();
					 break;

				 default:
					 break;
			}

			break;


		case gecko_evt_mesh_node_initialized_id:
			LOG_INFO("In gecko_evt_mesh_node_initialized_id event\n");
			uint16_t result;

			// Initialize generic client models
			result = gecko_cmd_mesh_generic_server_init()->result;
			if (result) {
				LOG_INFO("Mesh server init failed with code = %x", result);
			}

			struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

			if ((pData->provisioned)) {
				LOG_INFO("node is provisioned. address:%x, ivi:%ld\r\n", pData->address, pData->ivi);
				_elem_index = 0;
				enable_sensor_interrupts();
				lpn_node_init();
			}
			else {
				gecko_cmd_mesh_node_start_unprov_beaconing(0x3);
			}
			break;

		case gecko_evt_mesh_node_provisioning_started_id:
			displayPrintf(DISPLAY_ROW_ACTION, "Provisioning");
			break;

		case gecko_evt_mesh_node_provisioned_id:
			_elem_index = 0;
			lpn_node_init();
			displayPrintf(DISPLAY_ROW_ACTION, "Provisioned");
			break;

		case gecko_evt_mesh_node_provisioning_failed_id:
			LOG_INFO("provisioning failed, code %x\r\n", evt->data.evt_mesh_node_provisioning_failed.result);
			displayPrintf(DISPLAY_ROW_ACTION, "Provisioned Failed");
			gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_RESTART, 1);
			break;


		case gecko_evt_le_connection_opened_id:
			LOG_INFO("evt:gecko_evt_le_connection_opened_id\r\n");
			num_connections++;
			conn_handle = evt->data.evt_le_connection_opened.connection;
			displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");
			break;

		case gecko_evt_le_connection_closed_id:

			LOG_INFO("evt:conn closed, reason 0x%x\r\n", evt->data.evt_le_connection_closed.reason);
			conn_handle = 0xFF;
			if (num_connections > 0) {
				if (--num_connections == 0) {
					displayPrintf(DISPLAY_ROW_CONNECTION, "");
					// initialize lpn when there is no active connection
					lpn_init();
				}
			}

			break;

		case gecko_evt_mesh_node_reset_id:
			if (conn_handle != 0xFF) {
			    gecko_cmd_le_connection_close(conn_handle);
			}
			gecko_cmd_flash_ps_erase_all();
			gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);

			break;

		case gecko_evt_mesh_lpn_friendship_established_id:
			LOG_INFO("friendship established\r\n");
			displayPrintf(DISPLAY_ROW_CONNECTION, "LPN with friend");
		    break;

		case gecko_evt_mesh_lpn_friendship_failed_id:
			LOG_INFO("friendship failed\r\n");
			displayPrintf(DISPLAY_ROW_CONNECTION, "no friend");
			// try again in 2 seconds
			result = gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(2000),
		                                                 TIMER_ID_FRIEND_FIND,
		                                                 1)->result;
			if (result) {
				LOG_INFO("timer failure?!  %x\r\n", result);
			}
			break;

		case gecko_evt_mesh_lpn_friendship_terminated_id:
			LOG_INFO("friendship terminated\r\n");
			displayPrintf(DISPLAY_ROW_CONNECTION, "friend lost");
			if (num_connections == 0) {
				// try again in 2 seconds
				result = gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(2000),
		                                                   TIMER_ID_FRIEND_FIND,
		                                                   1)->result;
				if (result) {
					LOG_INFO("timer failure?!  %x\r\n", result);
				}
			}
			break;

		case gecko_evt_mesh_generic_server_client_request_id:
			LOG_DEBUG("Reading the subscribed data\r\n");

			uint8_t req_type;
			uint16_t data;

			req_type = evt->data.evt_mesh_generic_server_client_request.type;
			data = evt->data.evt_mesh_generic_server_client_request.parameters.data[0];

			LOG_INFO("Subscribed value = %d; data = %d\r\n", req_type, data);
			/*Tigger the Sprayer for "ON" Request*/
			if (data == 1) {
				gpioLed0SetOn();
			}
			break;

		case gecko_evt_mesh_generic_server_state_changed_id:
			mesh_lib_generic_server_event_handler(evt);
			break;

		case gecko_evt_gatt_server_user_write_request_id:

			break;

		case gecko_evt_system_external_signal_id:
			/*Right Sensor Event Triggered*/
			if ((evt->data.evt_system_external_signal.extsignals) & SENSOR_1_STATUS) {
				CORE_DECLARE_IRQ_STATE;
				CORE_ENTER_CRITICAL();
				LOG_INFO("S1 = %d\t S2 = %d\r\n", GPIO_PinInGet(IR_SENSOR_PORT, IR_SENSOR_1_PIN), GPIO_PinInGet(IR_SENSOR_PORT, IR_SENSOR_2_PIN));

				/*Check if both the sensors are triggered*/
				if ((GPIO_PinInGet(IR_SENSOR_PORT, IR_SENSOR_1_PIN) == 0) &&
						(GPIO_PinInGet(IR_SENSOR_PORT, IR_SENSOR_2_PIN) == 0)) {
					disable_sensor_interrupts();		//Disable both the sensor Interrupts
					LOG_INFO("Out count = %d\r\n", (++out_count));
					PS_SAVE_OUT_COUNT(out_count);		//Save the Updated Exit Count
					calculate_peope_count();			//Calculate and Publish the Count to Friend Node
					gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_INT_RESET, 1);	//Enable the Interrupt after 2 seconds
				}
				EXT_SIGNAL_SENSOR_1 &= ~(SENSOR_1_STATUS);		//Reset the INT signal flag
				CORE_EXIT_CRITICAL();
			}

			/*Left Sensor Event Triggered*/
			if ((evt->data.evt_system_external_signal.extsignals) & SENSOR_2_STATUS) {
				CORE_DECLARE_IRQ_STATE;
				CORE_ENTER_CRITICAL();
				LOG_INFO("S2 = %d\t S1 = %d\r\n", GPIO_PinInGet(IR_SENSOR_PORT, IR_SENSOR_2_PIN), GPIO_PinInGet(IR_SENSOR_PORT, IR_SENSOR_1_PIN));

				/*Check if both the sensors are triggered*/
				if((GPIO_PinInGet(IR_SENSOR_PORT, IR_SENSOR_1_PIN) == 0) &&
						(GPIO_PinInGet(IR_SENSOR_PORT, IR_SENSOR_2_PIN) == 0)) {

					disable_sensor_interrupts();				//Disable both the sensor Interrupts
					LOG_INFO("In count = %d\r\n", (++in_count));
					PS_SAVE_IN_COUNT(in_count);		//Save the Updated Entry Count
					calculate_peope_count();		//Calculate and Publish the Count to Friend Node
					gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_INT_RESET, 1);	//Enable the Interrupt after 2 seconds
				}
				EXT_SIGNAL_SENSOR_2 &= ~(SENSOR_2_STATUS);		//Reset the INT signal flag
				CORE_EXIT_CRITICAL();
			}

			break;
	}
}


/***************************************************************************
 * set_device_name
 * *************************************************************************
 * @brief	This function sets the device name and shows the Bluetooth address
 * 			on the Display Screen
 *
 * @param	none
 *
 * @result	none
 *
 */
void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16 res;

  sprintf(name, "5823LPN %02x:%02x", pAddr->addr[1], pAddr->addr[0]);

  displayPrintf(DISPLAY_ROW_NAME, "%s", name);
  displayPrintf(DISPLAY_ROW_BTADDR, "%x:%x:%x:%x:%x:%x", pAddr->addr[0], \
		  pAddr->addr[1], \
		  pAddr->addr[2], \
		  pAddr->addr[3], \
		  pAddr->addr[4], \
		  pAddr->addr[5]);

  // write device name to the GATT database
  gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name)->result;
}


/***************************************************************************
 * lpn_node_init
 * *************************************************************************
 * @brief	This function initializes all the models provisioned and initializes
 * 			the low power node functionality
 *
 * @param	none
 *
 * @result	none
 *
 */
void lpn_node_init()
{
	mesh_lib_init(malloc, free, 8);
	lpn_init();
}



/***************************************************************************
 * lpn_init
 * *************************************************************************
 * @brief	This function initializes LPN functionality with configuration
 * 			and friendship establishment
 *
 * @param	none
 *
 * @result	none
 *
 */
void lpn_init(void)
{
	uint16 result;

	// Do not initialize LPN if lpn is currently active
	// or any GATT connection is opened
	if (lpn_active || num_connections) {
		return;
	}

	// Initialize LPN functionality.
	result = gecko_cmd_mesh_lpn_init()->result;
	if (result) {
		LOG_INFO("LPN init failed (0x%x)\r\n", result);
		return;
	}

	lpn_active = 1;
	LOG_INFO("LPN initialized\r\n");
	displayPrintf(DISPLAY_ROW_BTADDR2, "LPN on");

	// Configure the lpn with following parameters:
	// - Minimum friend queue length = 2
	// - Poll timeout = 5 seconds
	result = gecko_cmd_mesh_lpn_configure(2, 5 * 1000)->result;
	if (result) {
		LOG_INFO("LPN conf failed (0x%x)\r\n", result);
		return;
	}

	LOG_DEBUG("trying to find friend...\r\n");
	result = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

	if (result != 0) {
		LOG_INFO("ret.code %x\r\n", result);
	}
}


/***************************************************************************//**
 * Deinitialize LPN functionality.
 ******************************************************************************/
void lpn_deinit(void)
{
	uint16 result;

	if (!lpn_active) {
		return; // lpn feature is currently inactive
		}

	result = gecko_cmd_hardware_set_soft_timer(0, // cancel friend finding timer
                                             TIMER_ID_FRIEND_FIND,
                                             1)->result;

	// Terminate friendship if exist
	result = gecko_cmd_mesh_lpn_terminate_friendship()->result;
	if (result) {
		LOG_INFO("Friendship termination failed (0x%x)\r\n", result);
	}

	// turn off lpn feature
	result = gecko_cmd_mesh_lpn_deinit()->result;
	if (result) {
		LOG_INFO("LPN deinit failed (0x%x)\r\n", result);
	}

	lpn_active = 0;
	LOG_INFO("LPN deinitialized\r\n");
	displayPrintf(DISPLAY_ROW_BTADDR2, "LPN off");
}




/***************************************************************************//**
 * Handling of short button presses (less than 0.25s).
 * This function is called from the main loop when application receives
 * event gecko_evt_system_external_signal_id.
 *
 * @param[in] button  Defines which button was pressed,
 *                    possible values are 0 = PB0, 1 = PB1.
 *
 * @note This function is called from application context (not ISR)
 *       so it is safe to call BGAPI functions
 ******************************************************************************/
void calculate_peope_count(void)
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  if (in_count >= out_count) {
	  people_count = (in_count - out_count);
  } else {
	  people_count = 0;
	  in_count = 0;
	  out_count = 0;
  }
  PS_SAVE_PEOPLE_COUNT(people_count);
  PS_SAVE_IN_COUNT(in_count);
  PS_SAVE_OUT_COUNT(out_count);
  displayPrintf(DISPLAY_ROW_TEMPVALUE, "Count = %d", people_count);
  CORE_EXIT_CRITICAL();

  LOG_INFO("people_count %d\r\n", people_count);

  /* send the request (lightness request is sent only once in this example) */
  //send_lightness_request(0);
  publish_sensor_data();
}


/***************************************************************************
 * publish_sensor_data
 * *************************************************************************
 * @brief	This function publishes the people count to the friend node
 *
 * @param	none
 *
 * @result	none
 *
 */
void publish_sensor_data(void)
{
	struct mesh_generic_state sensor_data;
	errorcode_t resp;

	sensor_data.kind = mesh_generic_state_level;
	sensor_data.level.level = people_count;

	resp = mesh_lib_generic_server_update(MESH_GENERIC_LEVEL_SERVER_MODEL_ID,
				  	  	  	  	  	  	  	  	_elem_index,
		                                        &sensor_data,
		                                        0,
		                                        0);
	if (resp) {
		LOG_INFO("Generic server update failed, code = %x\r\n", resp);
	} else {
		LOG_INFO("Generic server updated\r\n");

		resp = mesh_lib_generic_server_publish(MESH_GENERIC_LEVEL_SERVER_MODEL_ID,
														_elem_index,
														mesh_generic_state_level);

		if (resp) {
			LOG_INFO("Publish failed, code = %x\r\n", resp);
		} else {
			LOG_INFO("Published sensor data, code = %x\r\n", resp);
		}
	}

}


uint16_t ps_save_object(uint16_t key, void *pValue, uint8_t size)
{
	struct gecko_msg_flash_ps_save_rsp_t *pResp;

	pResp = gecko_cmd_flash_ps_save(key, size, pValue);

	return(pResp->result);
}

uint16_t ps_load_object(uint16_t key, void *pValue, uint8_t size)
{
	struct gecko_msg_flash_ps_load_rsp_t *pResp;

	pResp = gecko_cmd_flash_ps_load(key);

	if(pResp->result == 0)
	{
		memcpy(pValue, pResp->value.data, pResp->value.len);

		// sanity check: length of data stored in PS key must match the expected value
		if(size != pResp->value.len)
		{
			return(bg_err_unspecified);
		}
	}

	return(pResp->result);
}
