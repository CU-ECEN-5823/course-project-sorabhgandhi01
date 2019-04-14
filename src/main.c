
/***************************************************************************//**
 * @Author	Sorabh Gandhi
 * @Date	04/03/2019
 * @file main.c
 * @brief Bluetooth Mesh Publisher and Subscriber
 * This Thermometer allows to measure temperature using the temperature sensor SI7021,
 * from a mobile app.
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
 *				https://www.silabs.com/documents/login/reference-manuals/bluetooth-api-reference.pdf - API Refernce Manual
 *
 *				soc-btmesh-light -- SDK Example Prokect
 *				soc-btmesh-switch -- SDK Example Prokect
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
#include "native_gecko.h"
#include "mesh_generic_model_capi_types.h"
#include "gatt_db.h"
#include "mesh_lib.h"
#include <gecko_configuration.h>
#include <mesh_sizes.h>

/*********************************************************************************
 * 						Own Headers
 ********************************************************************************/
#include "log.h"
#include "gpio.h"
#include "display.h"
#include "ble_mesh_device_type.h"
//#include "timer.h"

/*********************************************************************************
 * 						Structures and Definitions
 ********************************************************************************/
/// Flag for indicating DFU Reset must be performed
//uint8_t boot_to_dfu = 0;

/// Maximum number of simultaneous Bluetooth connections
#define MAX_CONNECTIONS 2

/// Heap for Bluetooth stack
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + BTMESH_HEAP_SIZE + 1760];

bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);

/// Bluetooth advertisement set configuration
///
/// At minimum the following is required:
/// * One advertisement set for Bluetooth LE stack (handle number 0)
/// * One advertisement set for Mesh data (handle number 1)
/// * One advertisement set for Mesh unprovisioned beacons (handle number 2)
/// * One advertisement set for Mesh unprovisioned URI (handle number 3)
/// * N advertisement sets for Mesh GATT service advertisements
/// (one for each network key, handle numbers 4 .. N+3)
///
#define MAX_ADVERTISERS (4 + MESH_CFG_MAX_NETKEYS)

/// Priorities for bluetooth link layer operations
static gecko_bluetooth_ll_priorities linklayer_priorities = GECKO_BLUETOOTH_PRIORITIES_DEFAULT;


/// Timer Frequency used
#define TIMER_CLK_FREQ ((uint32)32768)
/// Convert miliseconds to timer ticks
#define TIMER_MS_2_TIMERTICK(ms) ((TIMER_CLK_FREQ * ms) / 1000)

#define DISPLAY_UPDATE 0x02
#define LOGGER_UPDATE 0x03

/*******************************************************************************
 * Timer handles defines.
 ******************************************************************************/
#define TIMER_ID_RESTART          78
#define TIMER_ID_FACTORY_RESET    77
#define TIMER_ID_PROVISIONING     66
#define TIMER_ID_RETRANS          10
#define TIMER_ID_FRIEND_FIND      20
#define TIMER_ID_NODE_CONFIGURED  30
#define TIMER_ID_SAVE_STATE               60
//#define SCHEDULER_SUPPORTS_DISPLAY_UPDATE_EVENT 1
//#define TIMER_SUPPORTS_1HZ_TIMER_EVENT	1

/// Bluetooth stack configuration
const gecko_configuration_t config1 =
{
#if defined(FEATURE_LFXO)
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
#else
  .sleep.flags = 0,
#endif // LFXO
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.max_advertisers = MAX_ADVERTISERS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap) - BTMESH_HEAP_SIZE,
  .bluetooth.sleep_clock_accuracy = 100,
  .bluetooth.linklayer_priorities = &linklayer_priorities,
  .gattdb = &bg_gattdb_data,
  .btmesh_heap_size = BTMESH_HEAP_SIZE,
#if (HAL_PA_ENABLE)
  .pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
  .pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
#endif // (HAL_PA_ENABLE)
  .max_timers = 16,
};

/*********************************************************************************
 * 							Helper Functions
 ********************************************************************************/
extern void gecko_main_init();
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);
extern void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
void handle_gecko_my_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
void set_device_name(bd_addr *pAddr);
void send_onoff_request(int retrans);
static void onoff_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags);
static void onoff_change(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms);
static errorcode_t onoff_update_and_publish(uint16_t element_index);
static errorcode_t onoff_update(uint16_t element_index);
void init_models();
void handle_button_state(uint8_t button);
void sorabh_node_init(void);
void lpn_init(void);

/*********************************************************************************
 * 							GLOBAL VARIABLES
 ********************************************************************************/

/// Flag for indicating DFU Reset must be performed
//uint8_t boot_to_dfu = 0;

/// current position of the switch
static uint8 switch_pos = 0;

/// number of on/off requests to be sent
static uint8 request_count;

/// transaction identifier
static uint8 trid = 0;

/// For indexing elements of the node (this example has only one element)
static uint16 _elem_index = 0xffff;

/// For indexing elements of the node
static uint16 _primary_elem_index = 0xffff;

static uint8_t connection_handle;

/// Flag for indicating that lpn feature is active
static uint8 lpn_active = 0;

/// number of active Bluetooth connections
static uint8 num_connections = 0;

/// handle of the last opened LE connection
static uint8 conn_handle = 0xFF;

extern uint8_t EXT_SIGNAL_PB0_BUTTON;

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

   gecko_bgapi_classes_init_client_lpn();
//   	 gecko_stack_init(&config1);
//     gecko_bgapi_class_dfu_init();
//     gecko_bgapi_class_system_init();
//     gecko_bgapi_class_le_gap_init();
//     gecko_bgapi_class_le_connection_init();
//     //gecko_bgapi_class_gatt_init();
//     gecko_bgapi_class_gatt_server_init();
//     gecko_bgapi_class_hardware_init();
//     gecko_bgapi_class_flash_init();
//     gecko_bgapi_class_test_init();
//     //gecko_bgapi_class_sm_init();
//     //mesh_native_bgapi_init();
//     gecko_bgapi_class_mesh_node_init();
//     //gecko_bgapi_class_mesh_prov_init();
//     gecko_bgapi_class_mesh_proxy_init();
//     gecko_bgapi_class_mesh_proxy_server_init();
//     //gecko_bgapi_class_mesh_proxy_client_init();
//     gecko_bgapi_class_mesh_generic_client_init();
//     //gecko_bgapi_class_mesh_generic_server_init();
//     //gecko_bgapi_class_mesh_vendor_model_init();
//     //gecko_bgapi_class_mesh_health_client_init();
//     //gecko_bgapi_class_mesh_health_server_init();
//     //gecko_bgapi_class_mesh_test_init();
//     gecko_bgapi_class_mesh_lpn_init();
//     //gecko_bgapi_class_mesh_friend_init();

  //Initialize timer
   //timer_Init();

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
#if DEVICE_IS_ONOFF_PUBLISHER
	switch (evt_id) {
		case gecko_evt_system_boot_id:

			 if ((GPIO_PinInGet(PB0_BUTTON_PORT, PB0_BUTTON_PIN) == 0) || \
					 (GPIO_PinInGet(PB1_BUTTON_PORT, PB1_BUTTON_PIN) == 0)) {

				 gecko_cmd_flash_ps_erase_all();
				 gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
				 displayPrintf(DISPLAY_ROW_ACTION, "Factory Reset");
			 } else {
				 struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();
				 gecko_cmd_hardware_set_soft_timer(1 * 32768, DISPLAY_UPDATE, 0);
				 gecko_cmd_hardware_set_soft_timer(328, LOGGER_UPDATE, 0);
				 set_device_name(&pAddr->address);
				 gecko_cmd_mesh_node_init()->result;
			 }
			break;

		case gecko_evt_hardware_soft_timer_id:

			switch (evt->data.evt_hardware_soft_timer.handle) {
				case TIMER_ID_FACTORY_RESET:
					gecko_cmd_system_reset(0);
					break;

				 case DISPLAY_UPDATE:
					displayUpdate();
					break;

				 case LOGGER_UPDATE:
					count++;
					break;

				 case TIMER_ID_NODE_CONFIGURED:
					 if (!lpn_active) {
						 LOG_INFO("try to initialize lpn...\r\n");
						 lpn_init();
					 }
					 break;

				 case TIMER_ID_FRIEND_FIND:
				 	 {
				 		LOG_INFO("trying to find friend...\r\n");
				 		result = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

				 		if (result != 0) {
				 			LOG_INFO("ret.code %x\r\n", result);
				 		}
				 	 }
			        break;

				 default:
					 break;
			}

			break;

		case gecko_evt_mesh_node_initialized_id:
			LOG_INFO("In gecko_evt_mesh_node_initialized_id event\n");
			//displayPrintf(DISPLAY_ROW_ACTION, "Node");

			// Initialize generic client models
			gecko_cmd_mesh_generic_client_init();

			struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

			if ((pData->provisioned)) {
				LOG_INFO("node is provisioned. address:%x, ivi:%ld\r\n", pData->address, pData->ivi);

				enable_button_interrupts();
				sorabh_node_init();

				lpn_init();
			}
			else {
				gecko_cmd_mesh_node_start_unprov_beaconing(0x3);
			}
			break;

		case gecko_evt_mesh_node_provisioning_started_id:
			displayPrintf(DISPLAY_ROW_ACTION, "Provisioning");
			break;

		case gecko_evt_mesh_node_provisioned_id:
			sorabh_node_init();

			// try to initialize lpn after 30 seconds, if no configuration messages come
			result = gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(30000),
			                                                 TIMER_ID_NODE_CONFIGURED,
			                                                 1)->result;
			if (result) {
				printf("timer failure?!  %x\r\n", result);
			}
			LOG_INFO("node provisioned, got address=%x\r\n", evt->data.evt_mesh_node_provisioned.address);
			displayPrintf(DISPLAY_ROW_ACTION, "Provisioned");
			break;

		case gecko_evt_mesh_node_provisioning_failed_id:
			LOG_INFO("provisioning failed, code %x\r\n", evt->data.evt_mesh_node_provisioning_failed.result);
			displayPrintf(DISPLAY_ROW_ACTION, "Provisioned Failed");
			gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_RESTART, 1);
			break;

		case gecko_evt_mesh_node_model_config_changed_id:
			LOG_INFO("model config changed\r\n");

			// try to init lpn 5 seconds after configuration change
			result = gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(5000),
		                                                 TIMER_ID_NODE_CONFIGURED,
		                                                 1)->result;
			if (result) {
				LOG_INFO("timer failure?!  %x\r\n", result);
			}
			break;

		case gecko_evt_le_connection_opened_id:
			LOG_INFO("evt:gecko_evt_le_connection_opened_id\r\n");
			num_connections++;
			conn_handle = evt->data.evt_le_connection_opened.connection;
			displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");
			lpn_deinit();
			break;

		case gecko_evt_le_connection_closed_id:

			 /* Check if need to boot to dfu mode */
//			if (boot_to_dfu) {
//				/* Enter to DFU OTA mode */
//				gecko_cmd_system_reset(2);
//			}

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
			gecko_cmd_flash_ps_erase_all();
			gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);

			break;

		case gecko_evt_le_connection_parameters_id:
			LOG_INFO("connection params: interval %d, timeout %d\r\n",
			             evt->data.evt_le_connection_parameters.interval,
			             evt->data.evt_le_connection_parameters.timeout);
			break;

		case gecko_evt_le_gap_adv_timeout_id:
		      // these events silently discarded
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

		case gecko_evt_system_external_signal_id:

			if ((evt->data.evt_system_external_signal.extsignals) & BUTTON_STATUS) {
				EXT_SIGNAL_PB0_BUTTON &= ~(BUTTON_STATUS);
				if(GPIO_PinInGet(gpioPortF, 6) == 0) {
					handle_button_state(0);
				}
				if(GPIO_PinInGet(gpioPortF, 6) == 1) {
					handle_button_state(1);
				}
			}

			break;
	}
#else

	switch (evt_id) {
			case gecko_evt_system_boot_id:
				 if ((GPIO_PinInGet(PB0_BUTTON_PORT, PB0_BUTTON_PIN) == 0) || \
						 (GPIO_PinInGet(PB1_BUTTON_PORT, PB1_BUTTON_PIN) == 0)) {

					 gecko_cmd_flash_ps_erase_all();
					 gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
					 displayPrintf(DISPLAY_ROW_ACTION, "Factory Reset");
				 } else {

					 struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();
					 set_device_name(&pAddr->address);
					 gecko_cmd_mesh_node_init()->result;
				 }
				break;

			case gecko_evt_hardware_soft_timer_id:

				switch (evt->data.evt_hardware_soft_timer.handle) {
					case TIMER_ID_FACTORY_RESET:
						gecko_cmd_system_reset(0);
						break;
				}

				break;

			case gecko_evt_mesh_node_initialized_id:
				 LOG_INFO("In gecko_evt_mesh_node_initialized_id event\n");
				struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

				if ((pData->provisioned)) {
					_primary_elem_index = 0;   // index of primary element is zero.
					mesh_lib_init(malloc,free,9);
					init_models();
					onoff_update_and_publish(0);
					gecko_cmd_mesh_generic_server_init();
				}
				else {
					gecko_cmd_mesh_node_start_unprov_beaconing(0x3);
				}

				break;

			case gecko_evt_mesh_node_provisioning_started_id:
				displayPrintf(DISPLAY_ROW_ACTION, "Provisioning");
				break;

			case gecko_evt_mesh_node_provisioned_id:
				displayPrintf(DISPLAY_ROW_ACTION, "Provisioned");
				_primary_elem_index = 0;   // index of primary element is zero.
				//displayPrintf(DISPLAY_ROW_CONNECTION, "SUB Init");
				mesh_lib_init(malloc,free,9);
				init_models();
				onoff_update_and_publish(0);
				gecko_cmd_mesh_generic_server_init();
				break;

			case gecko_evt_mesh_node_provisioning_failed_id:
				gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
				displayPrintf(DISPLAY_ROW_ACTION, "Provisioned Failed");
				break;

			case gecko_evt_mesh_generic_server_client_request_id:
			    mesh_lib_generic_server_event_handler(evt);
			    break;

			case gecko_evt_mesh_generic_server_state_changed_id:
			    mesh_lib_generic_server_event_handler(evt);
			    break;

			case gecko_evt_le_connection_opened_id:
				//connection_handle = evt->data.evt_le_connection_opened.connection;
				displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");
				break;

			case gecko_evt_le_connection_closed_id:
				//gecko_cmd_system_reset(2);
				displayPrintf(DISPLAY_ROW_CONNECTION, "");
				break;

			case gecko_evt_mesh_node_reset_id:
				gecko_cmd_flash_ps_erase_all();
				gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);

				break;
		}

#endif
}

void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16 res;

  // create unique device name using the last two bytes of the Bluetooth address
  if (DEVICE_USES_BLE_MESH_CLIENT_MODEL) {
	  sprintf(name, "5823PUB %02x:%02x", pAddr->addr[1], pAddr->addr[0]);
  } else {
	  sprintf(name, "5823SUB %02x:%02x", pAddr->addr[1], pAddr->addr[0]);
  }

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

void send_onoff_request(int retrans)
{
  uint16 resp;
  uint16 delay;
  struct mesh_generic_request req;
  const uint32 transtime = 0; /* using zero transition time by default */
  _elem_index = 0;

  req.kind = mesh_generic_request_on_off;
  req.on_off = switch_pos ? 0 : 1;

  // increment transaction ID for each request, unless it's a retransmission
  if (retrans == 0) {
    trid++;
  }

  /* delay for the request is calculated so that the last request will have a zero delay and each
   * of the previous request have delay that increases in 50 ms steps. For example, when using three
   * on/off requests per button press the delays are set as 100, 50, 0 ms
   */
  //delay = (request_count - 1) * 50;

  resp = mesh_lib_generic_client_publish(
    MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,
    _elem_index,
    trid,
    &req,
    transtime,   // transition time in ms
    delay,
    0     // flags
    );

  if (resp) {
	 LOG_INFO("gecko_cmd_mesh_generic_client_publish failed,code %x\r\n", resp);
  } else {
	 LOG_INFO("request sent, trid = %u \r\n", trid);
  }

  /* keep track of how many requests has been sent */
  if (request_count > 0) {
    request_count--;
  }
}

static void onoff_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags)
{
	LOG_INFO("request->on_off = %d\n\r",request->on_off);
//	if (request->on_off == 1) {
//		//LOG_INFO("");
//		displayPrintf(DISPLAY_ROW_ACTION, "PB0_BUTTON_RELEASED");
//	}
	if (request->on_off == 0) {
		//LOG_INFO("");
		displayPrintf(DISPLAY_ROW_ACTION, "PB0_BUTTON_RELEASED");
	} else {
		displayPrintf(DISPLAY_ROW_ACTION, "PB0_BUTTON_PRESSED");
	}
	onoff_update_and_publish(element_index);
}

static void onoff_change(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms)
{

}

static errorcode_t onoff_update(uint16_t element_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_generic_state_on_off;
  current.on_off.on = 0;

  target.kind = mesh_generic_state_on_off;
  target.on_off.on = 1;

  //displayPrintf(DISPLAY_ROW_ACTION, "onoff_update");

  return mesh_lib_generic_server_update(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        &target,
                                        0);
}

static errorcode_t onoff_update_and_publish(uint16_t element_index)
{
	onoff_update(element_index);

	//displayPrintf(DISPLAY_ROW_ACTION, "onoff_update_publish");

	mesh_lib_generic_server_publish(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
			element_index,
			mesh_generic_state_on_off);
}

void init_models()
{
	mesh_lib_generic_server_register_handler(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID, 0, onoff_request, onoff_change);
}

void handle_button_state(uint8_t button)
{
	if (button == 0) {
		displayPrintf(DISPLAY_ROW_ACTION, "PB0_BUTTON_PRESSED");
		switch_pos = 0;
	}
	if (button == 1) {
		displayPrintf(DISPLAY_ROW_ACTION, "PB0_BUTTON_RELEASED");
		switch_pos = 1;
	}

	send_onoff_request(0);
}

void sorabh_node_init()
{
	mesh_lib_init(malloc, free, 8);
}

/***************************************************************************//**
 * Initialize LPN functionality with configuration and friendship establishment.
 ******************************************************************************/
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

	LOG_INFO("trying to find friend...\r\n");
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
