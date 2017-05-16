// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "bt.h"
#include "bta_api.h"
#include "esp_deep_sleep.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "counter_task.h"

#include "sdkconfig.h"

#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

#define PROFILE_NUM 		    	    1
#define PROFILE_APP_IDX 	     		0
#define APP_ID          		    	0
#define SVC_INST_ID	                	0

#define SAMPLE_DEVICE_NAME "Sensor"

// Bluetooth Core Spec v5.0, Vol3.C.11
static uint8_t adv_rsp_data_raw[] = {
    2, //Length
    BTM_BLE_AD_TYPE_FLAG, // AD Type : Flags
    (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), //Data
    7, //Length
    BTM_BLE_AD_TYPE_NAME_CMPL,
    'S', 'e', 'n', 's', 'o', 'r',
    3,
    BTM_BLE_AD_TYPE_16SRV_PART,
    0x44, 0x00 // UUID 16 bits
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },

};

#define SVC_DECLARATION_SIZE    (sizeof(uint16_t))
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
#define CHAR_PROP_SIZE          (sizeof(uint16_t))

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;

static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t character_client_desc_uuid = ESP_GATT_UUID_CHAR_DESCRIPTION;

static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE|ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_not = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

#define ATT_TABLE_LEN       5

// ATT/Handle indexes
#define SVC_DECLARATION     0
#define CHAR_DECLARATION    1
#define CHAR_VALUE          2
#define CHAR_CCC            3
#define CHAR_CUD            4


#define UUID_SVC 0x4400
static const uint16_t svc = UUID_SVC;
static const uint16_t counter_uuid = UUID_SVC+1;
static uint16_t counter_value = 0;
static uint16_t counter_ccc = 0;
static const char counter_desc[] = "counter";

uint16_t handle_table[ATT_TABLE_LEN];

static const esp_gatts_attr_db_t custom_att_table[ATT_TABLE_LEN] =
{
    // Service declaration
    {{ESP_GATT_AUTO_RSP},
    {
      // UUID Length (ESP_UUID_LEN_16, ESP_UUID_LEN_32, ESP_UUID_LEN_128)
      ESP_UUID_LEN_16,
      // UUID ptr (ESP_GATT_UUID_PRI_SERVICE, ESP_GATT_UUID_SEC_SERVICE, ESP_GATT_UUID_INCLUDE_SERVICE)
      (uint8_t *)&primary_service_uuid,
      // Permission (READ, READ_ENCRYPTED, READ_ENC_MITM, WRITE, WRITE_ENCRYPTED, WRITE_ENC_MITM, WRITE_SIGNED, WRITE_SIGNED_MITM)
      ESP_GATT_PERM_READ,
      // Maximum lenght of the element value
      SVC_DECLARATION_SIZE,
      // Current lenght of the element value
      SVC_DECLARATION_SIZE,
      // Ptr to the element value (Service UUID)
      (uint8_t *)&svc
    }},
        // Characteristic declaration
        {{ESP_GATT_AUTO_RSP},
        {
          // UUID Length
          ESP_UUID_LEN_16,
          // UUID ptr
          (uint8_t *)&character_declaration_uuid,
          // Permission
          ESP_GATT_PERM_READ,
          // Maximum lenght of the element value
          CHAR_DECLARATION_SIZE,
          // Current lenght of the element value
          CHAR_DECLARATION_SIZE,
          // Ptr to the element value (Characteristic properties (extendable))
          // (BROADCAST, READ, WRITE_NR, WRITE, NOTIFY, INDICATE, AUTH, EXT_PROP)
          (uint8_t *)&char_prop_read_not
        }},
            // Characteristic Value
            {{ESP_GATT_AUTO_RSP},
            {
              // UUID Length
              ESP_UUID_LEN_16,
              // UUID ptr (Characteristic UUID)
              (uint8_t *)&counter_uuid,
              // Permission
              ESP_GATT_PERM_READ,
              // Maximum lenght of the element value
              CHAR_PROP_SIZE,
              // Current lenght of the element value
              CHAR_PROP_SIZE,
              // Ptr to the element value (Characteristic properties (extendable))
              // (BROADCAST, READ, WRITE_NR, WRITE, NOTIFY, INDICATE, AUTH, EXT_PROP)
              (uint8_t *)&counter_value
            }},
            // Characteristic CCC (To enable notification or indication)
            {{ESP_GATT_AUTO_RSP},
            {
              // UUID Length
              ESP_UUID_LEN_16,
              // UUID ptr (Characteristic UUID)
              (uint8_t *)&character_client_config_uuid,
              // Permission
              ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
              // Maximum lenght of the element value
              2,
              // Current lenght of the element value
              2,
              // Ptr to the element value (CCC properties)
              // (NOTIFY, INDICATE)
              (uint8_t *)&counter_ccc
            }},
            // Characteristic User Description
            {{ESP_GATT_AUTO_RSP},
            {
              // UUID Length
              ESP_UUID_LEN_16,
              // UUID ptr (Characteristic UUID)
              (uint8_t *)&character_client_desc_uuid,
              // Permission
              ESP_GATT_PERM_READ,
              // Maximum lenght of the element value
              sizeof(counter_desc),
              // Current lenght of the element value
              sizeof(counter_desc),
              // Ptr to the element value (User-friendly name for characteristic)
              (uint8_t *)counter_desc
            }}
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGE(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed\n");
        }
				else ESP_LOGE(GATTS_TABLE_TAG, "Advertising started\n");
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed\n");
        }
        else {
            ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
        }
		break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
										   esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGE(GATTS_TABLE_TAG, "GATT event = %x\n", event);
    switch (event) {
    	case ESP_GATTS_REG_EVT:
        esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);

		esp_ble_gatts_create_attr_tab(custom_att_table, gatts_if,
								ATT_TABLE_LEN, SVC_INST_ID);
       	break;
    	case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "Read occured\n");
       	 break;
    	case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "Write occured\n");
      	break;
    	case ESP_GATTS_EXEC_WRITE_EVT:
		break;
    	case ESP_GATTS_MTU_EVT:
		break;
   	 case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "Confirm %d\n", param->conf.status);
		break;
    	case ESP_GATTS_UNREG_EVT:
        	break;
    	case ESP_GATTS_DELETE_EVT:
        	break;
    	case ESP_GATTS_START_EVT:
        	break;
    	case ESP_GATTS_STOP_EVT:
        	break;
    	case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "Connection Done\n");

            profile_tab[PROFILE_APP_IDX].conn_id = param->connect.conn_id;

            esp_ble_gap_stop_advertising();
        	break;
    	case ESP_GATTS_DISCONNECT_EVT:
			ESP_LOGI(GATTS_TABLE_TAG, "Disconnection Done\n");

            esp_ble_gap_start_advertising(&adv_params);
		break;
    	case ESP_GATTS_OPEN_EVT:
		break;
    	case ESP_GATTS_CANCEL_OPEN_EVT:
		break;
    	case ESP_GATTS_CLOSE_EVT:
		break;
    	case ESP_GATTS_LISTEN_EVT:
		break;
    	case ESP_GATTS_CONGEST_EVT:
		break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
        esp_ble_gap_config_adv_data_raw(adv_rsp_data_raw, sizeof(adv_rsp_data_raw));
        esp_ble_gap_config_scan_rsp_data_raw(adv_rsp_data_raw, sizeof(adv_rsp_data_raw));
		ESP_LOGE(GATTS_TABLE_TAG, "The number handle =%x\n",param->add_attr_tab.num_handle);
		if(param->add_attr_tab.num_handle == ATT_TABLE_LEN){
			memcpy(handle_table, param->add_attr_tab.handles,
					sizeof(handle_table));
			esp_ble_gatts_start_service(handle_table[SVC_DECLARATION]);
		}

		break;
	}

    default:
        break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
									esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d\n", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    // Redirect event to the correct handler
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == profile_tab[idx].gatts_if) {
                if (profile_tab[idx].gatts_cb) {
                    profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

static void counterTask() {
    for(;;) {
        // Delay 1s
        vTaskDelay(1000 / portTICK_RATE_MS);
        counter_value++;

        // Notify client (Note : put last arg to true to send indication)
        esp_ble_gatts_send_indicate(profile_tab[PROFILE_APP_IDX].gatts_if,
                                    profile_tab[PROFILE_APP_IDX].conn_id,
                                    handle_table[CHAR_VALUE],
                                    2, (uint8_t*)&counter_value,
                                    false);
    }
}

void app_main()
{
    esp_err_t ret;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed\n", __func__);
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed\n", __func__);
        return;
    }

    // Start counting task
    static TaskHandle_t ct_task;
    xTaskCreate(&counterTask, "counterTask", 2048, NULL, 6, &ct_task);

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(APP_ID);
    return;
}
