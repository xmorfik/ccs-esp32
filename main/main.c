/* Ethernet Basic

   This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "ethernet_init.h"
#include "sdkconfig.h"
#include "mbcontroller.h"
#include "modbus_params.h"


#define MB_PORT_NUM     (CONFIG_MB_UART_PORT_NUM)   // Number of UART port used for Modbus connection
#define MB_DEV_SPEED    (CONFIG_MB_UART_BAUD_RATE)  // The communication speed of the UART

// Note: Some pins on target chip cannot be assigned for UART communication.
// See UART documentation for selected board and target to configure pins using Kconfig.

// The number of parameters that intended to be used in the particular control process
#define MASTER_MAX_CIDS num_device_parameters

// Number of reading of parameters from slave
#define MASTER_MAX_RETRY 3

// Timeout to update cid over Modbus
#define UPDATE_CIDS_TIMEOUT_MS          (500)
#define UPDATE_CIDS_TIMEOUT_TICS        (UPDATE_CIDS_TIMEOUT_MS / portTICK_PERIOD_MS)

// Timeout between polls
#define POLL_TIMEOUT_MS                 (1)
#define POLL_TIMEOUT_TICS               (POLL_TIMEOUT_MS / portTICK_PERIOD_MS)

// The macro to get offset for parameter in the appropriate structure
#define HOLD_OFFSET(field) ((uint16_t)(offsetof(holding_reg_params_t, field) + 1))
#define INPUT_OFFSET(field) ((uint16_t)(offsetof(input_reg_params_t, field) + 1))
#define COIL_OFFSET(field) ((uint16_t)(offsetof(coil_reg_params_t, field) + 1))
// Discrete offset macro
#define DISCR_OFFSET(field) ((uint16_t)(offsetof(discrete_reg_params_t, field) + 1))

#define STR(fieldname) ((const char*)( fieldname ))
// Options can be used as bit masks or parameter limits
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

static const char *TAG_MB = "MB_MASTER";
static const char *TAG_ETH = "ETHERNET";

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG_ETH, "Ethernet Link Up");
        ESP_LOGI(TAG_ETH, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_ETH, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG_ETH, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG_ETH, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG_ETH, "Ethernet Got IP Address");
    ESP_LOGI(TAG_ETH, "~~~~~~~~~~~");
    ESP_LOGI(TAG_ETH, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG_ETH, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG_ETH, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG_ETH, "~~~~~~~~~~~");
}

// Enumeration of modbus device addresses accessed by master device
enum {
    MB_DEVICE_ADDR1 = 1 // Only one slave device used for the test (add other slave addresses here)
};

// Enumeration of all supported CIDs for device (used in parameter definition table)
enum {
    CID_INP_DATA_0 = 0,
    CID_HOLD_DATA_0,
    CID_INP_DATA_1,
    CID_HOLD_DATA_1,
    CID_INP_DATA_2,
    CID_HOLD_DATA_2,
    CID_HOLD_TEST_REG,
    CID_RELAY_P1,
    CID_RELAY_P2,
    CID_COUNT
};

// Example Data (Object) Dictionary for Modbus parameters:
// The CID field in the table must be unique.
// Modbus Slave Addr field defines slave address of the device with correspond parameter.
// Modbus Reg Type - Type of Modbus register area (Holding register, Input Register and such).
// Reg Start field defines the start Modbus register number and Reg Size defines the number of registers for the characteristic accordingly.
// The Instance Offset defines offset in the appropriate parameter structure that will be used as instance to save parameter value.
// Data Type, Data Size specify type of the characteristic and its data size.
// Parameter Options field specifies the options that can be used to process parameter value (limits or masks).
// Access Mode - can be used to implement custom options for processing of characteristic (Read/Write restrictions, factory mode values and etc).
const mb_parameter_descriptor_t device_parameters[] = {
        // { CID, Param Name, Units, Modbus Slave Addr, Modbus Reg Type, Reg Start, Reg Size, Instance Offset, Data Type, Data Size, Parameter Options, Access Mode}
        { CID_INP_DATA_0, STR("Data_channel_0"), STR("Volts"), MB_DEVICE_ADDR1, MB_PARAM_INPUT, 0, 2,
                INPUT_OFFSET(input_data0), PARAM_TYPE_FLOAT, 4, OPTS( -10, 10, 1 ), PAR_PERMS_READ_WRITE_TRIGGER }
};

// Calculate number of parameters in the table
const uint16_t num_device_parameters = (sizeof(device_parameters)/sizeof(device_parameters[0]));

// The function to get pointer to parameter storage (instance) according to parameter description table
static void* master_get_param_data(const mb_parameter_descriptor_t* param_descriptor)
{
    assert(param_descriptor != NULL);
    void* instance_ptr = NULL;
    if (param_descriptor->param_offset != 0) {
        switch(param_descriptor->mb_param_type)
        {
            case MB_PARAM_HOLDING:
                instance_ptr = ((void*)&holding_reg_params + param_descriptor->param_offset - 1);
                break;
            case MB_PARAM_INPUT:
                instance_ptr = ((void*)&input_reg_params + param_descriptor->param_offset - 1);
                break;
            case MB_PARAM_COIL:
                instance_ptr = ((void*)&coil_reg_params + param_descriptor->param_offset - 1);
                break;
            case MB_PARAM_DISCRETE:
                instance_ptr = ((void*)&discrete_reg_params + param_descriptor->param_offset - 1);
                break;
            default:
                instance_ptr = NULL;
                break;
        }
    } else {
        ESP_LOGE(TAG_MB, "Wrong parameter offset for CID #%d", param_descriptor->cid);
        assert(instance_ptr != NULL);
    }
    return instance_ptr;
}

// User operation function to read slave values and check alarm
static void master_operation_func(void *arg)
{
    esp_err_t err = ESP_OK;
    float value = 0;
    bool alarm_state = false;
    const mb_parameter_descriptor_t* param_descriptor = NULL;

    ESP_LOGI(TAG_MB, "Start modbus test...");

    for(uint16_t retry = 0; retry <= MASTER_MAX_RETRY && (!alarm_state); retry++) {
        // Read all found characteristics from slave(s)
        for (uint16_t cid = 0; (err != ESP_ERR_NOT_FOUND) && cid < MASTER_MAX_CIDS; cid++)
        {
            // Get data from parameters description table
            // and use this information to fill the characteristics description table
            // and having all required fields in just one table
            err = mbc_master_get_cid_info(cid, &param_descriptor);
            if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
                void* temp_data_ptr = master_get_param_data(param_descriptor);
                assert(temp_data_ptr);
                uint8_t type = 0;
                if ((param_descriptor->param_type == PARAM_TYPE_ASCII) &&
                    (param_descriptor->cid == CID_HOLD_TEST_REG)) {
                    // Check for long array of registers of type PARAM_TYPE_ASCII
                    err = mbc_master_get_parameter(cid, (char*)param_descriptor->param_key,
                                                   (uint8_t*)temp_data_ptr, &type);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG_MB, "Characteristic #%d %s (%s) value = (0x%08x) read successful.",
                                 param_descriptor->cid,
                                 (char*)param_descriptor->param_key,
                                 (char*)param_descriptor->param_units,
                                 *(uint32_t*)temp_data_ptr);
                        // Initialize data of test array and write to slave
                        if (*(uint32_t*)temp_data_ptr != 0xAAAAAAAA) {
                            memset((void*)temp_data_ptr, 0xAA, param_descriptor->param_size);
                            *(uint32_t*)temp_data_ptr = 0xAAAAAAAA;
                            err = mbc_master_set_parameter(cid, (char*)param_descriptor->param_key,
                                                           (uint8_t*)temp_data_ptr, &type);
                            if (err == ESP_OK) {
                                ESP_LOGI(TAG_MB, "Characteristic #%d %s (%s) value = (0x%08x), write successful.",
                                         param_descriptor->cid,
                                         (char*)param_descriptor->param_key,
                                         (char*)param_descriptor->param_units,
                                         *(uint32_t*)temp_data_ptr);
                            } else {
                                ESP_LOGE(TAG_MB, "Characteristic #%d (%s) write fail, err = 0x%x (%s).",
                                         param_descriptor->cid,
                                         (char*)param_descriptor->param_key,
                                         (int)err,
                                         (char*)esp_err_to_name(err));
                            }
                        }
                    } else {
                        ESP_LOGE(TAG_MB, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                 param_descriptor->cid,
                                 (char*)param_descriptor->param_key,
                                 (int)err,
                                 (char*)esp_err_to_name(err));
                    }
                } else {
                    err = mbc_master_get_parameter(cid, (char*)param_descriptor->param_key,
                                                   (uint8_t*)&value, &type);
                    if (err == ESP_OK) {
                        *(float*)temp_data_ptr = value;
                        if ((param_descriptor->mb_param_type == MB_PARAM_HOLDING) ||
                            (param_descriptor->mb_param_type == MB_PARAM_INPUT)) {
                            ESP_LOGI(TAG_MB, "Characteristic #%d %s (%s) value = %f (0x%x) read successful.",
                                     param_descriptor->cid,
                                     (char*)param_descriptor->param_key,
                                     (char*)param_descriptor->param_units,
                                     value,
                                     *(uint32_t*)temp_data_ptr);
                            if (((value > param_descriptor->param_opts.max) ||
                                 (value < param_descriptor->param_opts.min))) {
                                alarm_state = true;
                                break;
                            }
                        } else {
                            uint16_t state = *(uint16_t*)temp_data_ptr;
                            const char* rw_str = (state & param_descriptor->param_opts.opt1) ? "ON" : "OFF";
                            ESP_LOGI(TAG_MB, "Characteristic #%d %s (%s) value = %s (0x%x) read successful.",
                                     param_descriptor->cid,
                                     (char*)param_descriptor->param_key,
                                     (char*)param_descriptor->param_units,
                                     (const char*)rw_str,
                                     *(uint16_t*)temp_data_ptr);
                            if (state & param_descriptor->param_opts.opt1) {
                                alarm_state = true;
                                break;
                            }
                        }
                    } else {
                        ESP_LOGE(TAG_MB, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                 param_descriptor->cid,
                                 (char*)param_descriptor->param_key,
                                 (int)err,
                                 (char*)esp_err_to_name(err));
                    }
                }
                vTaskDelay(POLL_TIMEOUT_TICS); // timeout between polls
            }
        }
        vTaskDelay(UPDATE_CIDS_TIMEOUT_TICS); //
    }

    if (alarm_state) {
        ESP_LOGI(TAG_MB, "Alarm triggered by cid #%d.",
                 param_descriptor->cid);
    } else {
        ESP_LOGE(TAG_MB, "Alarm is not triggered after %d retries.",
                 MASTER_MAX_RETRY);
    }
    ESP_LOGI(TAG_MB, "Destroy master...");
    ESP_ERROR_CHECK(mbc_master_destroy());
}

// Modbus master initialization
static esp_err_t master_init(void)
{
    // Initialize and start Modbus controller
    mb_communication_info_t comm = {
            .port = MB_PORT_NUM,
#if CONFIG_MB_COMM_MODE_ASCII
            .mode = MB_MODE_ASCII,
#elif CONFIG_MB_COMM_MODE_RTU
            .mode = MB_MODE_RTU,
#endif
            .baudrate = MB_DEV_SPEED,
            .parity = MB_PARITY_NONE
    };
    void* master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MB_RETURN_ON_FALSE((master_handler != NULL), ESP_ERR_INVALID_STATE, TAG_MB,
                       "mb controller initialization fail.");
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG_MB,
                       "mb controller initialization fail, returns(0x%x).",
                       (uint32_t)err);
    err = mbc_master_setup((void*)&comm);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG_MB,
                       "mb controller setup fail, returns(0x%x).",
                       (uint32_t)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, CONFIG_MB_UART_TXD, CONFIG_MB_UART_RXD,
                       CONFIG_MB_UART_RTS, UART_PIN_NO_CHANGE);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG_MB,
                       "mb serial set pin failure, uart_set_pin() returned (0x%x).", (uint32_t)err);

    err = mbc_master_start();
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG_MB,
                       "mb controller start fail, returns(0x%x).",
                       (uint32_t)err);

    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG_MB,
                       "mb serial set mode failure, uart_set_mode() returned (0x%x).", (uint32_t)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MB_RETURN_ON_FALSE((err == ESP_OK), ESP_ERR_INVALID_STATE, TAG_MB,
                       "mb controller set descriptor fail, returns(0x%x).",
                       (uint32_t)err);
    ESP_LOGI(TAG_MB, "Modbus master stack initialized...");
    return err;
}

void app_main(void)
{
    ESP_ERROR_CHECK(master_init());
    // Initialize Ethernet driver
    uint8_t eth_port_cnt = 0;
    esp_eth_handle_t *eth_handles;
    ESP_ERROR_CHECK(eth_init(&eth_handles, &eth_port_cnt));

    // Initialize TCP/IP network interface aka the esp-netif (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create instance(s) of esp-netif for Ethernet(s)
    if (eth_port_cnt == 1) {
        // Use ESP_NETIF_DEFAULT_ETH when just one Ethernet interface is used and you don't need to modify
        // default esp-netif configuration parameters.
        esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
        esp_netif_t *eth_netif = esp_netif_new(&cfg);
        // Attach Ethernet driver to TCP/IP stack
        ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handles[0])));
    } else {
        // Use ESP_NETIF_INHERENT_DEFAULT_ETH when multiple Ethernet interfaces are used and so you need to modify
        // esp-netif configuration parameters for each interface (name, priority, etc.).
        esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
        esp_netif_config_t cfg_spi = {
            .base = &esp_netif_config,
            .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
        };
        char if_key_str[10];
        char if_desc_str[10];
        char num_str[3];
        for (int i = 0; i < eth_port_cnt; i++) {
            itoa(i, num_str, 10);
            strcat(strcpy(if_key_str, "ETH_"), num_str);
            strcat(strcpy(if_desc_str, "eth"), num_str);
            esp_netif_config.if_key = if_key_str;
            esp_netif_config.if_desc = if_desc_str;
            esp_netif_config.route_prio -= i*5;
            esp_netif_t *eth_netif = esp_netif_new(&cfg_spi);

            // Attach Ethernet driver to TCP/IP stack
            ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handles[i])));
        }
    }

    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    // Start Ethernet driver state machine
    for (int i = 0; i < eth_port_cnt; i++) {
        ESP_ERROR_CHECK(esp_eth_start(eth_handles[i]));
    }


    float value = 0;

    esp_err_t err = ESP_OK;
    const mb_parameter_descriptor_t* param_descriptor = NULL;
    err = mbc_master_get_cid_info(0, &param_descriptor);

    void* temp_data_ptr = master_get_param_data(param_descriptor);
    assert(temp_data_ptr);
    uint8_t type = 0;

    err = mbc_master_get_parameter(0, (char*)param_descriptor->param_key,
                                   (uint8_t*)&value, &type);

    if (err == ESP_OK) {
        *(float*)temp_data_ptr = value;
        if ((param_descriptor->mb_param_type == MB_PARAM_HOLDING) ||
            (param_descriptor->mb_param_type == MB_PARAM_INPUT)) {
            ESP_LOGI(TAG_MB, "Characteristic #%d %s (%s) value = %f (0x%x) read successful.",
                     param_descriptor->cid,
                     (char*)param_descriptor->param_key,
                     (char*)param_descriptor->param_units,
                     value,
                     *(uint32_t*)temp_data_ptr);
        } else {
            uint16_t state = *(uint16_t*)temp_data_ptr;
            const char* rw_str = (state & param_descriptor->param_opts.opt1) ? "ON" : "OFF";
            ESP_LOGI(TAG_MB, "Characteristic #%d %s (%s) value = %s (0x%x) read successful.",
                     param_descriptor->cid,
                     (char*)param_descriptor->param_key,
                     (char*)param_descriptor->param_units,
                     (const char*)rw_str,
                     *(uint16_t*)temp_data_ptr);
        }
    } else {
        ESP_LOGE(TAG_MB, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                 param_descriptor->cid,
                 (char*)param_descriptor->param_key,
                 (int)err,
                 (char*)esp_err_to_name(err));
    }
}
