/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/*
 * The following example demonstrates how to use the alert and bus recovery
 * features of the TWAI driver. The example will do the following:
 * 1) Install and start the TWAI driver
 * 2) Have the TX task periodically broadcast messages expecting no ACK
 * 3) Reconfigure alerts to detect bus-off state
 * 4) Trigger bus errors by inverting TX GPIO
 * 5) Initiate bus-off recovery and wait for completion
 * 6) Uninstall TWAI driver
 */

#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "soc/gpio_sig_map.h"   // For GPIO matrix signal index
#include "string.h"

/* --------------------- Definitions and static variables ------------------ */
// Example Configuration
#define TX_GPIO_NUM GPIO_NUM_33
#define RX_GPIO_NUM GPIO_NUM_32
#define TX_TASK_PRIO 9
#define CTRL_TASK_PRIO 11

#define EXAMPLE_TAG "TWAI Alert and Recovery"

#define TAG EXAMPLE_TAG

struct _twai_alert_name_t {
    const uint32_t alert;
    const char *name;
    const bool report;
} static const _alert_name_list[]{
    // clang-format off
     {TWAI_ALERT_TX_IDLE,                "TX_IDLE",              false},
     {TWAI_ALERT_TX_SUCCESS,             "TX_SUCCESS",           false},
     {TWAI_ALERT_RX_DATA,                "RX_DATA",              false},
     {TWAI_ALERT_BELOW_ERR_WARN,         "BELOW_ERR_WARN",       false},
     {TWAI_ALERT_ERR_ACTIVE,             "ERR_ACTIVE",           false},
     {TWAI_ALERT_RECOVERY_IN_PROGRESS,   "RECOVERY_IN_PROGRESS", true},
     {TWAI_ALERT_BUS_RECOVERED,          "BUS_RECOVERED",        true},
     {TWAI_ALERT_ARB_LOST,               "ARB_LOST",             false},
     {TWAI_ALERT_ABOVE_ERR_WARN,         "ABOVE_ERR_WARN",       false},
     {TWAI_ALERT_BUS_ERROR,              "BUS_ERROR",            false},
     {TWAI_ALERT_TX_FAILED,              "TX_FAILED",            true},
     {TWAI_ALERT_RX_QUEUE_FULL,          "RX_QUEUE_FULL",        true},
     {TWAI_ALERT_ERR_PASS,               "ERR_PASS",             true},
     {TWAI_ALERT_BUS_OFF,                "BUS_OFF",              true},
     {TWAI_ALERT_RX_FIFO_OVERRUN,        "RX_FIFO_OVERRUN",      true},
     {TWAI_ALERT_TX_RETRIED,             "TX_RETRIED",           true},
     {TWAI_ALERT_PERIPH_RESET,           "PERIPH_RESET",         true},
    // clang-format on
};
static const uint32_t _alert_name_list_size =
    sizeof(_alert_name_list) / sizeof(_alert_name_list[0]);

static void can_messages_router_print_status(twai_status_info_t status_info,
                                             esp_log_level_t log_level) {
    ESP_LOG_LEVEL_LOCAL(log_level, TAG,
                        "TWAI State: %u\n\r \tmsgs_to_tx: %lu\n\r \tmsgs_to_rx: "
                        "%lu\n\r \ttx_error_counter: %lu\n\r \trx_error_counter: %lu\n\r "
                        "\ttx_failed_count: %lu\n\r \trx_missed_count: %lu\n\r \trx_overrun_count: "
                        "%lu\n\r \tarb_lost_count: %lu\n\r \tbus_error_count: %lu",
                        status_info.state, status_info.msgs_to_tx, status_info.msgs_to_rx,
                        status_info.tx_error_counter, status_info.rx_error_counter,
                        status_info.tx_failed_count, status_info.rx_missed_count,
                        status_info.rx_overrun_count, status_info.arb_lost_count,
                        status_info.bus_error_count);
}

// CAN Settings
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
static const twai_timing_config_t t_config = []() {
    twai_timing_config_t config = TWAI_TIMING_CONFIG_125KBITS();
    config.clk_src = TWAI_CLK_SRC_DEFAULT;
    return config;
}();
#pragma GCC diagnostic pop

static const twai_general_config_t g_config = {.mode = TWAI_MODE_NORMAL,
                                               .tx_io = TX_GPIO_NUM,
                                               .rx_io = RX_GPIO_NUM,
                                               .clkout_io = TWAI_IO_UNUSED,
                                               .bus_off_io = TWAI_IO_UNUSED,
                                               .tx_queue_len = 20,
                                               .rx_queue_len = 20,
                                               .alerts_enabled = TWAI_ALERT_ALL,
                                               .clkout_divider = 0,
                                               .intr_flags = ESP_INTR_FLAG_LEVEL1};

static const twai_message_t tx_msg = {.flags = TWAI_MSG_FLAG_EXTD,
                                      .identifier = 0x5000,
                                      .data_length_code = 8,
                                      .data = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef}};

static SemaphoreHandle_t tx_task_sem;
static SemaphoreHandle_t ctrl_task_sem;

/* --------------------------- Tasks and Functions -------------------------- */

// TX Task to continuously transmit messages.
static void tx_task(void *arg) {
    xSemaphoreTake(tx_task_sem, portMAX_DELAY);
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while (1) {
        if (twai_transmit(&tx_msg, pdMS_TO_TICKS(100)) == ESP_ERR_INVALID_STATE) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;   // Just try to continuously transmit in 100ms interval
        }
        vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(100));
        pxPreviousWakeTime = xTaskGetTickCount();
    }
    vTaskDelete(NULL);
}

// Control task to check for errors and recover and restart the CAN-Bus when reaching a BUS_OFF
// condition.
static void ctrl_task(void *arg) {
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");
    ESP_LOGI(EXAMPLE_TAG, "Starting transmissions");
    xSemaphoreGive(tx_task_sem);   // Start transmit task

    twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL);

    esp_err_t alertStatus;
    uint32_t alerts;
    bool stats_printed = false;
    twai_status_info_t status_info;

    while (1) {
        // Then check if there are can errors logged
        alerts = 0;
        stats_printed = false;
        alertStatus = twai_read_alerts(&alerts, portMAX_DELAY);

        if (alertStatus == ESP_OK) {
            if (alerts != 0) {
                // Print all raised alerts, if reporting of the occurred alert is enabled
                for (uint32_t i = 0; i < _alert_name_list_size; ++i) {
                    if (alerts & _alert_name_list[i].alert) {
                        if (_alert_name_list[i].report) {
                            // First print current CAN Status, if there was an issue, but only
                            // do this once each round
                            if (stats_printed == false) {
                                esp_err_t res = twai_get_status_info(&status_info);
                                if (res == ESP_OK) {
                                    // Print TWAI status
                                    can_messages_router_print_status(status_info, ESP_LOG_WARN);
                                    stats_printed = true;
                                } else {
                                    // Print error
                                    ESP_LOGW(TAG, "Could not get twai status: %s.",
                                             esp_err_to_name(res));
                                }
                            }
                            // Then print the alert cause
                            ESP_LOGE(TAG, "!!! ALERT !!!: %s (%lx/%lx)", _alert_name_list[i].name,
                                     _alert_name_list[i].alert, alerts);

                        } else {
                            ESP_LOGD(TAG, "!!! ALERT !!!: %s (%lx/%lx)", _alert_name_list[i].name,
                                     _alert_name_list[i].alert, alerts);
                        }
                    }
                }

                if (alerts & TWAI_ALERT_BUS_OFF) {
                    ESP_LOGI(EXAMPLE_TAG, "Bus Off state");
                    // Prepare to initiate bus recovery, reconfigure alerts to detect bus recovery
                    // completion
                    // twai_reconfigure_alerts(TWAI_ALERT_BUS_RECOVERED, NULL);
                    for (int i = 3; i > 0; i--) {
                        ESP_LOGW(EXAMPLE_TAG, "Initiate bus recovery in %d", i);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                    twai_initiate_recovery();   // Needs 128 occurrences of bus free signal
                    ESP_LOGI(EXAMPLE_TAG, "Initiate bus recovery");
                }
                if (alerts & TWAI_ALERT_BUS_RECOVERED) {
                    // Bus recovery was successful, exit control task to uninstall driver
                    ESP_LOGI(EXAMPLE_TAG, "Bus Recovered");

                    ESP_ERROR_CHECK(twai_start());
                    ESP_LOGI(EXAMPLE_TAG, "Driver started again");
                    continue;
                }
            }
        }
    }

    xSemaphoreGive(ctrl_task_sem);
    vTaskDelete(NULL);
}

// Sample data sent by PCAN in 10ms interval
uint8_t sample_data[8] = {0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89};

// Check if the received data matches the sample data
static void _check_my_message(twai_message_t *canMessage) {
    static uint32_t message_cnt = 0;
    if (canMessage->identifier == 0x10000) {
        if (canMessage->data_length_code != 8) {
            goto error;
        }
        if (memcmp(canMessage->data, sample_data, sizeof(sample_data)) == 0) {
            // Everything fine
            message_cnt++;
            return;
        } else {
            // The received message does not match the sample buffer => Print the corrupt message
            goto error;
        }
    }
error:
    if (canMessage != NULL) {
        ESP_LOGE(TAG,
                 "\tMessage ID: 0x%lx (%li), len: %i, data: %02X %02X %02X %02X %02X %02X "
                 "%02X %02X, msg cnt: %lu",
                 canMessage->identifier, canMessage->identifier, canMessage->data_length_code,
                 canMessage->data[0], canMessage->data[1], canMessage->data[2], canMessage->data[3],
                 canMessage->data[4], canMessage->data[5], canMessage->data[6], canMessage->data[7],
                 message_cnt);
    }
    return;
}

// RX Task to read messages from TWAI receive queue
static void rx_task(void *arg) {
    ESP_LOGI(TAG, "Receive Task started");

    esp_err_t receiveStatus;
    twai_message_t message;
    while (1) {
        receiveStatus = twai_receive(&message, pdMS_TO_TICKS(1000));

        switch (receiveStatus) {
            case ESP_OK: {
                // Handle Message
                switch (message.identifier) {
                    case 0x10000: {
                        // compare to other message
                        _check_my_message(&message);
                        break;
                    }
                    default: {
                        ESP_LOGE(TAG,
                                 "\tMessage ID: 0x%lx (%li), len: %i, data: %02X %02X %02X "
                                 "%02X %02X %02X "
                                 "%02X %02X",
                                 message.identifier, message.identifier, message.data_length_code,
                                 message.data[0], message.data[1], message.data[2], message.data[3],
                                 message.data[4], message.data[5], message.data[6],
                                 message.data[7]);
                        break;
                    }
                }
                break;
            }

            case ESP_ERR_TIMEOUT: {
                ESP_LOGE(TAG, "CAN receive timed out");
                break;
            }

            default: {
                ESP_LOGE(TAG, "Error receiving Message: %s", esp_err_to_name(receiveStatus));
                break;
            }
        }
    }

    vTaskDelete(NULL);
}

// Start Tasks and install drivers
extern "C" void app_main(void) {
    tx_task_sem = xSemaphoreCreateBinary();
    ctrl_task_sem = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(tx_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, TX_TASK_PRIO % 2);
    xTaskCreatePinnedToCore(rx_task, "TWAI_rx", 4096, NULL, TX_TASK_PRIO, NULL, TX_TASK_PRIO % 2);
    xTaskCreatePinnedToCore(ctrl_task, "TWAI_ctrl", 4096, NULL, CTRL_TASK_PRIO, NULL,
                            CTRL_TASK_PRIO % 2);

    // Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");

    xSemaphoreGive(ctrl_task_sem);   // Start control task
    vTaskDelay(pdMS_TO_TICKS(100));

    // Never get this semaphore...
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);   // Wait for completion

    // Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    // Cleanup
    vSemaphoreDelete(tx_task_sem);
    vSemaphoreDelete(ctrl_task_sem);
}

static void _twai_message_checker(twai_message_t *canMessage) {
    static const uint8_t kExpected[8] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
    if (canMessage->identifier == 0x01) {
        if (memcmp(canMessage->data, kExpected, sizeof(kExpected)) == 0) {
            // Got correct message
            ESP_LOGD(TAG,
                     "Got correct message ID: %lu Data: %02x %02x %02x %02x %02x %02x %02x %02x",
                     canMessage->identifier, canMessage->data[0], canMessage->data[1],
                     canMessage->data[2], canMessage->data[3], canMessage->data[4],
                     canMessage->data[5], canMessage->data[6], canMessage->data[7]);
        } else {
            // Got incorrect message
            ESP_LOGE(TAG, "Got wrong message ID: %lu Data: %02x %02x %02x %02x %02x %02x %02x %02x",
                     canMessage->identifier, canMessage->data[0], canMessage->data[1],
                     canMessage->data[2], canMessage->data[3], canMessage->data[4],
                     canMessage->data[5], canMessage->data[6], canMessage->data[7]);
        }

    } else {
        ESP_LOGW(TAG, "Got message ID: %lu Data: %02x %02x %02x %02x %02x %02x %02x %02x",
                 canMessage->identifier, canMessage->data[0], canMessage->data[1],
                 canMessage->data[2], canMessage->data[3], canMessage->data[4], canMessage->data[5],
                 canMessage->data[6], canMessage->data[7]);
    }
    return;
}
