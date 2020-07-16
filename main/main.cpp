#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "openradio.hpp"
#include "nvs_flash.h"
#include "../my_wifi_password.h"

static const int RX_BUF_SIZE = 1024;

static const uart_config_t uartConfig = {
    .baud_rate = 892858,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SCLK_APB
};

EventGroupHandle_t wifiEventGroup;
#define WIFIEVT_CONNECTED BIT0

void wifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifiEventGroup, WIFIEVT_CONNECTED);
        esp_wifi_connect();
        puts("Retrying wifi...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        printf("Got IP: " IPSTR "\n", IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifiEventGroup, WIFIEVT_CONNECTED);
    }
}

void initWifi() {
    wifiEventGroup = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifiEventHandler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifiEventHandler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {  };
    strcpy((char*)wifi_config.sta.ssid, "NETGEAR30");
    strcpy((char*)wifi_config.sta.password, MY_WIFI_PASSWORD);
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;


    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());

    puts("Done initializing wifi.\n");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    xEventGroupWaitBits(wifiEventGroup,
            WIFIEVT_CONNECTED,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    printf("connected to AP, SSID: %s Password: %s\n", wifi_config.sta.ssid, wifi_config.sta.password);
}

void init(void) {
    initWifi();
    

    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uartConfig);
    uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX);
    uart_set_pin(UART_NUM_1, 21, 18, 19, UART_PIN_NO_CHANGE);

    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uartConfig);
    uart_set_mode(UART_NUM_2, UART_MODE_RS485_HALF_DUPLEX);
    uart_set_pin(UART_NUM_2, 32, 35, 33, UART_PIN_NO_CHANGE);
}

extern "C" {
    void app_main(void) {
        //Initialize NVS
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        ESP_ERROR_CHECK(esp_event_loop_create_default());

        init();

        init_openradio(UART_NUM_2, UART_NUM_2);
    }
}