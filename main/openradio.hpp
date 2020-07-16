#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

//Initializes the openradio dæmon.
void init_openradio(uart_port_t rxPort, uart_port_t txPort);