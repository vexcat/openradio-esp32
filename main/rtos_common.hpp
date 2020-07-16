#pragma once
#include <functional>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void CPPTaskWrapper(void* param);
TaskHandle_t launchCPPTask(std::function<void()> owo, const char* name, int depth);
std::string to_string(int a);
void esp_oops(esp_err_t err);
