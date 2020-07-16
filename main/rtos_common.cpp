#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "sdkconfig.h"
#include <functional>
#include <stdexcept>
#include <string>
#include <sstream>
#include <vector>

void CPPTaskWrapper(void* param) {
    auto taskf = ((std::function<void()>*)param);
    auto name = pcTaskGetTaskName(NULL);
    try {
        printf("Task %s started\n", name);
        (*taskf)();
        delete taskf;
        printf("Task %s done\n", name);
        vTaskDelete(NULL);
    } catch(const std::exception& kami_no_manimani) {
        printf("A C++ exception occurred in task %s: %s\n", name, kami_no_manimani.what());
    } catch(...) {
        printf("Caught an unknown error\n");
    }
}

TaskHandle_t launchCPPTask(std::function<void()> owo, const char* name, int depth) {
    TaskHandle_t taskHandle = NULL;
    std::function<void()>* taskFunc = new std::function<void()>(owo);
    xTaskCreate(CPPTaskWrapper, name, depth, taskFunc, 5, &taskHandle);
    if (taskHandle == NULL) {
        ESP_LOGE("launchCPPTask", "Failed to create task %s!", name);
        esp_restart();
    }
    return taskHandle;
}

//std::to_string apparently doesn't exist????
std::string to_string(int a) {
    std::ostringstream ss;
    ss << a;
    return ss.str();
}

void esp_oops(esp_err_t err) {
    if(err != ESP_OK) {
        printf("OOPSIE WOOPSIE!! UwU We made a fucky wucky!! A wittle fucko boingo! The code monkeys at our headquarters are working VEWY HAWD to fix this!\n");
        printf("Got an ESP_ error: %d\n", err);
        throw std::runtime_error("ESP_: " + to_string((int)err));
    }
}